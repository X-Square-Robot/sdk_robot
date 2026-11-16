#pragma once

#include "x2robot/sdk/core/deadline.hpp"

#include "x2robot/sdk/core/model_capabilities.hpp"
#include "x2robot/sdk/core/service_context.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <grpcpp/grpcpp.h>

#include "x2robot/sdk/request_id.hpp"
#include "x2robot/sdk/result.hpp"
#include "x2robot/sdk/result_proto.hpp"
#include "x2robot/sdk/types.hpp"

namespace x2robot::sdk {

// All streaming APIs use the same result contract as unary APIs.  Normal
// lifetime is controlled by the handle: destruction or Cancel() cancels the
// corresponding gRPC request and lets the server release its subscription.
template <typename T>
class ResultStream
{
   public:
    virtual ~ResultStream() = default;
    virtual SdkResult<T> Read(Deadline deadline = Deadline{30000}) = 0;
    virtual SdkResult<void> Cancel() = 0;
};

template <typename Request, typename T>
class BidiResultStream : public ResultStream<T>
{
   public:
    virtual SdkResult<void> Write(const Request& request) = 0;
    virtual SdkResult<void> WritesDone() = 0;
};

template <typename T>
using ResultStreamHandle = std::unique_ptr<ResultStream<T>>;

template <typename Request, typename T>
using BidiResultStreamHandle = std::unique_ptr<BidiResultStream<Request, T>>;

namespace client {

namespace detail {

inline int32_t FromGrpcStatusCode(grpc::StatusCode code)
{
    switch (code)
    {
        case grpc::StatusCode::INVALID_ARGUMENT:
            return errc::kInvalidArgument;
        case grpc::StatusCode::PERMISSION_DENIED:
        case grpc::StatusCode::UNAUTHENTICATED:
            return errc::kUnauthenticated;
        case grpc::StatusCode::CANCELLED:
            return errc::kCancelled;
        case grpc::StatusCode::DEADLINE_EXCEEDED:
            return errc::kTimeout;
        case grpc::StatusCode::UNIMPLEMENTED:
            return errc::kUnimplemented;
        case grpc::StatusCode::UNAVAILABLE:
            return errc::kUnavailable;
        default:
            return errc::kInternal;
    }
}

inline void ConfigureContext(grpc::ClientContext* context, const std::string& request_id, Deadline deadline)
{
    context->AddMetadata("x-request-id", request_id);
    context->set_deadline(std::chrono::system_clock::now() + deadline);
}

template <typename T>
SdkResult<T> TransportFailure(const grpc::Status& status, const std::string& request_id)
{
    const std::string message = status.error_message().empty() ? "gRPC transport failure" : status.error_message();
    return SdkResult<T>::Err(MakeClientError(FromGrpcStatusCode(status.error_code()), message), request_id);
}

inline ErrorCode ClosedError()
{
    return MakeClientError(errc::kUnavailable, "robot connection closed");
}

template <typename Response, typename T>
class GrpcResultStream final : public ResultStream<T>
{
   public:
    using Reader = grpc::ClientReaderInterface<Response>;
    using Open = std::function<std::unique_ptr<Reader>(grpc::ClientContext*)>;

    GrpcResultStream(Open open, std::string request_id, std::shared_ptr<ActiveCall> call)
        : request_id_(std::move(request_id)), call_(std::move(call))
    {
        call_->SetWakeup([this] { ready_.notify_all(); space_.notify_all(); });
        reader_ = open(&call_->context);
        if (reader_) worker_ = std::thread([this] { WorkerCompletion done(call_); Run(); });
        else call_->WorkerDone();
    }

    ~GrpcResultStream() override
    {
        (void)Cancel();
        if (worker_.joinable()) worker_.join();
        call_->SetWakeup({});
    }

    bool Valid() const noexcept
    {
        return reader_ != nullptr;
    }

    SdkResult<void> WaitFirst(Deadline deadline)
    {
        std::unique_lock lock(mutex_);
        if (!ready_.wait_for(lock, deadline, [this] { return !responses_.empty() || done_; }))
        {
            // An empty local wait window is not an RPC failure. Match the arm
            // stream contract: return TIMEOUT and let the caller decide how
            // to report it (optional recording streams may keep waiting).
            return SdkResult<void>::Err(MakeClientError(errc::kTimeout, "gRPC stream first response timeout"), request_id_);
        }
        if (responses_.empty()) return TerminalFailure();
        auto parsed = LogRpcResult(call_->method, FromProtoResult<T>(responses_.front()));
        if (!parsed.Ok())
        {
            return SdkResult<void>::Err(parsed.ErrorValue(), parsed.request_id);
        }
        return SdkResult<void>::Success(parsed.request_id);
    }

    SdkResult<T> Read(Deadline deadline) override
    {
        if (auto error = ValidateDeadline(deadline)) return SdkResult<T>::Err(*error, request_id_);
        std::unique_lock lock(mutex_);
        if (!ready_.wait_for(lock, deadline, [this] { return !responses_.empty() || done_; }))
        {
            return SdkResult<T>::Err(MakeClientError(errc::kTimeout, "gRPC stream read timeout"), request_id_);
        }
        if (responses_.empty())
        {
            auto failure = TerminalFailure();
            return SdkResult<T>::Err(failure.ErrorValue(), failure.request_id);
        }
        Response response = std::move(responses_.front());
        responses_.pop_front();
        lock.unlock();
        space_.notify_one();
        return LogRpcResult(call_->method, FromProtoResult<T>(response));
    }

    SdkResult<void> Cancel() override
    {
        {
            std::lock_guard lock(mutex_);
            cancelled_ = true;
        }
        call_->Cancel();
        ready_.notify_all();
        space_.notify_all();
        return SdkResult<void>::Success(request_id_);
    }

   private:
    SdkResult<void> TerminalFailure() const
    {
        if (!reader_)
        {
            return SdkResult<void>::Err(MakeClientError(errc::kInternal, "failed to create gRPC stream reader"), request_id_);
        }
        if (status_.ok())
        {
            return SdkResult<void>::Err(MakeClientError(errc::kCancelled, "gRPC stream ended"), request_id_);
        }
        return TransportFailure<void>(status_, request_id_);
    }

    void Run()
    {
        Response response;
        while (reader_->Read(&response))
        {
            std::unique_lock lock(mutex_);
            space_.wait(lock, [this] { return responses_.size() < kQueueDepth || cancelled_ || call_->cancelled; });
            if (cancelled_ || call_->cancelled) break;
            responses_.push_back(std::move(response));
            lock.unlock();
            ready_.notify_all();
            response.Clear();
        }
        grpc::Status status = reader_->Finish();
        LogRpcStatus(call_->method, request_id_, status, call_->cancelled);
        {
            std::lock_guard lock(mutex_);
            status_ = std::move(status);
            done_ = true;
        }
        ready_.notify_all();
        space_.notify_all();
    }

    static constexpr size_t kQueueDepth = 64;
    std::string request_id_;
    std::shared_ptr<ActiveCall> call_;
    std::unique_ptr<Reader> reader_;
    std::thread worker_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::condition_variable space_;
    std::deque<Response> responses_;
    grpc::Status status_;
    bool done_{false};
    bool cancelled_{false};
};

template <typename Request, typename Response, typename T>
class GrpcBidiResultStream final : public BidiResultStream<Request, T>
{
   public:
    using ReaderWriter = grpc::ClientReaderWriterInterface<Request, Response>;
    using Open = std::function<std::unique_ptr<ReaderWriter>(grpc::ClientContext*)>;

    GrpcBidiResultStream(Open open, std::string request_id, std::shared_ptr<ActiveCall> call)
        : request_id_(std::move(request_id)), call_(std::move(call))
    {
        call_->SetWakeup([this] { ready_.notify_all(); space_.notify_all(); });
        stream_ = open(&call_->context);
        if (stream_) worker_ = std::thread([this] { WorkerCompletion done(call_); Run(); });
        else call_->WorkerDone();
    }

    ~GrpcBidiResultStream() override
    {
        (void)Cancel();
        if (worker_.joinable()) worker_.join();
        call_->SetWakeup({});
    }

    bool Valid() const noexcept
    {
        return stream_ != nullptr;
    }

    SdkResult<void> Write(const Request& request) override
    {
        std::lock_guard lock(write_mutex_);
        if (!stream_ || writes_done_)
        {
            return SdkResult<void>::Err(MakeClientError(errc::kCancelled, "gRPC stream is not writable"), request_id_);
        }
        if (!stream_->Write(request))
        {
            return SdkResult<void>::Err(MakeClientError(errc::kUnavailable, "gRPC stream write failed"), request_id_);
        }
        return SdkResult<void>::Success(request_id_);
    }

    SdkResult<void> WritesDone() override
    {
        std::lock_guard lock(write_mutex_);
        if (!stream_ || writes_done_) return SdkResult<void>::Success(request_id_);
        writes_done_ = true;
        if (!stream_->WritesDone())
        {
            return SdkResult<void>::Err(MakeClientError(errc::kUnavailable, "gRPC WritesDone failed"), request_id_);
        }
        return SdkResult<void>::Success(request_id_);
    }

    SdkResult<T> Read(Deadline deadline) override
    {
        if (auto error = ValidateDeadline(deadline)) return SdkResult<T>::Err(*error, request_id_);
        std::unique_lock lock(read_mutex_);
        if (!ready_.wait_for(lock, deadline, [this] { return !responses_.empty() || done_; }))
        {
            return LogRpcResult(call_->method, SdkResult<T>::Err(MakeClientError(errc::kTimeout, "gRPC bidi stream read timeout"), request_id_));
        }
        if (responses_.empty())
        {
            if (status_.ok())
            {
                return SdkResult<T>::Err(MakeClientError(errc::kCancelled, "gRPC bidi stream ended"), request_id_);
            }
            return TransportFailure<T>(status_, request_id_);
        }
        Response response = std::move(responses_.front());
        responses_.pop_front();
        space_.notify_one();
        return LogRpcResult(call_->method, FromProtoResult<T>(response));
    }

    SdkResult<void> Cancel() override
    {
        call_->Cancel();
        ready_.notify_all();
        return SdkResult<void>::Success(request_id_);
    }

   private:
    void Run()
    {
        Response response;
        while (stream_->Read(&response))
        {
            {
                std::unique_lock lock(read_mutex_);
                space_.wait(lock, [this] { return responses_.size() < 64 || call_->cancelled; });
                if (call_->cancelled) break;
                responses_.push_back(std::move(response));
            }
            ready_.notify_all();
            response.Clear();
        }
        grpc::Status status = stream_->Finish();
        LogRpcStatus(call_->method, request_id_, status, call_->cancelled);
        {
            std::lock_guard lock(read_mutex_);
            status_ = std::move(status);
            done_ = true;
        }
        ready_.notify_all();
    }

    std::string request_id_;
    std::shared_ptr<ActiveCall> call_;
    std::unique_ptr<ReaderWriter> stream_;
    std::thread worker_;
    std::mutex write_mutex_;
    std::mutex read_mutex_;
    std::condition_variable ready_, space_;
    std::deque<Response> responses_;
    grpc::Status status_;
    bool writes_done_{false};
    bool done_{false};
};

template <typename Service>
class GrpcServiceClient
{
   protected:
    using Stub = typename Service::StubInterface;

    explicit GrpcServiceClient(std::weak_ptr<ServiceContext> context) : context_(std::move(context))
    {
        if (auto locked = context_.lock(); locked && !locked->IsClosed())
        {
            const std::string_view full = Service::service_full_name();
            const auto name = full.substr(full.rfind('.') + 1);
            if (!locked->IsEx002() || Ex002SupportsService(name))
                stub_ = Service::NewStub(locked->Channel());
        }
    }

    std::shared_ptr<ServiceContext> LockContext() const { return context_.lock(); }

    template <typename T, typename Response, typename Request, typename Method>
    SdkResult<T> Unary(const Request& request, Method method, Deadline deadline, std::string_view method_name = {}) const
    {
        const auto request_id = NewRequestId();
        if (auto error = ValidateDeadline(deadline)) return SdkResult<T>::Err(*error, request_id);
        auto context = context_.lock();
        if (!context || context->IsClosed() || !stub_)
        {
            return LogRpcResult(std::string(Service::service_full_name()) + "/" + std::string(method_name), SdkResult<T>::Err(AvailabilityError(context), request_id));
        }
        auto call = context->BeginCall(std::string(Service::service_full_name()) + "/" + std::string(method_name), request_id, deadline);
        if (!call.Ok()) return SdkResult<T>::Err(call.ErrorValue(), request_id);
        auto& call_context = call.Value()->context;
        Response response;
        const grpc::Status status = ((*stub_).*method)(&call_context, request, &response);
        LogRpcStatus(call.Value()->method, request_id, status);
        if (!status.ok()) return TransportFailure<T>(status, request_id);
        return LogRpcResult(call.Value()->method, FromProtoResult<T>(response));
    }

    // Most result messages carry a message-valued `data` field and are handled
    // by Unary(). A small number of protocol-defined exceptions carry a scalar
    // field directly (RobotStatusResult.json, for example). Keep the transport
    // and result-header handling shared while letting the service select that
    // scalar field explicitly.
    template <typename T, typename Response, typename Request, typename Method, typename Extractor>
    SdkResult<T> UnaryScalar(const Request& request, Method method, Deadline deadline, Extractor extractor, std::string_view method_name = {}) const
    {
        const auto request_id = NewRequestId();
        if (auto error = ValidateDeadline(deadline)) return SdkResult<T>::Err(*error, request_id);
        auto context = context_.lock();
        if (!context || context->IsClosed() || !stub_)
        {
            return LogRpcResult(std::string(Service::service_full_name()) + "/" + std::string(method_name), SdkResult<T>::Err(AvailabilityError(context), request_id));
        }
        auto call = context->BeginCall(std::string(Service::service_full_name()) + "/" + std::string(method_name), request_id, deadline);
        if (!call.Ok()) return SdkResult<T>::Err(call.ErrorValue(), request_id);
        auto& call_context = call.Value()->context;
        Response response;
        const grpc::Status status = ((*stub_).*method)(&call_context, request, &response);
        LogRpcStatus(call.Value()->method, request_id, status);
        if (!status.ok()) return TransportFailure<T>(status, request_id);
        if (!response.has_header())
        {
            return SdkResult<T>::Err(MakeClientError(errc::kInternal, "服务端响应缺少 header 字段"), request_id);
        }
        const auto& header = response.header();
        if (!header.is_success())
        {
            auto error = header.has_error() ? FromProtoError(header.error()) : MakeClientError(errc::kInternal, "服务端失败响应缺少 error 字段");
            return LogRpcResult(call.Value()->method, SdkResult<T>::Err(std::move(error), header.request_id()));
        }
        return SdkResult<T>::Success(extractor(response), header.request_id());
    }

    template <typename Response, typename Request, typename Method>
    SdkResult<void> UnaryVoid(const Request& request, Method method, Deadline deadline, std::string_view method_name = {}) const
    {
        const auto request_id = NewRequestId();
        if (auto error = ValidateDeadline(deadline)) return SdkResult<void>::Err(*error, request_id);
        auto context = context_.lock();
        if (!context || context->IsClosed() || !stub_)
        {
            return LogRpcResult(std::string(Service::service_full_name()) + "/" + std::string(method_name), SdkResult<void>::Err(AvailabilityError(context), request_id));
        }
        auto call = context->BeginCall(std::string(Service::service_full_name()) + "/" + std::string(method_name), request_id, deadline);
        if (!call.Ok()) return SdkResult<void>::Err(call.ErrorValue(), request_id);
        auto& call_context = call.Value()->context;
        Response response;
        const grpc::Status status = ((*stub_).*method)(&call_context, request, &response);
        LogRpcStatus(call.Value()->method, request_id, status);
        if (!status.ok()) return TransportFailure<void>(status, request_id);
        return LogRpcResult(call.Value()->method, FromProtoResult(response));
    }

    template <typename T, typename Response, typename Request, typename Method>
    SdkResult<ResultStreamHandle<T>> ServerStream(const Request& request, Method method, Deadline first_deadline, std::string_view method_name = {}) const
    {
        const auto request_id = NewRequestId();
        if (auto error = ValidateDeadline(first_deadline)) return SdkResult<ResultStreamHandle<T>>::Err(*error, request_id);
        auto context = context_.lock();
        if (!context || context->IsClosed() || !stub_)
        {
            return SdkResult<ResultStreamHandle<T>>::Err(AvailabilityError(context), request_id);
        }
        auto call = context->BeginCall(std::string(Service::service_full_name()) + "/" + std::string(method_name), request_id, Deadline{0}, true);
        if (!call.Ok()) return SdkResult<ResultStreamHandle<T>>::Err(call.ErrorValue(), request_id);
        auto stub = stub_;
        auto stream = std::make_unique<GrpcResultStream<Response, T>>(
            [stub, request, method](grpc::ClientContext* call_context) { return ((*stub).*method)(call_context, request); }, request_id, call.Value());
        if (!stream->Valid())
        {
            return SdkResult<ResultStreamHandle<T>>::Err(MakeClientError(errc::kInternal, "failed to create gRPC stream"), request_id);
        }
        auto first = stream->WaitFirst(first_deadline);
        if (!first.Ok())
        {
            return SdkResult<ResultStreamHandle<T>>::Err(first.ErrorValue(), first.request_id);
        }
        return SdkResult<ResultStreamHandle<T>>::Success(ResultStreamHandle<T>(std::move(stream)), first.request_id);
    }

    // Read-style sensor APIs are request-scoped snapshots.  Use the matching
    // server-stream RPC for them as well: opening the RPC creates the ROS
    // subscription, the first response is the requested value, and cancelling
    // immediately releases the server-side subscription.  This keeps unary
    // and streaming sensor paths on one lifecycle without leaving a reader
    // alive between calls.
    template <typename T, typename Response, typename Request, typename Method>
    SdkResult<T> UnaryFromStream(const Request& request, Method method, Deadline first_deadline, std::string_view method_name = {}) const
    {
        auto stream = ServerStream<T, Response>(request, method, first_deadline, method_name);
        if (!stream.Ok())
        {
            return SdkResult<T>::Err(stream.ErrorValue(), stream.request_id);
        }

        auto first = stream.Value()->Read(first_deadline);
        auto cancelled = stream.Value()->Cancel();
        if (!first.Ok()) return first;
        if (!cancelled.Ok())
        {
            return SdkResult<T>::Err(cancelled.ErrorValue(), cancelled.request_id);
        }
        return first;
    }

    template <typename Request, typename Response, typename T, typename Method>
    SdkResult<T> ClientStream(const std::vector<Request>& requests, Method method, Deadline deadline, std::string_view method_name = {}) const
    {
        const auto request_id = NewRequestId();
        if (auto error = ValidateDeadline(deadline)) return SdkResult<T>::Err(*error, request_id);
        auto context = context_.lock();
        if (!context || context->IsClosed() || !stub_)
        {
            return LogRpcResult(std::string(Service::service_full_name()) + "/" + std::string(method_name), SdkResult<T>::Err(AvailabilityError(context), request_id));
        }
        auto call = context->BeginCall(std::string(Service::service_full_name()) + "/" + std::string(method_name), request_id, deadline);
        if (!call.Ok()) return SdkResult<T>::Err(call.ErrorValue(), request_id);
        auto& call_context = call.Value()->context;
        Response response;
        auto writer = ((*stub_).*method)(&call_context, &response);
        if (!writer)
        {
            return SdkResult<T>::Err(MakeClientError(errc::kInternal, "failed to create gRPC stream writer"), request_id);
        }
        for (const auto& request : requests)
        {
            if (!writer->Write(request))
            {
                writer->WritesDone();
                const auto status = writer->Finish();
                return status.ok() ? SdkResult<T>::Err(MakeClientError(errc::kUnavailable, "gRPC stream write failed"), request_id)
                                   : TransportFailure<T>(status, request_id);
            }
        }
        writer->WritesDone();
        const auto status = writer->Finish();
        LogRpcStatus(call.Value()->method, request_id, status);
        if (!status.ok()) return TransportFailure<T>(status, request_id);
        return LogRpcResult(call.Value()->method, FromProtoResult<T>(response));
    }

    template <typename Request, typename Response, typename Method>
    SdkResult<void> ClientStreamVoid(const std::vector<Request>& requests, Method method, Deadline deadline, std::string_view method_name = {}) const
    {
        const auto request_id = NewRequestId();
        if (auto error = ValidateDeadline(deadline)) return SdkResult<void>::Err(*error, request_id);
        auto context = context_.lock();
        if (!context || context->IsClosed() || !stub_)
        {
            return LogRpcResult(std::string(Service::service_full_name()) + "/" + std::string(method_name), SdkResult<void>::Err(AvailabilityError(context), request_id));
        }
        auto call = context->BeginCall(std::string(Service::service_full_name()) + "/" + std::string(method_name), request_id, deadline);
        if (!call.Ok()) return SdkResult<void>::Err(call.ErrorValue(), request_id);
        auto& call_context = call.Value()->context;
        Response response;
        auto writer = ((*stub_).*method)(&call_context, &response);
        if (!writer)
        {
            return SdkResult<void>::Err(MakeClientError(errc::kInternal, "failed to create gRPC stream writer"), request_id);
        }
        for (const auto& request : requests)
        {
            if (!writer->Write(request))
            {
                writer->WritesDone();
                const auto status = writer->Finish();
                return status.ok() ? SdkResult<void>::Err(MakeClientError(errc::kUnavailable, "gRPC stream write failed"), request_id)
                                   : TransportFailure<void>(status, request_id);
            }
        }
        writer->WritesDone();
        const auto status = writer->Finish();
        LogRpcStatus(call.Value()->method, request_id, status);
        if (!status.ok()) return TransportFailure<void>(status, request_id);
        return LogRpcResult(call.Value()->method, FromProtoResult(response));
    }

    template <typename Request, typename Response, typename T, typename Method>
    SdkResult<BidiResultStreamHandle<Request, T>> BidiStream(Method method, std::string_view method_name = {}) const
    {
        const auto request_id = NewRequestId();
        auto context = context_.lock();
        if (!context || context->IsClosed() || !stub_)
        {
            return SdkResult<BidiResultStreamHandle<Request, T>>::Err(AvailabilityError(context), request_id);
        }
        auto call = context->BeginCall(std::string(Service::service_full_name()) + "/" + std::string(method_name), request_id, Deadline{0}, true);
        if (!call.Ok()) return SdkResult<BidiResultStreamHandle<Request, T>>::Err(call.ErrorValue(), request_id);
        auto stub = stub_;
        auto stream = std::make_unique<GrpcBidiResultStream<Request, Response, T>>(
            [stub, method](grpc::ClientContext* call_context) { return ((*stub).*method)(call_context); }, request_id, call.Value());
        if (!stream->Valid())
        {
            return SdkResult<BidiResultStreamHandle<Request, T>>::Err(MakeClientError(errc::kInternal, "failed to create gRPC bidi stream"),
                                                                      request_id);
        }
        return SdkResult<BidiResultStreamHandle<Request, T>>::Success(BidiResultStreamHandle<Request, T>(std::move(stream)), request_id);
    }

    std::shared_ptr<ServiceContext> Context() const { return context_.lock(); }

   private:
    ErrorCode AvailabilityError(const std::shared_ptr<ServiceContext>& context) const {
        if (!context || context->IsClosed()) return ClosedError();
        return MakeClientError(errc::kUnimplemented,
            std::string(Service::service_full_name()) + " is not public for quanta_x3");
    }
    std::weak_ptr<ServiceContext> context_;
    std::shared_ptr<Stub> stub_;
};

}  // namespace detail
}  // namespace client
}  // namespace x2robot::sdk
