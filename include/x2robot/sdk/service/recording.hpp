#pragma once
#include "v2/sensors/recording.grpc.pb.h"
#include "x2robot/sdk/service/grpc_service.hpp"
namespace x2robot::sdk {
class Recording final : public client::detail::GrpcServiceClient<::xr::sdk::v2::Recording>
{
    using Service = ::xr::sdk::v2::Recording;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit Recording(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }
    SdkResult<::xr::sdk::v2::RecordingStatus> StartRecording(const ::xr::sdk::v2::StartRecordingRequest& request,
                                                             Deadline deadline = Deadline{10000}) const
    {
        return Unary<::xr::sdk::v2::RecordingStatus, ::xr::sdk::v2::RecordingResult>(request, &Stub::StartRecording, deadline, "StartRecording");
    }
    using Status = ::xr::sdk::v2::RecordingStatus;
    using Callback = std::function<void(const SdkResult<Status>&)>;
    // The callback receives the accepted state, then changed states/errors polled
    // every 5 seconds. Each status query has its own 30-second default deadline.
    // RPC failures have Ok()==false; recording failures have
    // Ok()==true and state()=="FAILED" (or STOPPING with an error_code).
    SdkResult<Status> StartRecording(const ::xr::sdk::v2::StartRecordingRequest& request,
                                     Callback callback,
                                     Deadline poll_interval = Deadline{5000},
                                     Deadline deadline = Deadline{10000},
                                     Deadline status_deadline = Deadline{30000}) const
    {
        if (!callback || poll_interval.count() <= 0 || status_deadline.count() <= 0)
            return SdkResult<Status>::Err(MakeClientError(errc::kInvalidArgument, "callback, poll interval and status deadline must be valid"));
        auto started = StartRecording(request, deadline);
        if (!started.Ok()) return started;  // A rejected START never creates a monitor.
        auto context = LockContext();
        if (!context) return started;
        auto monitor = context->StartPolling(
            "recording/" + request.session_id(), poll_interval,
            [recording = *this, id = request.session_id(), callback, initial = std::optional<SdkResult<Status>>(started), previous = std::string{},
             status_deadline](const std::atomic<bool>& cancelled) mutable {
                auto result = initial ? std::move(*initial) : recording.GetRecordingStatus(id, status_deadline);
                initial.reset();
                if (cancelled) return false;
                const auto signature = result.Ok()
                    ? "status:" + result.Value().SerializeAsString()
                    : "error:" + std::to_string(result.ErrorValue().error_code) + ":" + result.ErrorValue().error_message;
                if (signature != previous)
                {
                    previous = signature;
                    callback(result);
                }
                return !result.Ok() || (result.Value().state() != "READY" && result.Value().state() != "FAILED" &&
                                        result.Value().state() != "DELETED" && result.Value().state() != "DELETING");
            });
        // START remains authoritative if closing races with monitor setup.
        if (!monitor.Ok() && !context->IsClosed()) {
            try { callback(SdkResult<Status>::Err(monitor.ErrorValue(), monitor.request_id)); }
            catch (...) { client::LogRpcError("Recording.callback", monitor.request_id,
                MakeClientError(errc::kInternal, "recording callback threw during monitor setup")); }
        }
        return started;
    }
    // Stops notifications only; the recorder continues until StopRecording or its safety guard.
    void CancelRecordingCallback(const std::string& session_id) const
    {
        if (auto context = LockContext()) context->CancelPolling("recording/" + session_id);
    }
    SdkResult<::xr::sdk::v2::RecordingStatus> StopRecording(const std::string& session_id, Deadline deadline = Deadline{10000}) const
    {
        ::xr::sdk::v2::RecordingSessionRequest request;
        request.set_session_id(session_id);
        return Unary<::xr::sdk::v2::RecordingStatus, ::xr::sdk::v2::RecordingResult>(request, &Stub::StopRecording, deadline, "StopRecording");
    }
    // Explicit cleanup; only call automatically after a verified local download.
    SdkResult<Status> DeleteRecording(const std::string& session_id, Deadline deadline = Deadline{30000}) const
    {
        ::xr::sdk::v2::RecordingSessionRequest request;
        request.set_session_id(session_id);
        return Unary<Status, ::xr::sdk::v2::RecordingResult>(request, &Stub::DeleteRecording, deadline, "DeleteRecording");
    }
    SdkResult<::xr::sdk::v2::RecordingStatus> GetRecordingStatus(const std::string& session_id = {}, Deadline deadline = Deadline{30000}) const
    {
        ::xr::sdk::v2::RecordingSessionRequest request;
        request.set_session_id(session_id);
        return Unary<::xr::sdk::v2::RecordingStatus, ::xr::sdk::v2::RecordingResult>(request, &Stub::GetRecordingStatus, deadline,
                                                                                     "GetRecordingStatus");
    }
};
}  // namespace x2robot::sdk
