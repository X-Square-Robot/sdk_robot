#pragma once

#include <grpcpp/grpcpp.h>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <map>
#include <optional>
#include <thread>
#include <vector>
#include "v2/core/system.pb.h"
#include "x2robot/sdk/result.hpp"
#include "x2robot/sdk/core/logging.hpp"
#include "x2robot/sdk/types.hpp"

namespace x2robot::sdk::client {
// 调用自己持有 context；Robot 只登记弱引用，取消不影响其他连接。
struct ActiveCall {
    grpc::ClientContext context;
    std::atomic<bool> cancelled{false};
    const bool worker;
    std::string method, request_id;
    std::chrono::steady_clock::time_point started{std::chrono::steady_clock::now()};
    explicit ActiveCall(bool has_worker) : worker(has_worker) {}
    ~ActiveCall();
    void Cancel();
    void SetWakeup(std::function<void()> callback);
    void WorkerStarted();
    void WorkerDone();
    bool WaitWorker(std::chrono::steady_clock::time_point deadline);
 private:
    std::mutex mutex;
    std::condition_variable changed;
    std::function<void()> wakeup;
    std::thread::id worker_id;
    bool done{false};
};
struct WorkerCompletion {
    std::shared_ptr<ActiveCall> call;
    explicit WorkerCompletion(std::shared_ptr<ActiveCall> value) : call(std::move(value)) { call->WorkerStarted(); }
    ~WorkerCompletion() { call->WorkerDone(); }
};

class ServiceContext {
 public:
    explicit ServiceContext(std::shared_ptr<grpc::Channel> channel, std::string model = {}, bool authorized = false);
    ~ServiceContext();
    std::shared_ptr<grpc::Channel> Channel() const { return channel_; }
    const std::string& ModelName() const noexcept { return model_; }
    bool IsEx002() const noexcept { return model_ == "quanta_x3"; }
    bool IsClosed() const noexcept { return closing_.load(); }
    SdkResult<std::shared_ptr<ActiveCall>> BeginCall(std::string_view method, const std::string& request_id,
        Deadline deadline = Deadline{0}, bool worker = false, bool internal = false);
    SdkResult<void> SetWorkMode(const ::xr::sdk::v2::RobotModeParam& request, Deadline deadline,
        const std::string& request_id);
    // Connection-owned polling jobs; callbacks run on a worker, never the RPC caller.
    // Returning false ends the job. Re-registering a key cancels the previous job.
    SdkResult<void> StartPolling(const std::string& key, Deadline interval,
        std::function<bool(const std::atomic<bool>&)> poll);
    void CancelPolling(const std::string& key);
    SdkResult<void> Close();
 private:
    SdkResult<void> SendWorkMode(const ::xr::sdk::v2::RobotModeParam&, Deadline, const std::string&, bool internal);
    void HeartbeatLoop();
    std::vector<std::shared_ptr<ActiveCall>> Calls();
    const std::shared_ptr<grpc::Channel> channel_;
    const std::string model_, session_id_;
    std::atomic<bool> closing_{false};
    std::mutex mutex_, close_mutex_, mode_mutex_;
    std::condition_variable heartbeat_changed_;
    std::vector<std::weak_ptr<ActiveCall>> calls_;
    std::map<std::string, std::weak_ptr<ActiveCall>> polling_;
    bool acquired_{false}, release_needed_{false}, heartbeat_stop_{false};
    std::optional<ErrorCode> heartbeat_error_;
    std::thread heartbeat_;
    SdkResult<void> close_result_;
};
}  // namespace x2robot::sdk::client
