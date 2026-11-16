#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <iomanip>
#include <mutex>
#include <random>
#include <sstream>
#include <thread>

#include <google/protobuf/util/json_util.h>
#include <poll.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include "interrupt.hpp"
#include "options.hpp"
#include "x2robot/sdk/robot.hpp"

extern char** environ;

namespace {
using namespace sdk_recording;
using x2robot::sdk::Deadline;
using x2robot::sdk::Robot;
using Status = ::xr::sdk::v2::RecordingStatus;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

template <typename T>
T Checked(x2robot::sdk::SdkResult<T> result)
{
    if (!result.Ok()) throw std::runtime_error(result.ErrorValue().error_message + " (request_id=" + result.request_id + ")");
    return std::move(result.Value());
}

Status CheckedStatus(x2robot::sdk::SdkResult<Status> result)
{
    auto status = Checked(std::move(result));
    if (status.state() == "FAILED") throw std::runtime_error(status.error_code() + ": " + status.error_message());
    return status;
}

void Print(const google::protobuf::Message& message)
{
    std::string json;
    const auto result = google::protobuf::util::MessageToJsonString(message, &json);
    if (!result.ok()) throw std::runtime_error(result.ToString());
    std::cout << json << std::endl;
}

void CheckInterrupt()
{
    if (x2robot::examples::ExitRequestedBySignal()) throw std::runtime_error("interrupted; recover using the same --session-id");
}

Status WaitFor(Robot& robot, const std::string& id, const std::string& state, double timeout, double status_timeout = 30)
{
    const auto end = Clock::now() + std::chrono::duration<double>(timeout);
    while (Clock::now() < end)
    {
        CheckInterrupt();
        const auto remaining = std::chrono::duration_cast<Deadline>(end - Clock::now());
        auto status = CheckedStatus(
            robot.GetRecording().GetRecordingStatus(id, std::max(1ms, std::min(Deadline{static_cast<int64_t>(status_timeout * 1000)}, remaining))));
        if (status.state() == state) return status;
        if (status.state() == "DELETING" || status.state() == "DELETED")
            throw std::runtime_error("session is " + status.state() + "; use a new session ID or retry --action delete");
        std::this_thread::sleep_for(100ms);
    }
    throw std::runtime_error("session " + id + " is still pending; query/stop with the same --session-id");
}

std::vector<std::string> SelectItems(Robot& robot, const Options& options)
{
    if (options.explicit_items) return options.items;
    const auto path = ConfigPath(options, robot.ModelName());
    const auto config = ReadConfig(path);
    if (config.model != robot.ModelName()) throw std::runtime_error("preset model differs from robot model: " + robot.ModelName());
    std::vector<std::string> selected;
    for (const auto& item : config.items)
        if (item.enabled) selected.push_back(item.id);
    ValidateItems(selected);
    std::cout << "config=" << path.string() << std::endl;
    return selected;
}

struct RecordingUpdates
{
    std::mutex mutex;
    std::string error;
    void OnStatus(const x2robot::sdk::SdkResult<Status>& result)
    {
        std::string failure;
        if (!result.Ok()) failure = result.ErrorValue().error_message;
        else
        {
            const auto& status = result.Value();
            std::cout << "recording_state=" << status.state() << std::endl;
            if (!status.error_code().empty()) failure = status.error_code() + ": " + status.error_message();
            else if (status.state() == "STOPPING" || status.state() == "FINALIZING" || status.state() == "READY" || status.state() == "FAILED")
                failure = "recording ended early (state=" + status.state() + ")";
        }
        std::lock_guard lock(mutex);
        if (error.empty()) error = failure;
    }
    void Check()
    {
        std::lock_guard lock(mutex);
        if (!error.empty()) throw std::runtime_error(error);
    }
};

void RecordFor(const std::shared_ptr<RecordingUpdates>& updates, double seconds)
{
    const auto end = Clock::now() + std::chrono::duration<double>(seconds);
    if (seconds == 0) std::cout << "Recording; press Enter to stop: " << std::flush;
    while (seconds == 0 || Clock::now() < end)
    {
        CheckInterrupt();
        updates->Check();
        if (seconds == 0)
        {
            pollfd input{STDIN_FILENO, POLLIN, 0};
            const int ready = ::poll(&input, 1, 100);
            if (ready < 0 && errno != EINTR) throw std::runtime_error("stdin poll failed");
            if (ready > 0)
            {
                char value;
                const auto read = ::read(STDIN_FILENO, &value, 1);
                if (read == 0 || (read > 0 && value == '\n')) return;
                if (read < 0 && errno != EINTR) throw std::runtime_error("stdin read failed");
            }
        }
        else
            std::this_thread::sleep_for(50ms);
    }
}

void ParseBag(const Options& options, const fs::path& bag, const fs::path& output)
{
    const auto converter = Resource("tools/rosbag_to_json_mp4/convert_rosbag.py");
    std::vector<std::string> command{options.python, converter.string(), "--bag-path",      bag.string(),
                                     "--output-dir", output.string(),    "--replay-format", options.replay_format};
    std::vector<char*> argv;
    for (auto& arg : command) argv.push_back(arg.data());
    argv.push_back(nullptr);
    pid_t child;
    const int spawned = ::posix_spawnp(&child, argv[0], nullptr, nullptr, argv.data(), environ);
    if (spawned) throw std::runtime_error("cannot launch parser: " + std::string(std::strerror(spawned)));
    int status;
    while (::waitpid(child, &status, 0) < 0)
    {
        if (errno != EINTR) throw std::runtime_error("cannot wait for parser");
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) throw std::runtime_error("rosbag parsing failed; raw bag retained at " + bag.string());
}

class DownloadProgress
{
    Clock::time_point started_ = Clock::now(), printed_{};

   public:
    DownloadProgress()
    {
        std::cout << "download_started: waiting for metadata" << std::endl;
    }
    void operator()(uint64_t received, uint64_t total, bool verified)
    {
        const auto now = Clock::now();
        if (!verified && printed_ != Clock::time_point{} && now - printed_ < 1s) return;
        printed_ = now;
        const auto rate = received / std::max(0.001, std::chrono::duration<double>(now - started_).count());
        std::ostringstream line;
        line << std::fixed << std::setprecision(1) << (verified ? "download_complete=" : "download_progress=")
             << (total ? 100.0 * received / total : 100.0) << "% " << received / 1048576.0 << "/" << total / 1048576.0 << " MiB | "
             << rate / 1048576.0 << " MiB/s | ETA ";
        if (rate > 0)
            line << std::setprecision(0) << (total - received) / rate << "s";
        else
            line << "--";
        if (verified)
            line << " | MD5 verified";
        else if (received == total)
            line << " | awaiting verification";
        std::cout << line.str() << std::endl;
    }
};

void Run(Robot& robot, Options options)
{
    const auto status_deadline = Deadline{std::max<int64_t>(1, static_cast<int64_t>(options.status_timeout * 1000))};
    if (options.action == "delete")
    {
        Print(CheckedStatus(robot.GetRecording().DeleteRecording(options.session_id, status_deadline)));
        return;
    }
    if (options.action == "status")
    {
        Print(CheckedStatus(robot.GetRecording().GetRecordingStatus(options.session_id, status_deadline)));
        return;
    }
    if (options.session_id.empty())
    {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        options.session_id =
            "sdk_" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()) + "_" + std::to_string(std::random_device{}());
    }
    // Print before START so an uncertain RPC can always be recovered with this ID.
    std::cout << "session_id=" << options.session_id << std::endl;
    if (options.action == "collect" || options.action == "start")
    {
        const auto items = SelectItems(robot, options);
        ::xr::sdk::v2::StartRecordingRequest request;
        request.set_session_id(options.session_id);
        for (const auto& item : items) request.add_data_items(item);
        CheckInterrupt();
        auto updates = std::make_shared<RecordingUpdates>();
        CheckedStatus(robot.GetRecording().StartRecording(
            request, [updates](const auto& result) { updates->OnStatus(result); }, 5000ms, 10000ms, status_deadline));
        if (options.action == "start")
        {
            Print(WaitFor(robot, options.session_id, "RECORDING", 30, options.status_timeout));
            return;
        }
        try
        {
            WaitFor(robot, options.session_id, "RECORDING", 30, options.status_timeout);
            RecordFor(updates, options.duration);
        }
        catch (...)
        {
            // Stop on interruption or a post-start failure, without hiding the first error.
            const auto first_error = std::current_exception();
            try
            {
                CheckedStatus(robot.GetRecording().StopRecording(options.session_id));
            }
            catch (const std::exception& error)
            {
                std::cerr << "Stop failed: " << error.what() << '\n';
            }
            std::rethrow_exception(first_error);
        }
        CheckedStatus(robot.GetRecording().StopRecording(options.session_id));
    }
    else if (options.action == "stop")
    {
        CheckedStatus(robot.GetRecording().StopRecording(options.session_id));
    }
    const auto status = WaitFor(robot, options.session_id, "READY", options.finalize_timeout, options.status_timeout);
    robot.GetRecording().CancelRecordingCallback(options.session_id);
    Print(status);
    if (options.action == "stop") return;
    if (status.file_id().empty()) throw std::runtime_error("READY response is missing file_id");
    fs::create_directories(options.output_dir);
    const auto bag = options.output_dir / (options.session_id + ".tar");
    CheckInterrupt();
    const auto download_timeout = Deadline{std::max<int64_t>(1, static_cast<int64_t>(options.finalize_timeout * 1000))};
    Checked(robot.GetFiles().DownloadFile(status.file_id(), bag.string(), download_timeout, DownloadProgress{}));
    std::string cleanup_error;
    if (options.cleanup_after_download)
    {
        try
        {
            const auto deleted = CheckedStatus(robot.GetRecording().DeleteRecording(options.session_id, status_deadline));
            if (deleted.state() != "DELETED") throw std::runtime_error("unexpected cleanup state: " + deleted.state());
            std::cout << "remote_cleanup=DELETED session_id=" << options.session_id << std::endl;
        }
        catch (const std::exception& error)
        {
            cleanup_error = "remote cleanup failed; verified local file retained at " + bag.string() + ": " + error.what();
            std::cerr << cleanup_error << std::endl;
        }
    }
    std::cout << "downloaded=" << bag.string() << " (length/MD5 verified by SDK)" << std::endl;
    if (!options.raw_only) ParseBag(options, bag, options.output_dir / options.session_id);
    if (!cleanup_error.empty()) throw std::runtime_error(cleanup_error);
}
}  // namespace

int main(int argc, char** argv)
{
    std::unique_ptr<Robot> robot;
    int code = 0;
    try
    {
        const auto options = ParseOptions(argc, argv);
        if (options.help)
        {
            std::cout << "SDK2 recording, download and optional JSON/MP4/replay conversion. No motion commands.\n"
                         "Usage: data_collection_v2 [--address HOST:PORT]\n"
                         "  --action list|collect|start|stop|status|download|delete (default: collect)\n"
                         "  --preset NAME | --config FILE.conf | --items ID... (items overrides config)\n"
                         "  --session-id ID       reuse this ID for stop/status/download\n"
                         "  --duration SECONDS    default 10; 0 waits for Enter\n"
                         "  --output-dir DIR      default data_collection_v2\n"
                         "  --cleanup-after-download  delete remote session after verified download (default)\n"
                         "  --keep-robot-data      retain remote bag/archive after download\n"
                         "  --raw-only            download without parsing or requiring Python\n"
                         "  --replay-format none|json|both (default: both)\n"
                         "  --status-timeout SECONDS (default: 30; independent of the 5s poll interval)\n"
                         "  --finalize-timeout SECONDS (default: 600)\n"
                         "  --python EXECUTABLE   parser interpreter (default: python3)\n";
            return 0;
        }
        if (options.action == "list")
        {
            const auto preset = ReadConfig(ConfigPath(options));
            std::cout << "preset=" << preset.name << " model=" << preset.model << " (local configuration)\n";
            for (const auto& item : preset.items)
                std::cout << item.id << "\tenabled=" << (item.enabled ? "true" : "false") << "\t" << item.description << '\n';
            return 0;
        }
        robot = Checked(Robot::Create(options.address, true));
        Run(*robot, options);
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << std::endl;
        code = x2robot::examples::ExitRequestedBySignal() ? 130 : 1;
    }
    if (robot)
    {
        const auto closed = robot->Close();
        if (!closed.Ok())
        {
            std::cerr << "Robot.Close: " << closed.ErrorValue().error_message << '\n';
            code = 1;
        }
    }
    return code;
}
