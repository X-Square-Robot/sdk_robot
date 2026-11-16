// Counterpart of python/v2/quanta_x3/lidar_points.py. Read-only; no SDK mode entry.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include "interrupt.hpp"
#include "point_cloud.hpp"
#include "x2robot/sdk/robot.hpp"

namespace {
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using x2robot::sdk::DepthPoints;
using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;

struct Options {
    std::string server = "127.0.0.1:55051";
    std::string action = "single";
    std::optional<double> seconds;
    bool help = false;
};

Options Parse(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("missing value for " + arg);
            return argv[i];
        };
        if (arg == "--help" || arg == "-h")
            options.help = true;
        else if (arg == "--server")
            options.server = next();
        else if (arg == "--action")
            options.action = next();
        else if (arg == "--seconds") {
            const auto text = next();
            size_t end = 0;
            const double value = std::stod(text, &end);
            if (end != text.size() || !std::isfinite(value))
                throw std::invalid_argument("seconds must be a finite number");
            options.seconds = std::max(0.1, value);
        } else
            throw std::invalid_argument("unknown argument: " + arg);
    }
    if (!options.help && options.action != "single" && options.action != "stream" && options.action != "verify")
        throw std::invalid_argument("action must be single, stream or verify");
    return options;
}

template <typename T>
bool Check(const SdkResult<T>& result, const char* operation) {
    if (result.Ok()) return true;
    const auto& error = result.ErrorValue();
    std::cerr << operation << " failed [" << error.error_name << "]: " << error.error_message
              << " (request_id=" << result.request_id << ")\n";
    return false;
}

bool ReadSingle(const DepthPoints& service) {
    const auto cloud = service.GetLidarPoints(8s);
    return Check(cloud, "DepthPoints.GetLidarPoints") && x2robot::examples::point_cloud::Display(cloud.Value());
}

bool ReadStream(const DepthPoints& service, std::optional<double> seconds) {
    auto opened = service.GetLidarPointsStream(8s);
    if (!Check(opened, "DepthPoints.GetLidarPointsStream")) return false;
    auto stream = std::move(opened.Value());
    const auto start = Clock::now();
    size_t count = 0;
    bool ok = true;
    while (!x2robot::examples::ExitRequestedBySignal()) {
        if (seconds && std::chrono::duration<double>(Clock::now() - start).count() >= *seconds) break;
        // Short local waits make both --seconds and Ctrl+C responsive, even
        // when the server pauses publishing. They do not reopen the RPC.
        auto cloud = stream->Read(100ms);
        if (x2robot::examples::ExitRequestedBySignal()) break;
        if (!cloud.Ok() && cloud.ErrorValue().error_code == x2robot::sdk::errc::kTimeout) continue;
        if (!Check(cloud, "DepthPoints.GetLidarPointsStream read") ||
            !x2robot::examples::point_cloud::Display(cloud.Value())) {
            ok = false;
            break;
        }
        ++count;
        std::cout.flush();
    }
    if (!count && !x2robot::examples::ExitRequestedBySignal()) {
        std::cerr << "Lidar stream failed: no samples\n";
        ok = false;
    }
    ok = Check(stream->Cancel(), "DepthPoints.GetLidarPointsStream cancel") && ok;
    return ok;
}
}  // namespace

int main(int argc, char** argv) {
    Options options;
    try {
        options = Parse(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "Argument failed: " << error.what() << '\n';
        return 2;
    }
    if (options.help) {
        std::cout << "Usage: quanta_x3_lidar_points [--server HOST:PORT]\n"
                     "  [--action single|stream|verify] [--seconds SECONDS]\n"
                     "Read-only Quanta X3 3D lidar. Default: single; stream runs until Ctrl+C\n"
                     "unless --seconds is given. Verify reads one cloud and a 0.5-second stream.\n";
        return 0;
    }
    x2robot::examples::InstallGracefulExitOnCtrlC();
    auto connected = Robot::Create(options.server, true);
    if (!Check(connected, "Robot.Create")) return 1;
    auto robot = std::move(connected.Value());
    bool ok = false;
    try {
        if (robot->ModelName() != "quanta_x3") throw std::runtime_error("this example is only for Quanta X3");
        const auto service = robot->GetDepthPoints();
        if (options.action == "single")
            ok = ReadSingle(service);
        else if (options.action == "stream")
            ok = ReadStream(service, options.seconds);
        else
            ok = ReadSingle(service) && !x2robot::examples::ExitRequestedBySignal() && ReadStream(service, 0.5);
    } catch (const std::exception& error) {
        std::cerr << "Lidar example failed: " << error.what() << '\n';
    }
    if (x2robot::examples::ExitRequestedBySignal()) {
        std::cout << "Stopped by user.\n";
        ok = true;
    }
    ok = Check(robot->Close(), "Robot.Close") && ok;
    return ok ? 0 : 1;
}
