#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "data_collection/replay.hpp"
#include "start_check.hpp"
#include "interrupt.hpp"
#include "sdk_mode_cleanup.hpp"

namespace {
double Number(const std::string& text, const std::string& option, bool zero_allowed)
{
    size_t size = 0;
    const auto value = std::stod(text, &size);
    if (size != text.size() || !std::isfinite(value) || value < 0 || (!zero_allowed && value == 0))
    {
        throw std::runtime_error(option + " must be finite and " + (zero_allowed ? "nonnegative" : "positive"));
    }
    return value;
}

void Usage()
{
    std::cout << "Usage: data_replay_v2 <parsed-session/replay> [--address HOST:PORT]\n"
                 "  --mode joint_pos|end_pose (default: joint_pos)\n"
                 "  --speed N       playback speed (default: 1)\n"
                 "  --max-lag-ms N  drop commands beyond this delay; 0 never drops (default: 0)\n"
                 "  --dry-run       print per-stream counts without connecting to a robot\n"
                 "  --check-only    read current state and check start without motion\n"
                 "  --start-check-config FILE  start tolerances (default: bundled replay.conf)\n"
                 "  --yes           skip the confirmation before live replay\n"
                 "Reads stream_*.pb from the rosbag converter's --replay-format both output.\n";
}
}  // namespace

int main(int argc, char** argv)
{
    std::unique_ptr<x2robot::sdk::Robot> robot;
    int code = 0;
    try
    {
        if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")
        {
            Usage();
            return argc < 2 ? 1 : 0;
        }
        std::string address = "127.0.0.1:55051", mode = "joint_pos";
        double speed = 1, max_lag_ms = 0;
        bool dry_run = false, yes = false, check_only = false;
        std::filesystem::path start_config;
        for (int i = 2; i < argc; ++i)
        {
            const std::string arg = argv[i];
            const auto value = [&]() -> std::string {
                if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
                return argv[++i];
            };
            if (arg == "--address")
                address = value();
            else if (arg == "--mode")
                mode = value();
            else if (arg == "--speed")
                speed = Number(value(), arg, false);
            else if (arg == "--max-lag-ms")
                max_lag_ms = Number(value(), arg, true);
            else if (arg == "--dry-run")
                dry_run = true;
            else if (arg == "--check-only")
                check_only = true;
            else if (arg == "--start-check-config")
                start_config = value();
            else if (arg == "--yes")
                yes = true;
            else
                throw std::runtime_error("unknown option: " + arg);
        }
        if (mode != "joint_pos" && mode != "end_pose") throw std::runtime_error("mode must be joint_pos or end_pose");
        if (dry_run && check_only) throw std::runtime_error("--dry-run and --check-only are mutually exclusive");
        if (start_config.empty()) start_config = sdk_recording::Resource("examples/cpp/data_collection_v2/replay.conf");
        const auto start_limits = sdk_examples::v2::LoadStartLimits(start_config);
        auto episode = sdk_examples::LoadEpisode(argv[1]);
        if (!episode) return 1;
        std::ifstream manifest(std::filesystem::path(argv[1]) / "episode.json");
        const std::string text((std::istreambuf_iterator<char>(manifest)), {});
        const auto recorded_model = sdk_examples::ManifestString(text, "model");
        if (dry_run)
        {
            sdk_examples::ValidateReplayJointCommands(*episode, sdk_examples::ReplayJointLimits(recorded_model), mode == "joint_pos");
            sdk_examples::PrintReplayPlan(sdk_examples::AnalyzeEpisode(*episode, mode == "joint_pos"), mode == "joint_pos", speed, max_lag_ms);
            return 0;
        }
        if (sdk_examples::AnalyzeEpisode(*episode, mode == "joint_pos").applicable == 0)
        {
            std::cout << "Nothing to replay for the selected mode; no connection or commands needed\n";
            return 0;
        }
        if (!yes && !check_only)
        {
            std::cout << "Press Enter to start replay (Ctrl+C to cancel)..." << std::flush;
            std::string line;
            if (!std::getline(std::cin, line)) return 1;
        }
        if (x2robot::examples::ExitRequestedBySignal()) return 130;
        auto connected = x2robot::sdk::Robot::Create(address, true);
        if (!connected.Ok()) throw std::runtime_error(connected.ErrorValue().error_message);
        robot = std::move(connected.Value());
        if (!recorded_model.empty() && recorded_model != robot->ModelName())
            throw std::runtime_error("recorded model differs from the connected robot");
        sdk_examples::ValidateReplayJointCommands(*episode, sdk_examples::ReplayJointLimits(robot->ModelName()), mode == "joint_pos", check_only);
        sdk_examples::v2::CheckStartState(*robot, *episode, mode == "joint_pos", start_limits);
        if (check_only)
            std::cout << "PASS: start check only; no motion commands sent\n";
        else
        {
            x2robot::examples::RegisterIdleRestoreOnCtrlC(robot);
            sdk_examples::ReplayBehavior options;
            options.velocity_chassis_tracking = true;
            options.warn_and_continue = true;
            options.recorded_parts_only = true;
            sdk_examples::EpisodeReplayer replayer(*robot, options);
            code = replayer.Replay(*episode, mode, speed, &x2robot::examples::ExitRequested(), max_lag_ms) ? 0 : 1;
        }
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        code = 1;
    }
    if (robot)
    {
        if (!x2robot::examples::RestoreIdleModeOnExit(robot)) code = 1;
        if (!robot->Close().Ok()) code = 1;
    }
    return code;
}
