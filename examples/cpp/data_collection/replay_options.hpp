#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace sdk_examples {

struct ReplayOptions {
    std::string episode_dir{"./collected_data/episode_0000"};
    std::string address{"127.0.0.1:55051"};
    std::string mode{"joint_pos"};
    double speed{1.0};
    double max_lag_ms{0.0};
    bool help{false};
    bool yes{false};
    bool dry_run{false};
    bool replay_chassis{true};
};

// Both replay entries accept named options and the historical positional form.
// Reject typos instead of silently selecting joint mode or replacing the server.
inline bool ParseReplayArgs(int argc, char** argv, ReplayOptions* options, std::string* error) {
    const auto fail = [&](const std::string& message) {
        *error = message;
        return false;
    };
    const auto number = [](const std::string& text, double* value) {
        try {
            std::size_t end = 0;
            *value = std::stod(text, &end);
            return end == text.size() && std::isfinite(*value);
        } catch (...) {
            return false;
        }
    };
    bool server_set = false, mode_set = false, speed_set = false, lag_set = false;
    bool positional_only = false;
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (!positional_only && arg == "--") { positional_only = true; continue; }
        if (!positional_only && (arg == "--help" || arg == "-h")) { options->help = true; continue; }
        if (!positional_only && arg == "--yes") { options->yes = true; continue; }
        if (!positional_only && arg == "--dry-run") { options->dry_run = true; continue; }
        if (!positional_only && arg == "--no-chassis") { options->replay_chassis = false; continue; }
        if (!positional_only && !arg.empty() && arg[0] == '-') {
            const auto equal = arg.find('=');
            const auto name = arg.substr(0, equal);
            bool* seen = name == "--server" ? &server_set : name == "--mode" ? &mode_set :
                         name == "--speed" ? &speed_set : name == "--max-lag-ms" ? &lag_set : nullptr;
            if (!seen) return fail("unknown option: " + name);
            if (*seen) return fail("duplicate option: " + name);
            *seen = true;
            std::string value;
            if (equal != std::string::npos) value = arg.substr(equal + 1);
            else if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) value = argv[++i];
            if (value.empty()) return fail(name + " requires a value");
            if (name == "--server") options->address = value;
            else if (name == "--mode") options->mode = value;
            else if (!number(value, name == "--speed" ? &options->speed : &options->max_lag_ms))
                return fail(name + " requires a finite number");
        } else {
            positional.push_back(arg);
        }
    }
    if (options->help) return true;
    if (positional.empty() && argc > 1) return fail("episode directory is required");
    std::size_t index = 0;
    if (index < positional.size()) options->episode_dir = positional[index++];
    if (!server_set && index < positional.size()) options->address = positional[index++];
    if (!mode_set && index < positional.size()) options->mode = positional[index++];
    if (!speed_set && index < positional.size() && !number(positional[index++], &options->speed))
        return fail("speed requires a finite number");
    if (index != positional.size()) return fail("too many positional arguments");
    if (options->mode == "joint") options->mode = "joint_pos";
    if (options->mode == "pose") options->mode = "end_pose";
    if (options->mode != "joint_pos" && options->mode != "end_pose")
        return fail("mode must be joint_pos or end_pose");
    if (options->speed <= 0.0) return fail("--speed must be > 0");
    if (options->max_lag_ms < 0.0) return fail("--max-lag-ms must be >= 0");
    return true;
}

inline void PrintReplayUsage(const char* program) {
    std::cout << "Usage: " << program
              << " <episode-dir> [server] [joint_pos|end_pose] [speed]\n"
                 "       Named options may appear before or after the episode directory:\n"
                 "  --server HOST:PORT        Robot address (default 127.0.0.1:55051).\n"
                 "  --mode joint_pos|end_pose Replay joints or recorded Cartesian poses (default joint_pos).\n"
                 "  --speed N                 Positive playback multiplier (default 1).\n"
                 "  --yes                     Skip the data_replay_example confirmation.\n"
                 "  --dry-run                 Print the per-stream replay plan without connecting.\n"
                 "  --no-chassis              Skip all chassis mode and motion commands.\n"
                 "  --max-lag-ms N            Drop records later than N ms; 0 never drops records.\n"
                 "  -h, --help                Show this help without connecting.\n"
                 "Example: " << program
              << " ./episode_0000 --server 127.0.0.1:55051 --mode end_pose --dry-run\n"
                 "end_pose consumes recorded left_arm_end_pose/right_arm_end_pose/waist_end_pose;\n"
                 "it does not convert joint-only recordings into Cartesian poses.\n";
    std::cout << "Quanta X3 end_pose aligns and refreshes all three poses at 50 Hz;\n"
                 "--max-lag-ms applies to event replay, not synchronized pose interpolation.\n";
}

}  // namespace sdk_examples
