#include "data_collection/replay.hpp"
#include "data_collection/replay_options.hpp"
#include "interrupt.hpp"
#include "sdk_mode_cleanup.hpp"

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {
std::atomic<bool> g_stop{false};
void OnSignal(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }

}

int main(int argc, char** argv) {
  sdk_examples::ReplayOptions options;
  std::string error;
  if (!sdk_examples::ParseReplayArgs(argc, argv, &options, &error)) {
    std::cerr << "Replay: " << error << '\n';
    sdk_examples::PrintReplayUsage(argv[0]);
    return 2;
  }
  if (argc < 2 || options.help) {
    sdk_examples::PrintReplayUsage(argv[0]);
    return argc < 2 ? 1 : 0;
  }
  const auto& [episode_dir, address, mode, speed, max_lag_ms, help, yes, dry_run, replay_chassis] = options;
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  const auto episode = sdk_examples::LoadEpisode(episode_dir);
  if (!episode) return 1;
  bool joint_mode = false;
  if (!sdk_examples::ReplayMode(mode, &joint_mode)) {
    std::cerr << "Replay: mode must be joint_pos or end_pose\n";
    return 2;
  }
  std::cout << "Episode " << episode->episode_id << " task=\"" << episode->task
            << "\" records=" << episode->records.size() << " duration=" << episode->duration
            << "s\nMode: " << mode << " speed: " << speed << "x\n";
  if (dry_run) {
    // Offline evidence for QA/CI: the same per-stream plan a live replay prints,
    // with no connection and no motion.
    sdk_examples::PrintReplayPlan(
        sdk_examples::AnalyzeEpisode(*episode, joint_mode, replay_chassis), joint_mode, speed, max_lag_ms);
    return 0;
  }
  if (!yes) {
    std::cout << "Press Enter to start replay (Ctrl+C to cancel)...";
    std::string line;
    if (!std::getline(std::cin, line)) return 1;
  }

  auto connected = x2robot::sdk::Robot::Create(address, true);
  if (!connected.Ok()) {
    std::cerr << "Robot.Create failed: " << connected.ErrorValue().error_message << '\n';
    return 1;
  }
  auto robot = std::move(connected.Value());
  x2robot::examples::RegisterIdleRestoreOnCtrlC(robot);
  sdk_examples::EpisodeReplayer replayer(*robot);
  bool ok = replayer.Replay(*episode, mode, speed, &g_stop, max_lag_ms, replay_chassis);
  // Replay 进入过 SDK 模式：关闭连接前交还 IDLE，task_info.type 回到 0。
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = robot->Close().Ok() && ok;
  return ok ? 0 : 1;
}
