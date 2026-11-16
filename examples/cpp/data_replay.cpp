#include "data_collection/replay.hpp"
#include "data_collection/replay_options.hpp"
#include "interrupt.hpp"
#include "sdk_mode_cleanup.hpp"

#include <atomic>
#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <iostream>

namespace {
std::atomic<bool> g_stop{false};
void OnSignal(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }
}

// Non-interactive replay utility:
//   data_replay <episode-dir> [server] [joint_pos|end_pose] [speed]
int main(int argc, char** argv) {
  sdk_examples::ReplayOptions options;
  std::string error;
  if (!sdk_examples::ParseReplayArgs(argc, argv, &options, &error)) {
    std::cerr << "Replay: " << error << '\n';
    sdk_examples::PrintReplayUsage(argv[0]);
    return 2;
  }
  if (options.help) {
    sdk_examples::PrintReplayUsage(argv[0]);
    return 0;
  }
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  const auto episode = sdk_examples::LoadEpisode(options.episode_dir);
  if (!episode) return 1;
  if (options.dry_run) {
    sdk_examples::PrintReplayPlan(sdk_examples::AnalyzeEpisode(*episode, options.mode == "joint_pos", options.replay_chassis),
                                 options.mode == "joint_pos", options.speed, options.max_lag_ms);
    return 0;
  }
  auto connected = x2robot::sdk::Robot::Create(options.address, true);
  if (!connected.Ok()) {
    std::cerr << "Robot.Create failed: " << connected.ErrorValue().error_message << '\n';
    return 1;
  }
  auto robot = std::move(connected.Value());
  x2robot::examples::RegisterIdleRestoreOnCtrlC(robot);
  sdk_examples::EpisodeReplayer replayer(*robot);
  bool ok = replayer.Replay(*episode, options.mode, options.speed, &g_stop, options.max_lag_ms, options.replay_chassis);
  // Replay 进入过 SDK 模式：关闭连接前交还 IDLE，task_info.type 回到 0。
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = robot->Close().Ok() && ok;
  return ok ? 0 : 1;
}
