#pragma once

// 示例退出时把机器人交还应用。
//
// 示例一旦调用 System.SetWorkMode(SDK)，整机侧 /application/task_info.type 就会
// 变成 5(SDK)；退出前必须回到 IDLE(0)，否则机器人一直停在 SDK 模式，应用侧
// 任务无法恢复。用法：
//   check(x2robot::examples::NoteWorkMode(robot, sdk,
//         robot->GetSystem().SetWorkMode(sdk, 3s)), "System.SetWorkMode");
//   ...
//   ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
//   ok = check(robot->Close(), "Robot.Close") && ok;
// Ctrl+C 由 interrupt.cpp 的看门狗线程处理：第一次 Ctrl+C 先执行这里的清理
// 钩子（可以安全做 RPC），再退出 130；第二次 Ctrl+C 立即退出。

#include <atomic>
#include <chrono>
#include <iostream>

#include "interrupt.hpp"
#include "x2robot/sdk/robot.hpp"

namespace x2robot::examples {

inline bool& SdkModeEntered() {
  static bool entered = false;
  return entered;
}

inline void MarkSdkModeEntered(const ::xr::sdk::v2::RobotModeParam& request) {
  if (request.mode() == ::xr::sdk::v2::SDK) SdkModeEntered() = true;
}

// 交还 SDK 模式（task type 回到 0）；失败返回 false，让示例以失败退出。
inline bool RestoreIdleModeOnExit(x2robot::sdk::Robot* robot) {
  if (!SdkModeEntered()) return true;
  SdkModeEntered() = false;
  if (robot == nullptr) return true;
  ::xr::sdk::v2::RobotModeParam idle;
  idle.set_mode(::xr::sdk::v2::IDLE);
  // Cover serial task shutdown (up to 11 s) and WBC STOP (up to 21 s).
  // Long Cartesian replays have taken over 11 s just to stop WBC (SX001-176).
  // Wait for the original request; do not retry an ambiguous mode transition.
  auto result = robot->GetSystem().SetWorkMode(idle, std::chrono::seconds(35));
  if (result.Ok()) return true;
  std::cerr << "System.SetWorkMode(IDLE) failed: "
            << result.ErrorValue().error_message << '\n';
  return false;
}

// 智能指针版本（unique_ptr / shared_ptr）。
template <typename RobotPtr>
bool RestoreIdleModeOnExit(RobotPtr& robot) {
  return RestoreIdleModeOnExit(robot ? robot.get() : nullptr);
}

// 把 Robot 注册给 Ctrl+C 看门狗：SIGINT/SIGTERM 时由看门狗线程调用
// RestoreIdleModeOnExit 交还 IDLE。只借用裸指针：Robot 由示例 main 持有，
// 存活到进程结束，看门狗只在退出路径上使用它。
template <typename RobotPtr>
void RegisterIdleRestoreOnCtrlC(RobotPtr& robot) {
  static std::atomic_bool registered{false};
  if (registered.exchange(true)) return;
  RegisterExitHook(robot.get(), [](void* context) -> bool {
    return RestoreIdleModeOnExit(static_cast<x2robot::sdk::Robot*>(context));
  });
}

// 只记录本进程真的把机器人切进 SDK 模式的那一次：没进过就不在退出时打扰
// 别人的 SDK 会话。
template <typename T>
x2robot::sdk::SdkResult<T> NoteWorkMode(
    const ::xr::sdk::v2::RobotModeParam& request,
    x2robot::sdk::SdkResult<T> result) {
  if (result.Ok()) MarkSdkModeEntered(request);
  return result;
}

// 带 Robot 的版本：除了记录占用，还注册 Ctrl+C 清理钩子。
template <typename RobotPtr, typename T>
x2robot::sdk::SdkResult<T> NoteWorkMode(
    RobotPtr& robot,
    const ::xr::sdk::v2::RobotModeParam& request,
    x2robot::sdk::SdkResult<T> result) {
  if (result.Ok()) {
    MarkSdkModeEntered(request);
    if (request.mode() == ::xr::sdk::v2::SDK) {
      RegisterIdleRestoreOnCtrlC(robot);
    }
  }
  return result;
}

}  // namespace x2robot::examples
