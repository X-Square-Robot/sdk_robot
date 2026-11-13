#pragma once

#include <atomic>
#include <csignal>
#include <cstdlib>

namespace x2robot::examples {

// 退出钩子：返回 true 表示清理成功。钩子在独立线程里执行（不在信号处理函数
// 里），所以可以安全地做 RPC —— Ctrl+C 也要把机器人交还应用。
using ExitHook = bool (*)(void*);

// 注册退出清理钩子并启动看门狗线程；示例在拿到 Robot 之后调用一次。
void RegisterExitHook(void* context, ExitHook hook);

// 第二次 Ctrl+C：放弃清理，立即退出（RPC 卡死时的逃生通道）。
[[noreturn]] inline void ForceExitOnCtrlC(int) noexcept { std::_Exit(130); }

inline std::atomic_bool& ExitRequested() noexcept {
  static std::atomic_bool requested{false};
  return requested;
}

inline std::atomic_int& ExitSignalCount() noexcept {
  static std::atomic_int count{0};
  return count;
}

// 第一次 Ctrl+C：只置位。看门狗线程会先执行退出钩子（把
// /application/task_info.type 从 5 交还成 0），再以 130 退出；第二次 Ctrl+C
// 直接退出，不再等待清理。信号处理函数里不做 RPC、不析构。
inline void RequestExitOnCtrlC(int) noexcept {
  if (ExitSignalCount().fetch_add(1, std::memory_order_acq_rel) > 0) {
    std::_Exit(130);
  }
  ExitRequested().store(true, std::memory_order_release);
}

// 兼容旧调用点：仍然提供"立刻退出、不清理"的处理器。
inline void InstallForceExitOnCtrlC() noexcept {
  std::signal(SIGINT, ForceExitOnCtrlC);
}

// 示例的统一 Ctrl+C/SIGTERM 语义：优雅退出 + 交还 SDK 模式。
inline void InstallGracefulExitOnCtrlC() noexcept {
  std::signal(SIGINT, RequestExitOnCtrlC);
  std::signal(SIGTERM, RequestExitOnCtrlC);
}

// 主循环用它判断是否收到了退出信号。
inline bool ExitRequestedBySignal() noexcept {
  return ExitRequested().load(std::memory_order_acquire);
}

}  // namespace x2robot::examples
