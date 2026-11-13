#include "interrupt.hpp"

#include <chrono>
#include <mutex>
#include <thread>

namespace {

struct Registration {
  void* context = nullptr;
  x2robot::examples::ExitHook hook = nullptr;
  bool watchdog_started = false;
};

Registration& RegistrationState() {
  static Registration registration;
  return registration;
}

std::mutex& RegistrationMutex() {
  static std::mutex mutex;
  return mutex;
}

// 收到退出信号后在这里执行清理：钩子里的 SDK 调用自带 deadline，不会无限
// 等待；无论清理成功与否都以 130 退出，避免把 Ctrl+C 变成挂死。
void Watchdog() {
  using namespace std::chrono_literals;
  while (true) {
    std::this_thread::sleep_for(50ms);
    if (!x2robot::examples::ExitRequested().load(std::memory_order_acquire)) {
      continue;
    }
    void* context = nullptr;
    x2robot::examples::ExitHook hook = nullptr;
    {
      std::lock_guard<std::mutex> lock(RegistrationMutex());
      context = RegistrationState().context;
      hook = RegistrationState().hook;
    }
    if (hook != nullptr) (void)hook(context);
    std::_Exit(130);
  }
}

struct InterruptInstaller {
  InterruptInstaller() { x2robot::examples::InstallGracefulExitOnCtrlC(); }
};

// Every executable gets this translation unit through sdk_add_example, so even
// examples without a streaming loop share the same Ctrl+C contract: restore
// the robot to IDLE, then exit 130.
const InterruptInstaller kInterruptInstaller{};

}  // namespace

namespace x2robot::examples {

void RegisterExitHook(void* context, ExitHook hook) {
  std::lock_guard<std::mutex> lock(RegistrationMutex());
  auto& registration = RegistrationState();
  registration.context = context;
  registration.hook = hook;
  if (registration.watchdog_started) return;
  registration.watchdog_started = true;
  std::thread(Watchdog).detach();
}

}  // namespace x2robot::examples
