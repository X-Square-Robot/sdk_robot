// Counterpart of python/v1/chassis_control.py.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"
#include "sdk_mode_cleanup.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#if defined(__linux__)
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using namespace std::chrono_literals;

namespace {
std::atomic_bool stop{false};
void OnSignal(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }

// 键盘遥操作与 v1/v2 Python 示例保持一致：速度模式要求以 >=10 Hz 连续下发，
// 并且按键要单字符即时响应，所以用 raw 终端 + 后台 20 Hz 发送线程。
constexpr double kDefaultSpeed = 0.35;
constexpr double kDefaultTurn = 0.40;
constexpr double kSpeedLimit = 1.0;
constexpr double kTurnLimit = 2.0;
constexpr double kSpeedStep = 0.02;
constexpr double kTurnStep = 0.02;
constexpr auto kVelocityPublishPeriod = 50ms;
constexpr int kKeyTimeoutMs = 100;

// 后台按固定频率持续下发当前速度：按键只更新目标速度，松开（读键超时）
// 立即回到停止，避免"只发一次命令"被底盘看门狗忽略。
class VelocityPublisher {
 public:
  explicit VelocityPublisher(x2robot::sdk::ChassisController& chassis)
      : chassis_(chassis), worker_([this] { Run(); }) {}
  ~VelocityPublisher() { Stop(); }

  void Update(double vel_x, double vel_y, double vel_yaw) {
    std::lock_guard lock(mutex_);
    vel_x_ = vel_x;
    vel_y_ = vel_y;
    vel_yaw_ = vel_yaw;
    condition_.notify_all();
  }

  void Stop() {
    if (stopped_.exchange(true)) return;
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  void Send(double vel_x, double vel_y, double vel_yaw) {
    ::xr::sdk::v2::ChassisVelocity velocity;
    velocity.set_vel_x(vel_x);
    velocity.set_vel_y(vel_y);
    velocity.set_vel_yaw(vel_yaw);
    const auto result = chassis_.SetVelocity(velocity, 500ms);
    if (result.Ok()) {
      error_reported_ = false;
      return;
    }
    // 网络抖动时只提示一次，避免刷屏打断键盘交互。
    if (!error_reported_) {
      error_reported_ = true;
      const auto& error = result.ErrorValue();
      std::cerr << "Chassis.SetVelocity failed [" << error.error_name << "]: "
                << error.error_message << '\n';
    }
  }

  void Run() {
    while (!stopped_.load()) {
      double vel_x = 0.0;
      double vel_y = 0.0;
      double vel_yaw = 0.0;
      {
        std::unique_lock lock(mutex_);
        condition_.wait_for(lock, kVelocityPublishPeriod);
        vel_x = vel_x_;
        vel_y = vel_y_;
        vel_yaw = vel_yaw_;
      }
      if (stopped_.load()) break;
      Send(vel_x, vel_y, vel_yaw);
    }
    Send(0.0, 0.0, 0.0);
  }

  x2robot::sdk::ChassisController& chassis_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic_bool stopped_{false};
  std::atomic_bool error_reported_{false};
  double vel_x_{0.0};
  double vel_y_{0.0};
  double vel_yaw_{0.0};
  std::thread worker_;
};

#if defined(__linux__)
// 单字符即时读取（raw 模式）。析构时恢复终端，Ctrl+C 退出后不会把终端
// 卡在 raw 状态。
class RawTerminal {
 public:
  RawTerminal() {
    if (tcgetattr(STDIN_FILENO, &saved_) != 0) return;
    ::termios raw = saved_;
    // 关掉行缓冲、回显和 ISIG：Ctrl+C 变成普通字节 0x03，由读键循环自己处理，
    // 这样退出时能先恢复终端（否则 watchdog 直接 _Exit 会把终端留在 raw 状态）。
    raw.c_lflag &= ~static_cast<::tcflag_t>(ICANON | ECHO | ECHONL | ISIG | IEXTEN);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    active_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
  }
  ~RawTerminal() { Restore(); }

  void Restore() {
    if (!active_) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    active_ = false;
  }

  bool active() const { return active_; }

  // -1: 超时无按键；-2: 输入关闭或读失败；其它: 按键字节。
  int ReadKey(int timeout_ms) const {
    ::fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    ::timeval timeout{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    const int ready = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &timeout);
    if (ready == 0) return -1;
    if (ready < 0) return -2;
    char key = '\0';
    return read(STDIN_FILENO, &key, 1) == 1 ? static_cast<unsigned char>(key) : -2;
  }

 private:
  ::termios saved_{};
  bool active_{false};
};
#endif

}  // namespace

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  const auto& e = result.ErrorValue();
  std::cerr << operation << " failed [" << e.error_name << "]: " << e.error_message << '\n';
  return false;
}

bool SetMode(x2robot::sdk::ChassisController& chassis,
             ::xr::sdk::v2::ChassisControlMode mode) {
  ::xr::sdk::v2::ChassisControlModeParam param; param.set_mode(mode);
  return check(chassis.SetControlMode(param, 3s), "Chassis.SetControlMode");
}

bool StopChassis(x2robot::sdk::ChassisController& chassis) {
  ::xr::sdk::v2::ChassisVelocity velocity;
  bool ok = true;
  for (int i = 0; i < 30; ++i) ok = check(chassis.SetVelocity(velocity, 500ms), "Chassis.SetVelocity(stop)") && ok;
  return ok;
}

bool RunKeyboard(x2robot::sdk::ChassisController& chassis) {
  if (!SetMode(chassis, ::xr::sdk::v2::VELOCITY)) return false;
  std::cout << "Stopping chassis..." << std::endl;
  StopChassis(chassis);

  double direction_x = 0.0;
  double direction_y = 0.0;
  double direction_yaw = 0.0;
  double speed = kDefaultSpeed;
  double turn = kDefaultTurn;

  VelocityPublisher publisher(chassis);
  publisher.Update(0.0, 0.0, 0.0);

  std::cout << "Keyboard control chassis velocity\n"
            << "  w - forward    s - backward\n"
            << "  a - turn left  d - turn right\n"
            << "  i/k - speed +/- 0.02, j/l - turn +/- 0.02\n"
            << "  space - stop   q - quit\n"
            << "Current: speed=" << speed << " m/s, turn=" << turn
            << " rad/s" << std::endl;

#if defined(__linux__)
  RawTerminal terminal;
  if (!terminal.active()) {
    std::cerr << "keyboard mode requires an interactive Linux terminal; "
                 "use velocity/global modes instead\n";
    publisher.Stop();
    return false;
  }
  while (!stop.load()) {
    const int key = terminal.ReadKey(kKeyTimeoutMs);
    if (key == -2) break;
    if (key == 3) break;  // raw 模式里 Ctrl+C 是普通字节，按退出处理
    bool quit = false;
    switch (key >= 0 ? std::tolower(key) : -1) {
      case 'w': direction_x = 1.0; direction_y = 0.0; direction_yaw = 0.0; break;
      case 's': direction_x = -1.0; direction_y = 0.0; direction_yaw = 0.0; break;
      case 'a': direction_x = 0.0; direction_y = 0.0; direction_yaw = 1.0; break;
      case 'd': direction_x = 0.0; direction_y = 0.0; direction_yaw = -1.0; break;
      case 'i':
        speed = std::min(kSpeedLimit, speed + kSpeedStep);
        std::cout << "speed=" << speed << " m/s, turn=" << turn << " rad/s" << std::endl;
        break;
      case 'k':
        speed = std::max(0.0, speed - kSpeedStep);
        std::cout << "speed=" << speed << " m/s, turn=" << turn << " rad/s" << std::endl;
        break;
      case 'j':
        turn = std::min(kTurnLimit, turn + kTurnStep);
        std::cout << "speed=" << speed << " m/s, turn=" << turn << " rad/s" << std::endl;
        break;
      case 'l':
        turn = std::max(0.0, turn - kTurnStep);
        std::cout << "speed=" << speed << " m/s, turn=" << turn << " rad/s" << std::endl;
        break;
      case ' ': direction_x = 0.0; direction_y = 0.0; direction_yaw = 0.0; break;
      case 'q': quit = true; break;
      default:
        // 读键超时（按键已松开）或未知按键：停车。
        direction_x = 0.0;
        direction_y = 0.0;
        direction_yaw = 0.0;
        break;
    }
    if (quit) break;
    publisher.Update(direction_x * speed, direction_y * speed,
                     direction_yaw * turn);
  }
  terminal.Restore();
#else
  std::cerr << "keyboard mode is only supported on Linux; "
               "use velocity/global modes instead\n";
  publisher.Stop();
  return false;
#endif

  publisher.Stop();
  return StopChassis(chassis);
}

void PrintOdom(const ::nav_msgs::Odometry& odom) {
  const auto& p = odom.pose().pose().position();
  const auto& q = odom.pose().pose().orientation();
  const auto& v = odom.twist().twist().linear();
  const auto& w = odom.twist().twist().angular();
  std::cout << "position: (" << p.x() << ", " << p.y() << ", " << p.z() << ")\n"
            << "orientation: (" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ")\n"
            << "linear velocity: (" << v.x() << ", " << v.y() << ", " << v.z() << ")\n"
            << "angular velocity: (" << w.x() << ", " << w.y() << ", " << w.z() << ")\n";
}

bool MoveByVelocity(x2robot::sdk::ChassisController& chassis) {
  if (!SetMode(chassis, ::xr::sdk::v2::VELOCITY)) return false;
  auto send = [&](double x, double y, double yaw, int count, std::chrono::milliseconds period) {
    ::xr::sdk::v2::ChassisVelocity v; v.set_vel_x(x); v.set_vel_y(y); v.set_vel_yaw(yaw);
    bool ok = true;
    for (int i = 0; i < count && !stop.load(); ++i) {
      ok = check(chassis.SetVelocity(v, 500ms), "Chassis.SetVelocity") && ok;
      std::this_thread::sleep_for(period);
    }
    return ok;
  };
  bool ok = send(0.3, 0.0, 0.0, 300, 10ms);
  std::this_thread::sleep_for(1s);
  ok = send(0.0, 0.0, -0.4, 800, 10ms) && ok;
  std::this_thread::sleep_for(1s);
  std::cout << "rotate yaw to positive 0.4 rad/s\n";
  ok = send(0.0, 0.0, 0.4, 800, 10ms) && ok;
  std::this_thread::sleep_for(1s);
  ok = StopChassis(chassis) && ok;
  std::this_thread::sleep_for(1s);
  return ok;
}

bool MoveGlobal(x2robot::sdk::ChassisController& chassis) {
  auto current = chassis.GetGlobalPosition(3s);
  bool ok = check(current, "Chassis.GetGlobalPosition");
  if (ok) std::cout << "current global position: x=" << current.Value().x()
                    << ", y=" << current.Value().y() << ", yaw=" << current.Value().yaw() << '\n';
  ok = SetMode(chassis, ::xr::sdk::v2::GLOBAL) && ok;
  ::xr::sdk::v2::ChassisPosition target; target.set_x(1.2); target.set_y(-0.2); target.set_yaw(0.0);
  ok = check(chassis.MoveToGlobalPosition(target, 5s), "Chassis.MoveToGlobalPosition") && ok;
  std::this_thread::sleep_for(2s);
  current = chassis.GetGlobalPosition(3s); ok = check(current, "Chassis.GetGlobalPosition(after)") && ok;
  if (current.Ok()) std::cout << "current global position: x=" << current.Value().x()
                              << ", y=" << current.Value().y() << ", yaw=" << current.Value().yaw() << '\n';
  return ok;
}

bool MoveRelative(x2robot::sdk::ChassisController& chassis, bool cancel) {
  auto current = chassis.GetGlobalPosition(3s);
  bool ok = check(current, "Chassis.GetGlobalPosition");
  if (!ok) return false;
  std::cout << "current global position: x=" << current.Value().x() << ", y=" << current.Value().y()
            << ", yaw=" << current.Value().yaw() << '\n';
  ok = check(chassis.SetVirtualZeroPoint(current.Value(), 3s), "Chassis.SetVirtualZeroPoint") && ok;
  ok = SetMode(chassis, ::xr::sdk::v2::RELATIVE) && ok;
  ::xr::sdk::v2::ChassisPosition target; target.set_x(0.85);
  std::cout << "move to relative position 0.85 meters forward\n";
  ok = check(chassis.MoveToRelativePosition(target, 5s), "Chassis.MoveToRelativePosition") && ok;
  if (!cancel) std::this_thread::sleep_for(2s);
  auto relative = chassis.GetRelativePosition(3s); ok = check(relative, "Chassis.GetRelativePosition") && ok;
  if (relative.Ok()) std::cout << "current relative position: x=" << relative.Value().x()
                                << ", y=" << relative.Value().y() << ", yaw=" << relative.Value().yaw() << '\n';
  return ok;
}

bool MoveByMap(Robot& robot, bool cancel) {
  auto navigation = robot.GetNavigation(); auto chassis = robot.GetChassisController(); bool ok = true;
  ::xr::sdk::v2::NavigationModeParam nav; nav.set_mode(::xr::sdk::v2::BUILT_IN_NAVIGATION);
  ok = check(navigation.SetNavigationMode(nav, 3s), "Navigation.SetNavigationMode") && ok;
  ::xr::sdk::v2::CoordinateSystemModeParam coord;
  coord.set_coordinate_system_mode(::xr::sdk::v2::COORDINATE_SYSTEM_MODE_MAP);
  ok = check(chassis.SetTrajectoryCoordSystemMode(coord, 3s), "Chassis.SetTrajectoryCoordSystemMode") && ok;
  ok = check(navigation.StartMapping(3s), "Navigation.StartMapping") && ok;
  std::cout << "move around to build map...\n"; ok = MoveByVelocity(chassis) && ok;
  std::this_thread::sleep_for(1s);
  ::xr::sdk::v2::SaveMapParam save; save.set_map_name("test");
  ok = check(navigation.StopMapping(save, 10s), "Navigation.StopMapping") && ok;
  ::xr::sdk::v2::StartLocalizationParam localization; localization.set_map_name("test"); localization.set_use_init_pose(false);
  ok = check(navigation.StartLocalization(localization, 10s), "Navigation.StartLocalization") && ok;
  std::this_thread::sleep_for(2s); ok = MoveRelative(chassis, cancel) && ok;
  if (cancel) ok = check(navigation.CancelNavigation(3s), "Navigation.CancelNavigation") && ok;
  auto odom = chassis.GetOdometry(3s); ok = check(odom, "Chassis.GetOdometry") && ok;
  if (odom.Ok()) PrintOdom(odom.Value()); return ok;
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string mode = argc > 2 ? argv[2] : "keyboard";
  std::signal(SIGINT, OnSignal);
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value()); bool ok = true;
  ::xr::sdk::v2::RobotModeParam sdk_mode; sdk_mode.set_mode(::xr::sdk::v2::SDK);
  ok = check(x2robot::examples::NoteWorkMode(robot, sdk_mode, robot->GetSystem().SetWorkMode(sdk_mode, 3s)), "System.SetWorkMode") && ok;
  auto chassis = robot->GetChassisController();
  if (mode == "global") ok = MoveGlobal(chassis);
  else if (mode == "relative") ok = MoveRelative(chassis, false);
  else if (mode == "velocity") ok = MoveByVelocity(chassis);
  else if (mode == "map") ok = MoveByMap(*robot, false);
  else if (mode == "map_cancel") ok = MoveByMap(*robot, true);
  else if (mode == "keyboard") ok = RunKeyboard(chassis);
  else { std::cerr << "Valid modes: map, map_cancel, keyboard, global, relative, velocity\n"; ok = false; }
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = check(robot->Close(), "Robot.Close") && ok; return ok ? 0 : 1;
}
