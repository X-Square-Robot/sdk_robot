// Counterpart of python/v1/quanta_x1/arm_control.py.
// The Python example uses TOPP-RA.  This dependency-free C++ version sends
// the same reset/target waypoints with a bounded linear interpolation.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"
#include "sdk_mode_cleanup.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using namespace std::chrono_literals;

namespace {
std::atomic_bool stop{false};
// 轨迹命令的 RPC deadline 不能按本机环回（<1 ms）来设：现场常经交换机/WiFi
// 访问主控（例如 10.137.20.54 实测往返约 20 ms），10 ms 会让每条命令都报
// DeadlineExceeded。这里取 500 ms，与 quanta_x2/x3 的手臂示例一致。
constexpr auto kCommandDeadline = 500ms;
void signal_handler(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }
}

template <typename T>
bool check(const SdkResult<T>& result, const char* name) {
  if (result.Ok()) return true;
  std::cerr << name << " failed: " << result.ErrorValue().error_message << '\n';
  return false;
}

template <typename Arm>
bool MoveJoints(Arm& arm, const std::vector<double>& target) {
  auto current = arm.GetJointStates(3s);
  if (!check(current, "Arm.GetJointStates")) return false;
  if (current.Value().position_size() != static_cast<int>(target.size())) {
    std::cerr << "unexpected joint count: " << current.Value().position_size()
              << " (expected " << target.size() << ")\n";
    return false;
  }
  constexpr int steps = 200;
  for (int step = 1; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / steps;
    ::xr::sdk::v2::JointPositions command;
    for (int joint = 0; joint < current.Value().position_size(); ++joint) {
      command.add_positions(current.Value().position(joint) * (1.0 - ratio) +
                            target[joint] * ratio);
    }
    // 失败就停：继续把后面的轨迹点发下去会在伺服恢复后造成位置跳变。
    if (!check(arm.SetJointPositions(command, kCommandDeadline),
               "Arm.SetJointPositions")) return false;
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

template <typename Arm>
bool MovePose(Arm& arm, const ::geometry_msgs::Pose& target) {
  auto current = arm.GetEndPose(3s);
  if (!check(current, "Arm.GetEndPose")) return false;
  const auto start = current.Value().pose();
  constexpr int steps = 200;
  for (int step = 1; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / steps;
    ::geometry_msgs::Pose command = start;
    command.mutable_position()->set_x(
        start.position().x() * (1.0 - ratio) + target.position().x() * ratio);
    command.mutable_position()->set_y(
        start.position().y() * (1.0 - ratio) + target.position().y() * ratio);
    command.mutable_position()->set_z(
        start.position().z() * (1.0 - ratio) + target.position().z() * ratio);
    // Match the endpoint orientation.  The position trajectory is the part
    // that matters for this example and avoids pulling in an extra math lib.
    command.mutable_orientation()->CopyFrom(target.orientation());
    if (!check(arm.SetEndPose(command, kCommandDeadline), "Arm.SetEndPose"))
      return false;
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

template <typename Arm, typename RobotPtr>
bool Run(Arm arm, RobotPtr& robot, const std::string& action,
         const std::string& mode) {
  if (action == "stream") {
    bool ok = true;
    if (mode == "end_pose") {
      auto opened = arm.GetEndPoseStream(5s);
      if (!check(opened, "Arm.GetEndPoseStream")) return false;
      auto stream = std::move(opened.Value());
      std::cout << "Starting arm end pose streaming (Ctrl+C to stop)...\n";
      while (!stop) {
        auto value = stream->Read(2s);
        if (!check(value, "Arm stream read")) { ok = false; break; }
        std::cout << "end_pose: " << value.Value().ShortDebugString() << '\n';
      }
      return check(stream->Cancel(), "Arm.EndPoseStream.Cancel") && ok;
    }
    auto opened = arm.GetJointStatesStream(5s);
    if (!check(opened, "Arm.GetJointStatesStream")) return false;
    auto stream = std::move(opened.Value());
    std::cout << "Starting arm joint states streaming (Ctrl+C to stop)...\n";
    while (!stop) {
      auto value = stream->Read(2s);
      if (!check(value, "Arm stream read")) { ok = false; break; }
      std::cout << "joint_state: " << value.Value().ShortDebugString() << '\n';
    }
    return check(stream->Cancel(), "Arm.JointStatesStream.Cancel") && ok;
  }

  if (action != "move") {
    std::cerr << "Valid actions: move, stream; modes: joint_pos, end_pose\n";
    return false;
  }

  ::xr::sdk::v2::RobotModeParam sdk;
  sdk.set_mode(::xr::sdk::v2::SDK);
  bool ok = check(x2robot::examples::NoteWorkMode(robot, sdk, robot->GetSystem().SetWorkMode(sdk, 3s)), "System.SetWorkMode");
  ::xr::sdk::v2::ManipulatorControlModeParam control;
  control.set_mode(mode == "end_pose"
                       ? ::xr::sdk::v2::MANIPULATOR_END_POSE
                       : ::xr::sdk::v2::MANIPULATOR_JOINT_POSITIONS);
  ok = check(robot->GetRobotControl().SetManipulatorControlMode(control, 3s),
             "RobotControl.SetManipulatorControlMode") && ok;
  if (!ok) {
    std::cerr << "not starting the trajectory: SDK mode/control mode setup failed\n";
    return false;
  }

  if (mode == "end_pose") {
    ::geometry_msgs::Pose reset;
    reset.mutable_orientation()->set_w(1.0);
    std::cout << "Starting end pose reset...\n";
    if (!MovePose(arm, reset)) return false;
    std::this_thread::sleep_for(2s);
    ::geometry_msgs::Pose target;
    target.mutable_position()->set_z(0.2);
    target.mutable_orientation()->set_x(-0.0076);
    target.mutable_orientation()->set_y(0.0868);
    target.mutable_orientation()->set_z(0.0868);
    target.mutable_orientation()->set_w(0.9924);
    std::cout << "Executing trajectory control...\n";
    if (!MovePose(arm, target)) return false;
  } else {
    std::cout << "Starting joint reset...\n";
    if (!MoveJoints(arm, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) return false;
    std::this_thread::sleep_for(1s);
    const std::vector<double> target{-0.1486, 0.4707, -0.8101, 0.6350, 0.3164,
                                     0.0};
    std::cout << "Moving to target joint angles\n";
    if (!MoveJoints(arm, target)) return false;
  }
  std::this_thread::sleep_for(2s);
  std::cout << "Demo finished.\n";
  return true;
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string side = argc > 2 ? argv[2] : "left";
  const std::string action = argc > 3 ? argv[3] : "move";
  const std::string mode = argc > 4 ? argv[4] : "joint_pos";
  std::signal(SIGINT, signal_handler);

  auto created = Robot::Create(address, true);
  if (!check(created, "Robot.Create")) return 1;
  auto robot = std::move(created.Value());
  bool ok = side == "right"
                ? Run(robot->GetRightArmController(), robot, action, mode)
                : Run(robot->GetLeftArmController(), robot, action, mode);
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
