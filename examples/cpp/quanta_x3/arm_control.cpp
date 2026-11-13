// Quanta X3 counterpart of python/v2/quanta_x3/arm_control.py.
// The C++ sample mirrors the Python TOPP-RA trajectory without adding a
// toppra runtime dependency: for a straight path, TOPP-RA reduces to a
// scalar trapezoidal/triangular velocity profile.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"
#include "sdk_mode_cleanup.hpp"
#include "linear_toppra_trajectory.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using namespace std::chrono_literals;

namespace {
std::atomic_bool stop{false};
void signal_handler(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }
}

template <typename T>
bool check(const SdkResult<T>& result, const char* name) {
  if (result.Ok()) return true;
  std::cerr << name << " failed: " << result.ErrorValue().error_message << '\n';
  return false;
}

constexpr std::size_t kQuantaX3JointCount = 7;
constexpr std::array<double, kQuantaX3JointCount> kJointLowerLimits = {
    -3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708};
constexpr std::array<double, kQuantaX3JointCount> kJointUpperLimits = {
    3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708};

bool IsFinite(double value) { return std::isfinite(value); }

struct QuaternionValues {
  double x;
  double y;
  double z;
  double w;
};

double Dot(const QuaternionValues& lhs, const QuaternionValues& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
}

bool NormalizeQuaternion(const ::geometry_msgs::Quaternion& input,
                         QuaternionValues* output, const char* name) {
  const QuaternionValues value{input.x(), input.y(), input.z(), input.w()};
  const double norm = std::sqrt(Dot(value, value));
  if (!IsFinite(norm) || norm < 1e-9) {
    std::cerr << name << " contains an invalid quaternion\n";
    return false;
  }
  output->x = value.x / norm;
  output->y = value.y / norm;
  output->z = value.z / norm;
  output->w = value.w / norm;
  return true;
}

QuaternionValues Negate(const QuaternionValues& value) {
  return {-value.x, -value.y, -value.z, -value.w};
}

QuaternionValues Slerp(const QuaternionValues& start,
                       const QuaternionValues& end, double alpha) {
  QuaternionValues adjusted_end = end;
  double dot = Dot(start, adjusted_end);
  // scipy.spatial.transform.Slerp takes the shortest rotation path too.
  if (dot < 0.0) {
    adjusted_end = Negate(adjusted_end);
    dot = -dot;
  }
  dot = std::clamp(dot, -1.0, 1.0);
  alpha = std::clamp(alpha, 0.0, 1.0);

  QuaternionValues result{};
  if (dot > 1.0 - 1e-6) {
    result.x = start.x + alpha * (adjusted_end.x - start.x);
    result.y = start.y + alpha * (adjusted_end.y - start.y);
    result.z = start.z + alpha * (adjusted_end.z - start.z);
    result.w = start.w + alpha * (adjusted_end.w - start.w);
    const double norm = std::sqrt(Dot(result, result));
    if (norm > 1e-12) {
      result.x /= norm;
      result.y /= norm;
      result.z /= norm;
      result.w /= norm;
    }
    return result;
  }

  const double angle = std::acos(dot);
  const double sine = std::sin(angle);
  const double start_weight = std::sin((1.0 - alpha) * angle) / sine;
  const double end_weight = std::sin(alpha * angle) / sine;
  result.x = start_weight * start.x + end_weight * adjusted_end.x;
  result.y = start_weight * start.y + end_weight * adjusted_end.y;
  result.z = start_weight * start.z + end_weight * adjusted_end.z;
  result.w = start_weight * start.w + end_weight * adjusted_end.w;
  return result;
}

bool FinitePose(const ::geometry_msgs::Pose& pose, const char* name) {
  const auto& position = pose.position();
  const auto& orientation = pose.orientation();
  if (IsFinite(position.x()) && IsFinite(position.y()) && IsFinite(position.z()) &&
      IsFinite(orientation.x()) && IsFinite(orientation.y()) &&
      IsFinite(orientation.z()) && IsFinite(orientation.w())) {
    return true;
  }
  std::cerr << name << " contains non-finite values\n";
  return false;
}

using x2robot::examples::linear_toppra::ScalarTrajectory;
using x2robot::examples::linear_toppra::MakeTrajectory;

template <typename Arm>
bool MoveJoints(Arm& arm, const std::vector<double>& target) {
  if (target.size() != kQuantaX3JointCount) {
    std::cerr << "unexpected target joint count: " << target.size()
              << " (expected " << kQuantaX3JointCount << ")\n";
    return false;
  }
  for (std::size_t joint = 0; joint < kQuantaX3JointCount; ++joint) {
    if (!IsFinite(target[joint]) || target[joint] < kJointLowerLimits[joint] ||
        target[joint] > kJointUpperLimits[joint]) {
      std::cerr << "target joint " << joint << " is outside the Quanta X3 limits\n";
      return false;
    }
  }

  auto current = arm.GetJointStates(3s);
  if (!check(current, "Arm.GetJointStates")) return false;
  if (current.Value().position_size() !=
      static_cast<int>(kQuantaX3JointCount)) {
    std::cerr << "unexpected joint count: " << current.Value().position_size()
              << " (expected " << kQuantaX3JointCount << ")\n";
    return false;
  }

  std::array<double, kQuantaX3JointCount> start{};
  for (std::size_t joint = 0; joint < kQuantaX3JointCount; ++joint) {
    const double value = current.Value().position(static_cast<int>(joint));
    if (!IsFinite(value)) {
      std::cerr << "Arm.GetJointStates returned a non-finite position\n";
      return false;
    }
    // Match the Python example's sensor-drift guard before planning.
    start[joint] = std::clamp(value, kJointLowerLimits[joint],
                              kJointUpperLimits[joint]);
  }

  constexpr double kVelocityLimit = 1.0;
  constexpr double kAccelerationLimit = 3.0;
  double path_velocity_limit = std::numeric_limits<double>::infinity();
  double path_acceleration_limit = std::numeric_limits<double>::infinity();
  for (std::size_t joint = 0; joint < kQuantaX3JointCount; ++joint) {
    const double displacement = std::abs(target[joint] - start[joint]);
    if (displacement <= 1e-12) continue;
    path_velocity_limit =
        std::min(path_velocity_limit, kVelocityLimit / displacement);
    path_acceleration_limit =
        std::min(path_acceleration_limit, kAccelerationLimit / displacement);
  }
  if (!IsFinite(path_velocity_limit)) path_velocity_limit = kVelocityLimit;
  if (!IsFinite(path_acceleration_limit)) path_acceleration_limit = kAccelerationLimit;

  ScalarTrajectory trajectory(1.0, path_velocity_limit,
                              path_acceleration_limit);
  if (!MakeTrajectory(1.0, path_velocity_limit, path_acceleration_limit,
                      &trajectory)) {
    return false;
  }
  constexpr auto kInterval = 5ms;  // 200 Hz maximum SDK control rate.
  std::cout << "Executing TOPP-RA joint trajectory: Estimated duration "
            << trajectory.duration() << "s\n";
  const auto started = std::chrono::steady_clock::now();
  for (double timestamp = 0.0; timestamp < trajectory.duration();
       timestamp = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()) {
    // RPC 耗时计入轨迹进度，不补发已经过期的采样点。
    const auto next_sample = std::chrono::steady_clock::now() + kInterval;
    const double alpha = std::clamp(trajectory.Position(timestamp), 0.0, 1.0);
    ::xr::sdk::v2::JointPositions command;
    for (std::size_t joint = 0; joint < kQuantaX3JointCount; ++joint) {
      const double position = std::clamp(
          start[joint] * (1.0 - alpha) + target[joint] * alpha,
          kJointLowerLimits[joint], kJointUpperLimits[joint]);
      command.add_positions(position);
    }
    if (!check(arm.SetJointPositions(command, 500ms),
               "Arm.SetJointPositions")) {
      return false;
    }
    std::this_thread::sleep_until(next_sample);
  }

  ::xr::sdk::v2::JointPositions final_command;
  for (double position : target) final_command.add_positions(position);
  return check(arm.SetJointPositions(final_command, 500ms),
               "Arm.SetJointPositions(final)");
}

// WBC 残余误差仅用于展示；通信失败、无效位姿仍中止演示。
bool PoseError(const ::geometry_msgs::Pose& actual, const ::geometry_msgs::Pose& target,
               std::array<double, 2>* error)
{
    if (!FinitePose(actual, "end-pose feedback") || !FinitePose(target, "end-pose target")) return false;
    QuaternionValues a{}, b{};
    if (!NormalizeQuaternion(actual.orientation(), &a, "feedback") ||
        !NormalizeQuaternion(target.orientation(), &b, "target"))
        return false;
    const auto& p = actual.position();
    const auto& q = target.position();
    *error = {std::hypot(p.x() - q.x(), p.y() - q.y(), p.z() - q.z()),
              2 * std::acos(std::clamp(std::abs(Dot(a, b)), 0.0, 1.0))};
    return true;
}

template <typename Arm, typename OtherArm, typename Waist>
bool HoldEndPose(Arm& arm,
                 OtherArm& other,
                 Waist& waist,
                 const ::geometry_msgs::Pose& target,
                 const ::geometry_msgs::Pose& other_pose,
                 const ::geometry_msgs::Pose& waist_pose)
{
    // 与 Python 一致：持续刷新三路目标 1 秒，不等待精确到位。
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    std::array<std::array<double, 2>, 3> errors{};
    while (true)
    {
        if (!check(arm.SetEndPose(target, 500ms), "Arm.SetEndPose(hold)") ||
            !check(other.SetEndPose(other_pose, 500ms), "OtherArm.SetEndPose(hold)") ||
            !check(waist.SetEndPose(waist_pose, 500ms), "Waist.SetEndPose(hold)"))
            return false;
        auto actual = arm.GetEndPose(500ms);
        auto other_actual = other.GetEndPose(500ms);
        auto waist_actual = waist.GetEndPose(500ms);
        if (!check(actual, "Arm.GetEndPose(hold)") || !check(other_actual, "OtherArm.GetEndPose(hold)") ||
            !check(waist_actual, "Waist.GetEndPose(hold)"))
            return false;
        if (!PoseError(actual.Value().pose(), target, &errors[0]) ||
            !PoseError(other_actual.Value().pose(), other_pose, &errors[1]) ||
            !PoseError(waist_actual.Value().pose(), waist_pose, &errors[2]))
            return false;
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break;
        std::this_thread::sleep_until(std::min(now + 5ms, deadline));
    }
    constexpr std::array<const char*, 3> names{"arm", "other", "waist"};
    std::cout << "End-pose target hold complete; residual errors (mm, deg): ";
    for (std::size_t i = 0; i < errors.size(); ++i)
    {
        if (i) std::cout << ", ";
        std::cout << names[i] << "=(" << errors[i][0] * 1000 << ", "
                  << errors[i][1] * 180 / std::acos(-1.0) << ')';
    }
    std::cout << '\n';
    return true;
}

template <typename Arm, typename OtherArm, typename Waist>
bool MovePose(Arm& arm,
              OtherArm& other_arm,
              Waist& waist,
              const ::geometry_msgs::Pose& target,
              const ::geometry_msgs::Pose& other_pose,
              const ::geometry_msgs::Pose& waist_pose)
{
    auto current = arm.GetEndPose(3s);
    if (!check(current, "Arm.GetEndPose")) return false;
    if (!FinitePose(target, "target pose")) return false;

    const auto start = current.Value().pose();
    if (!FinitePose(start, "Arm.GetEndPose")) return false;
    QuaternionValues start_orientation{};
    QuaternionValues target_orientation{};
    if (!NormalizeQuaternion(start.orientation(), &start_orientation, "Arm.GetEndPose orientation") ||
        !NormalizeQuaternion(target.orientation(), &target_orientation, "target orientation"))
    {
        return false;
    }

    const double dx = target.position().x() - start.position().x();
    const double dy = target.position().y() - start.position().y();
    const double dz = target.position().z() - start.position().z();
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double path_length = distance > 1e-6 ? distance : 1.0;
    constexpr double kVelocityLimit = 2.2;
    constexpr double kAccelerationLimit = 0.3;
    ScalarTrajectory trajectory(path_length, kVelocityLimit, kAccelerationLimit);
    if (!MakeTrajectory(path_length, kVelocityLimit, kAccelerationLimit, &trajectory))
    {
        return false;
    }

    constexpr auto kInterval = 5ms;  // 200 Hz, matching the Python example.
    std::cout << "Executing TOPP-RA smooth trajectory: Estimated duration " << trajectory.duration() << "s\n";
    const auto started = std::chrono::steady_clock::now();
    for (double timestamp = 0.0; timestamp < trajectory.duration();
         timestamp = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count())
    {
        const auto next_sample = std::chrono::steady_clock::now() + kInterval;
        const double alpha = std::clamp(trajectory.Position(timestamp) / path_length, 0.0, 1.0);
        ::geometry_msgs::Pose command = start;
        command.mutable_position()->set_x(start.position().x() + dx * alpha);
        command.mutable_position()->set_y(start.position().y() + dy * alpha);
        command.mutable_position()->set_z(start.position().z() + dz * alpha);
        const auto orientation = Slerp(start_orientation, target_orientation, alpha);
        command.mutable_orientation()->set_x(orientation.x);
        command.mutable_orientation()->set_y(orientation.y);
        command.mutable_orientation()->set_z(orientation.z);
        command.mutable_orientation()->set_w(orientation.w);
        if (!check(arm.SetEndPose(command, 500ms), "Arm.SetEndPose") ||
            !check(other_arm.SetEndPose(other_pose, 500ms), "OtherArm.SetEndPose") ||
            !check(waist.SetEndPose(waist_pose, 3s), "Waist.SetEndPose"))
        {
            return false;
        }
        std::this_thread::sleep_until(next_sample);
    }

    if (!check(arm.SetEndPose(target, 500ms), "Arm.SetEndPose(final)") ||
        !check(other_arm.SetEndPose(other_pose, 500ms), "OtherArm.SetEndPose(final)") ||
        !check(waist.SetEndPose(waist_pose, 3s), "Waist.SetEndPose(final)"))
    {
        return false;
    }
    return HoldEndPose(arm, other_arm, waist, target, other_pose, waist_pose);
}

template <typename Arm, typename OtherArm, typename RobotPtr>
bool Run(Arm arm, OtherArm other_arm, RobotPtr& robot,
         const std::string& action,
         const std::string& mode, const std::string& side, bool assume_yes) {
  if (mode != "joint_pos" && mode != "end_pose") {
    std::cerr << "Valid modes: joint_pos, end_pose\n";
    return false;
  }
  if (action == "stream") {
    if (mode == "end_pose") {
      // 与 Python 示例一致：先准备 SDK 和末端模式，再等待末端反馈。
      ::xr::sdk::v2::RobotModeParam sdk;
      sdk.set_mode(::xr::sdk::v2::SDK);
      if (!check(x2robot::examples::NoteWorkMode(
                     robot, sdk, robot->GetSystem().SetWorkMode(sdk, 5s)),
                 "System.SetWorkMode")) {
        return false;
      }
      ::xr::sdk::v2::ManipulatorControlModeParam control;
      control.set_mode(::xr::sdk::v2::MANIPULATOR_END_POSE);
      if (!check(robot->GetRobotControl().SetManipulatorControlMode(control, 5s),
                 "RobotControl.SetManipulatorControlMode")) {
        return false;
      }
      std::this_thread::sleep_for(1s);
      auto opened = arm.GetEndPoseStream(5s);
      if (!check(opened, "Arm.GetEndPoseStream")) return false;
      auto stream = std::move(opened.Value());
      std::cout << "Starting arm end pose streaming (Ctrl+C to stop)...\n";
      while (!stop) {
        auto value = stream->Read(2s);
        if (!value.Ok()) break;
        std::cout << "end_pose: " << value.Value().ShortDebugString() << '\n';
      }
      return check(stream->Cancel(), "Arm.EndPoseStream.Cancel");
    }
    auto opened = arm.GetJointStatesStream(5s);
    if (!check(opened, "Arm.GetJointStatesStream")) return false;
    auto stream = std::move(opened.Value());
    std::cout << "Starting arm joint states streaming (Ctrl+C to stop)...\n";
    while (!stop) {
      auto value = stream->Read(2s);
      if (!value.Ok()) break;
      std::cout << "joint_state: " << value.Value().ShortDebugString() << '\n';
    }
    return check(stream->Cancel(), "Arm.JointStatesStream.Cancel");
  }

  if (action != "move") {
    std::cerr << "Valid actions: move, stream; modes: joint_pos, end_pose\n";
    return false;
  }
  if (!assume_yes) {
    std::cout << "About to run Quanta X3 " << side << " arm " << mode
              << " control. The robot will move. Press Enter to continue or Ctrl+C to cancel "
              << "(use --yes for automation): " << std::flush;
    std::string confirmation;
    if (!std::getline(std::cin, confirmation)) {
      std::cerr << "Motion confirmation is required. Press Enter in an interactive terminal or "
                   "pass --yes for automation.\n";
      return false;
    }
  }

  ::xr::sdk::v2::RobotModeParam sdk;
  sdk.set_mode(::xr::sdk::v2::SDK);
  if (!check(x2robot::examples::NoteWorkMode(robot, sdk, robot->GetSystem().SetWorkMode(sdk, 3s)), "System.SetWorkMode")) {
    return false;
  }
  ::xr::sdk::v2::ManipulatorControlModeParam control;
  control.set_mode(mode == "end_pose"
                       ? ::xr::sdk::v2::MANIPULATOR_END_POSE
                       : ::xr::sdk::v2::MANIPULATOR_JOINT_POSITIONS);
  if (!check(robot->GetRobotControl().SetManipulatorControlMode(control, 3s),
             "RobotControl.SetManipulatorControlMode")) {
    return false;
  }

  auto waist = robot->GetWaistController();

  if (mode == "end_pose") {
    std::this_thread::sleep_for(1s);
    // Quanta X3's whole-body controller requires the three reasoning targets
    // (left arm, right arm and waist) to be refreshed together in end-pose mode.
    // Hold the other arm and the waist at their current feedback values while the
    // selected arm follows the trajectory; the waist target is published on
    // /whole_body_controller/reasoning_target_head.
    auto other_current = other_arm.GetEndPose(3s);
    if (!check(other_current, "OtherArm.GetEndPose")) return false;
    const auto other_pose = other_current.Value().pose();
    if (!FinitePose(other_pose, "OtherArm.GetEndPose")) return false;
    QuaternionValues other_orientation{};
    if (!NormalizeQuaternion(other_pose.orientation(), &other_orientation, "OtherArm.GetEndPose orientation"))
    {
        return false;
    }

    auto waist_current = waist.GetEndPose(3s);
    if (!check(waist_current, "Waist.GetEndPose")) return false;
    const auto waist_pose = waist_current.Value().pose();
    if (!FinitePose(waist_pose, "Waist.GetEndPose")) return false;
    QuaternionValues waist_orientation{};
    if (!NormalizeQuaternion(waist_pose.orientation(), &waist_orientation, "Waist.GetEndPose orientation"))
    {
        return false;
    }

    ::geometry_msgs::Pose reset;
    // 与 Python X3 示例使用同一组实测姿态，避免沿用其他机型的逆解分支。
    reset.mutable_position()->set_x(0.125556);
    reset.mutable_position()->set_y(side == "left" ? 0.193477 : -0.193477);
    reset.mutable_position()->set_z(0.857640);
    reset.mutable_orientation()->set_x(0.001356);
    reset.mutable_orientation()->set_y(0.002202);
    reset.mutable_orientation()->set_z(0.708243);
    reset.mutable_orientation()->set_w(0.705964);
    std::cout << "Starting end pose reset...\n";
    if (!MovePose(arm, other_arm, waist, reset, other_pose, waist_pose)) return false;
    // 保持同一组另一臂/腰目标，避免阶段切换累计偏差。
    ::geometry_msgs::Pose target = reset;
    // 沿 z 方向抬升 10 cm，保持准备位的 x/y 和姿态。
    target.mutable_position()->set_z(reset.position().z() + 0.1);
    std::cout << "Executing smooth end-pose trajectory...\n";
    if (!MovePose(arm, other_arm, waist, target, other_pose, waist_pose)) return false;
  } else {
    const std::vector<double> reset =
        side == "left"
            ? std::vector<double>{0.002453, 1.570471, 1.570013, -0.591459, -0.002960, -0.005304, 0.002296}
            : std::vector<double>{-0.000184, 1.568038, -1.567079, -0.591765, -0.000566, -0.005404, 0.000034};
    std::vector<double> target = reset;
    target[3] += 0.2;
    std::cout << "Starting joint reset...\n";
    if (!MoveJoints(arm, reset)) return false;
    std::this_thread::sleep_for(1s);
    std::cout << "Moving " << side << " arm to the safe target joint angles\n";
    if (!MoveJoints(arm, target)) return false;
  }
  std::this_thread::sleep_for(2s);
  std::cout << "Demo finished.\n";
  return true;
}

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "quanta_x3_arm_control [address] [left|right] [move|stream] "
                 "[joint_pos|end_pose] [--yes]\n";
    return 0;
  }
  if (argc > 6 || (argc == 6 && std::string(argv[5]) != "--yes")) {
    std::cerr << "Invalid arguments; use --help\n";
    return 2;
  }
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string side = argc > 2 ? argv[2] : "left";
  const std::string action = argc > 3 ? argv[3] : "move";
  const std::string mode = argc > 4 ? argv[4] : "joint_pos";
  const bool assume_yes = argc > 5 && std::string(argv[5]) == "--yes";
  if (side != "left" && side != "right") {
    std::cerr << "side must be left or right\n";
    return 2;
  }
  std::signal(SIGINT, signal_handler);

  auto created = Robot::Create(address, true);
  if (!check(created, "Robot.Create")) return 1;
  auto robot = std::move(created.Value());
  bool ok = side == "right"
                ? Run(robot->GetRightArmController(),
                      robot->GetLeftArmController(), robot, action, mode, side,
                      assume_yes)
                : Run(robot->GetLeftArmController(),
                      robot->GetRightArmController(), robot, action, mode, side,
                      assume_yes);
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
