// Counterpart of python/v2/quanta_x2/arm_control.py.
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

constexpr std::size_t kQuantaX2JointCount = 7;
constexpr std::array<double, kQuantaX2JointCount> kJointLowerLimits = {
    -3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708};
constexpr std::array<double, kQuantaX2JointCount> kJointUpperLimits = {
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
  if (target.size() != kQuantaX2JointCount) {
    std::cerr << "unexpected target joint count: " << target.size()
              << " (expected " << kQuantaX2JointCount << ")\n";
    return false;
  }
  for (std::size_t joint = 0; joint < kQuantaX2JointCount; ++joint) {
    if (!IsFinite(target[joint]) || target[joint] < kJointLowerLimits[joint] ||
        target[joint] > kJointUpperLimits[joint]) {
      std::cerr << "target joint " << joint << " is outside the Quanta X2 limits\n";
      return false;
    }
  }

  auto current = arm.GetJointStates(3s);
  if (!check(current, "Arm.GetJointStates")) return false;
  if (current.Value().position_size() !=
      static_cast<int>(kQuantaX2JointCount)) {
    std::cerr << "unexpected joint count: " << current.Value().position_size()
              << " (expected " << kQuantaX2JointCount << ")\n";
    return false;
  }

  std::array<double, kQuantaX2JointCount> start{};
  for (std::size_t joint = 0; joint < kQuantaX2JointCount; ++joint) {
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
  for (std::size_t joint = 0; joint < kQuantaX2JointCount; ++joint) {
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
  constexpr auto kInterval = 2ms;  // 500 Hz, matching the Python example.
  std::cout << "Executing TOPP-RA joint trajectory: Estimated duration "
            << trajectory.duration() << "s\n";
  for (double timestamp = 0.0; timestamp < trajectory.duration();
       timestamp += std::chrono::duration<double>(kInterval).count()) {
    const double alpha = std::clamp(trajectory.Position(timestamp), 0.0, 1.0);
    ::xr::sdk::v2::JointPositions command;
    for (std::size_t joint = 0; joint < kQuantaX2JointCount; ++joint) {
      const double position = std::clamp(
          start[joint] * (1.0 - alpha) + target[joint] * alpha,
          kJointLowerLimits[joint], kJointUpperLimits[joint]);
      command.add_positions(position);
    }
    if (!check(arm.SetJointPositions(command, 500ms),
               "Arm.SetJointPositions")) {
      return false;
    }
    std::this_thread::sleep_for(kInterval);
  }

  ::xr::sdk::v2::JointPositions final_command;
  for (double position : target) final_command.add_positions(position);
  return check(arm.SetJointPositions(final_command, 500ms),
               "Arm.SetJointPositions(final)");
}

template <typename Arm, typename OtherArm, typename Waist>
bool MovePose(Arm& arm, const ::geometry_msgs::Pose& target,
              OtherArm& other_arm, Waist& waist) {
  auto current = arm.GetEndPose(3s);
  if (!check(current, "Arm.GetEndPose")) return false;
  if (!FinitePose(target, "target pose")) return false;

  const auto start = current.Value().pose();
  if (!FinitePose(start, "Arm.GetEndPose")) return false;
  QuaternionValues start_orientation{};
  QuaternionValues target_orientation{};
  if (!NormalizeQuaternion(start.orientation(), &start_orientation,
                           "Arm.GetEndPose orientation") ||
      !NormalizeQuaternion(target.orientation(), &target_orientation,
                           "target orientation")) {
    return false;
  }

  auto other_current = other_arm.GetEndPose(3s);
  if (!check(other_current, "OtherArm.GetEndPose")) return false;
  const auto other_pose = other_current.Value().pose();
  if (!FinitePose(other_pose, "OtherArm.GetEndPose")) return false;
  QuaternionValues unused_orientation{};
  if (!NormalizeQuaternion(other_pose.orientation(), &unused_orientation,
                           "OtherArm.GetEndPose orientation")) {
    return false;
  }

  auto waist_current = waist.GetEndPose(3s);
  if (!check(waist_current, "Waist.GetEndPose")) return false;
  const auto waist_pose = waist_current.Value().pose();
  if (!FinitePose(waist_pose, "Waist.GetEndPose")) return false;
  if (!NormalizeQuaternion(waist_pose.orientation(), &unused_orientation,
                           "Waist.GetEndPose orientation")) {
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
  if (!MakeTrajectory(path_length, kVelocityLimit, kAccelerationLimit,
                      &trajectory)) {
    return false;
  }

  constexpr auto kInterval = 5ms;  // 200 Hz, matching the Python example.
  std::cout << "Executing TOPP-RA smooth trajectory: Estimated duration "
            << trajectory.duration() << "s\n";
  for (double timestamp = 0.0; timestamp < trajectory.duration();
       timestamp += std::chrono::duration<double>(kInterval).count()) {
    const double alpha = std::clamp(trajectory.Position(timestamp) / path_length,
                                    0.0, 1.0);
    ::geometry_msgs::Pose command = start;
    command.mutable_position()->set_x(start.position().x() + dx * alpha);
    command.mutable_position()->set_y(start.position().y() + dy * alpha);
    command.mutable_position()->set_z(start.position().z() + dz * alpha);
    const auto orientation =
        Slerp(start_orientation, target_orientation, alpha);
    command.mutable_orientation()->set_x(orientation.x);
    command.mutable_orientation()->set_y(orientation.y);
    command.mutable_orientation()->set_z(orientation.z);
    command.mutable_orientation()->set_w(orientation.w);

    // CX002's whole-body controller expects all three pose streams to be
    // refreshed together, just as the Python example does.
    if (!check(arm.SetEndPose(command, 500ms), "Arm.SetEndPose") ||
        !check(other_arm.SetEndPose(other_pose, 500ms),
               "OtherArm.SetEndPose") ||
        !check(waist.SetEndPose(waist_pose, 3s), "Waist.SetEndPose")) {
      return false;
    }
    std::this_thread::sleep_for(kInterval);
  }

  if (!check(arm.SetEndPose(target, 500ms), "Arm.SetEndPose(final)") ||
      !check(other_arm.SetEndPose(other_pose, 500ms),
             "OtherArm.SetEndPose(final)") ||
      !check(waist.SetEndPose(waist_pose, 3s), "Waist.SetEndPose(final)")) {
    return false;
  }
  std::this_thread::sleep_for(100ms);
  return true;
}

template <typename Arm, typename OtherArm, typename Waist, typename RobotPtr>
bool Run(Arm arm, OtherArm other_arm, Waist waist, RobotPtr& robot,
         const std::string& action, const std::string& mode,
         const std::string& side, bool assume_yes) {
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
  if (mode != "joint_pos" && mode != "end_pose") {
    std::cerr << "Valid modes: joint_pos, end_pose\n";
    return false;
  }

  if (!assume_yes) {
    std::cout << "About to run Quanta X2 " << side << " arm " << mode
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

  if (mode == "end_pose") {
    std::this_thread::sleep_for(1s);
    ::geometry_msgs::Pose reset;
    reset.mutable_position()->set_x(0.400);
    reset.mutable_position()->set_y(side == "left" ? 0.220 : -0.220);
    reset.mutable_position()->set_z(0.877);
    reset.mutable_orientation()->set_w(1.0);
    std::cout << "Starting end pose reset...\n";
    if (!MovePose(arm, reset, other_arm, waist)) return false;
    std::this_thread::sleep_for(2s);

    ::geometry_msgs::Pose target = reset;
    // Keep the same straight-line TOPP-RA path as Python, with a smaller
    // demonstration excursion for a safer standalone C++ sample.
    target.mutable_position()->set_z(0.980);
    std::cout << "Executing smooth end-pose trajectory...\n";
    if (!MovePose(arm, target, other_arm, waist)) return false;
  } else {
    const std::vector<double> reset =
        side == "left"
            ? std::vector<double>{1.57, -1.57, -1.57, -1.57, 0.0, 0.0, 0.0}
            : std::vector<double>{-1.57, -1.57, 1.57, -1.57, 0.0, 0.0, 0.0};
    const std::vector<double> target =
        side == "left"
            ? std::vector<double>{1.57, -1.57, -1.57, -1.37, 0.0, 0.0, 0.30}
            : std::vector<double>{-1.57, -1.57, 1.57, -1.37, 0.0, 0.0,
                                  0.30};
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
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string side = argc > 2 ? argv[2] : "left";
  const std::string action = argc > 3 ? argv[3] : "move";
  const std::string mode = argc > 4 ? argv[4] : "joint_pos";
  const bool assume_yes = argc > 5 && std::string(argv[5]) == "--yes";
  std::signal(SIGINT, signal_handler);

  auto created = Robot::Create(address, true);
  if (!check(created, "Robot.Create")) return 1;
  auto robot = std::move(created.Value());
  bool ok = side == "right"
                ? Run(robot->GetRightArmController(),
                      robot->GetLeftArmController(), robot->GetWaistController(),
                      robot, action, mode, side, assume_yes)
                : Run(robot->GetLeftArmController(),
                      robot->GetRightArmController(), robot->GetWaistController(),
                      robot, action, mode, side, assume_yes);
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
