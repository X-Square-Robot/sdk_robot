// Counterpart of python/v1/desktop/master_arm_control.py.
#include <algorithm>
#include <array>
#include <atomic>
#include "interrupt.hpp"
#include <chrono>
#include <csignal>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include "x2robot/sdk/robot.hpp"
#include "sdk_mode_cleanup.hpp"
#include "master_arm_trajectory.hpp"
using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using namespace std::chrono_literals;
namespace {
std::atomic_bool stop{false};
constexpr std::array<double, 6> kMasterJointLower =
    {-2.7925, 0.0, -2.7925, -1.5708, -1.5708, -1.9199};
constexpr std::array<double, 6> kMasterJointUpper =
    {2.7925, 3.6652, 0.0, 1.5708, 1.5708, 1.9199};
constexpr double kJointFeedbackLimitTolerance = 1e-3;
// Conservative command-profile defaults for this example, not hardware ratings.
constexpr double kMinimumDuration = 4.0;
constexpr double kDefaultJointDelta = 0.262; // rad, first joint only
constexpr double kDefaultZDelta = 0.10; // m, end_pose mode only
constexpr double kMaximumZDelta = 0.10;
constexpr double kMaximumDuration = 120.0;
// Match python/v2/desktop/arm_control.py and the shared C++ slave-arm profile.
constexpr double kJointVelocity = 1.0;       // rad/s
constexpr double kJointAcceleration = 3.0;   // rad/s^2
constexpr double kLinearVelocity = 2.2;      // m/s
constexpr double kLinearAcceleration = 0.3;  // m/s^2
constexpr double kJointInterval = 0.002; // s, 500 Hz like the Python slave arm
constexpr double kPoseInterval = 0.005;  // s, 200 Hz like the Python slave arm
constexpr auto kCommandDeadline = 500ms;
namespace trajectory = x2robot::examples::master_arm_trajectory;

bool NormalizePose(::geometry_msgs::Pose* pose)
{
    const auto& p = pose->position();
    auto* q = pose->mutable_orientation();
    const double norm = std::hypot(std::hypot(q->x(), q->y()), std::hypot(q->z(), q->w()));
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()) ||
        !std::isfinite(norm) || norm < 1e-9) {
        std::cerr << "MasterArm.GetEndPose returned invalid position/orientation\n";
        return false;
    }
    q->set_x(q->x() / norm);
    q->set_y(q->y() / norm);
    q->set_z(q->z() / norm);
    q->set_w(q->w() / norm);
    return true;
}

double OrientationDot(const ::geometry_msgs::Quaternion& a,
                      const ::geometry_msgs::Quaternion& b)
{
    return a.x() * b.x() + a.y() * b.y() + a.z() * b.z() + a.w() * b.w();
}

double OrientationDistance(const ::geometry_msgs::Quaternion& a,
                           const ::geometry_msgs::Quaternion& b)
{
    return 2.0 * std::acos(std::clamp(std::abs(OrientationDot(a, b)), 0.0, 1.0));
}

double PositionDistance(const ::geometry_msgs::Pose& a, const ::geometry_msgs::Pose& b)
{
    return std::hypot(std::hypot(a.position().x() - b.position().x(),
                                 a.position().y() - b.position().y()),
                      a.position().z() - b.position().z());
}

// Shortest-path SLERP, as used by the Python slave-arm example.
::geometry_msgs::Quaternion InterpolateOrientation(
    const ::geometry_msgs::Quaternion& start,
    const ::geometry_msgs::Quaternion& target, double ratio)
{
    const double dot = OrientationDot(start, target);
    const double sign = dot < 0.0 ? -1.0 : 1.0;
    const double cosine = std::clamp(std::abs(dot), 0.0, 1.0);
    const double angle = std::acos(cosine);
    const double denominator = std::sin(angle);
    const bool coincident = cosine > 1.0 - 1e-12;
    const double a = coincident ? 1.0 - ratio : std::sin((1.0 - ratio) * angle) / denominator;
    const double b = sign * (coincident ? ratio : std::sin(ratio * angle) / denominator);
    ::geometry_msgs::Quaternion result;
    result.set_x(a * start.x() + b * target.x());
    result.set_y(a * start.y() + b * target.y());
    result.set_z(a * start.z() + b * target.z());
    result.set_w(a * start.w() + b * target.w());
    const double norm = std::hypot(std::hypot(result.x(), result.y()),
                                   std::hypot(result.z(), result.w()));
    result.set_x(result.x() / norm);
    result.set_y(result.y() / norm);
    result.set_z(result.z() / norm);
    result.set_w(result.w() / norm);
    return result;
}

::geometry_msgs::Pose InterpolatePose(const ::geometry_msgs::Pose& start,
                                      const ::geometry_msgs::Pose& target,
                                      double ratio, double orientation_ratio)
{
    ::geometry_msgs::Pose command = start;
    command.mutable_position()->set_x(start.position().x() + (target.position().x() - start.position().x()) * ratio);
    command.mutable_position()->set_y(start.position().y() + (target.position().y() - start.position().y()) * ratio);
    command.mutable_position()->set_z(start.position().z() + (target.position().z() - start.position().z()) * ratio);
    *command.mutable_orientation() = InterpolateOrientation(start.orientation(), target.orientation(), orientation_ratio);
    return command;
}

template <typename Send>
bool SendTrajectory(const trajectory::Profile& profile, double sample_interval,
                    const std::string& name, Send send)
{
    const double seconds = profile.duration();
    if (!std::isfinite(seconds) || seconds <= 0.0 || seconds > kMaximumDuration) {
        std::cerr << name << ": invalid trajectory or duration exceeds " << kMaximumDuration << " s\n";
        return false;
    }
    const int steps = static_cast<int>(std::ceil(seconds / sample_interval));
    const auto interval = std::chrono::duration<double>(seconds / steps);
    std::cout << name << ": TOPPRA trajectory duration=" << seconds << " s, steps=" << steps << '\n';
    // Include the measured start pose. RPC delays lengthen the trajectory; do
    // not skip ahead in progress or burst commands to catch up to a clock.
    for (int step = 0; step <= steps; ++step) {
        if (x2robot::examples::ExitRequestedBySignal()) return false;
        const auto sent_at = std::chrono::steady_clock::now();
        const double time_fraction = static_cast<double>(step) / steps;
        if (!send(profile.Progress(time_fraction), time_fraction)) return false;
        if (step != steps) std::this_thread::sleep_until(sent_at + interval);
    }
    return !x2robot::examples::ExitRequestedBySignal();
}

template <typename CheckTarget>
bool WaitForTarget(const std::string& name, CheckTarget check_target)
{
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!x2robot::examples::ExitRequestedBySignal() && std::chrono::steady_clock::now() < deadline) {
        const int state = check_target(); // -1: invalid feedback, 0: waiting, 1: reached
        if (state < 0) return false;
        if (state > 0) {
            std::cout << name << ": target reached\n";
            return true;
        }
        std::this_thread::sleep_for(50ms);
    }
    std::cerr << name << ": target not reached before timeout or interruption\n";
    return false;
}

bool NormalizeJointFeedback(const ::sensor_msgs::JointState& state,
                            std::vector<double>* joints)
{
    if (state.position_size() != static_cast<int>(kMasterJointLower.size())) {
        std::cerr << "MasterArm.GetJointStates returned " << state.position_size()
                  << " joints; expected " << kMasterJointLower.size() << '\n';
        return false;
    }
    joints->clear();
    joints->reserve(kMasterJointLower.size());
    for (std::size_t i = 0; i < kMasterJointLower.size(); ++i) {
        const double value = state.position(static_cast<int>(i));
        if (!std::isfinite(value) ||
            value < kMasterJointLower[i] - kJointFeedbackLimitTolerance ||
            value > kMasterJointUpper[i] + kJointFeedbackLimitTolerance) {
            std::cerr << "MasterArm joint " << i + 1 << " feedback " << value
                      << " is outside [" << kMasterJointLower[i] << ", "
                      << kMasterJointUpper[i] << "]\n";
            return false;
        }
        joints->push_back(std::clamp(value, kMasterJointLower[i], kMasterJointUpper[i]));
    }
    return true;
}
void sig(int signal)
{
    stop.store(true);
    x2robot::examples::RequestExitOnCtrlC(signal);
}
}  // namespace
template <typename T>
bool check(const SdkResult<T>& r, const char* n)
{
    if (r.Ok()) return true;
    std::cerr << n << " failed: " << r.ErrorValue().error_message << '\n';
    return false;
}
// ManipulatorControlMode 的取值见 protocol/proto/v2/controller/robot_control.proto：
// 0=末端位姿、1=关节位置、2=重力补偿。日志里同时打印枚举名，避免只看到数字。
const char* ControlModeName(int mode)
{
    switch (static_cast<::xr::sdk::v2::ManipulatorControlMode>(mode))
    {
    case ::xr::sdk::v2::MANIPULATOR_END_POSE: return "MANIPULATOR_END_POSE";
    case ::xr::sdk::v2::MANIPULATOR_JOINT_POSITIONS: return "MANIPULATOR_JOINT_POSITIONS";
    case ::xr::sdk::v2::MANIPULATOR_GRAVITY_COMPENSATION: return "MANIPULATOR_GRAVITY_COMPENSATION";
    }
    return "UNKNOWN";
}
template <typename Arm>
bool Read(Arm& arm, const std::string& name)
{
    bool ok = true;
    auto mode = arm.GetControlMode(3s);
    ok = check(mode, (name + ".GetControlMode").c_str()) && ok;
    if (mode.Ok())
        std::cout << name << " control_mode=" << mode.Value().mode() << " ("
                  << ControlModeName(mode.Value().mode()) << ")\n";
    auto joints = arm.GetJointStates(3s);
    ok = check(joints, (name + ".GetJointStates").c_str()) && ok;
    if (joints.Ok()) std::cout << name << " joints=" << joints.Value().ShortDebugString() << '\n';
    auto pose = arm.GetEndPose(3s);
    ok = check(pose, (name + ".GetEndPose").c_str()) && ok;
    if (pose.Ok()) std::cout << name << " end_pose=" << pose.Value().ShortDebugString() << '\n';
    auto grip = arm.GetGripperPosition(3s);
    ok = check(grip, (name + ".GetGripperPosition").c_str()) && ok;
    if (grip.Ok()) std::cout << name << " gripper=" << grip.Value().position() << '\n';
    return ok;
}
template <typename Arm>
bool SetMode(Arm& arm, const std::string& mode, const std::string& name)
{
    ::xr::sdk::v2::ManipulatorControlModeParam param;
    if (mode == "end_pose")
    {
        param.set_mode(::xr::sdk::v2::MANIPULATOR_END_POSE);
    }
    else if (mode == "gravity")
    {
        param.set_mode(::xr::sdk::v2::MANIPULATOR_GRAVITY_COMPENSATION);
    }
    else if (mode == "joint_pos")
    {
        param.set_mode(::xr::sdk::v2::MANIPULATOR_JOINT_POSITIONS);
    }
    else
    {
        std::cerr << "Unknown control mode: " << mode << '\n';
        return false;
    }
    return check(arm.SetControlMode(param, 3s), (name + ".SetControlMode").c_str());
}
template <typename Arm>
bool Move(Arm& arm, const std::string& mode, const std::string& name, bool zero,
          double minimum_seconds, double joint_delta, double z_delta)
{
    if (mode != "joint_pos" && mode != "end_pose") return false;
    if (x2robot::examples::ExitRequestedBySignal() || !SetMode(arm, mode, name)) return false;
    if (mode == "end_pose")
    {
        auto current = arm.GetEndPose(3s);
        if (!check(current, (name + ".GetEndPose").c_str())) return false;
        ::geometry_msgs::Pose start = current.Value().pose();
        if (!NormalizePose(&start)) return false;
        ::geometry_msgs::Pose target = start;
        if (zero) {
            target.Clear();
            target.mutable_orientation()->set_w(1.0);
        } else {
            target.mutable_position()->set_z(target.position().z() + z_delta);
        }
        const double distance = PositionDistance(start, target);
        // Match the Python reference's fallback for a pure orientation move.
        const trajectory::Profile profile(distance > 1e-6 ? distance : 1.0,
            kLinearVelocity, kLinearAcceleration, minimum_seconds);
        if (!SendTrajectory(profile, kPoseInterval, name, [&](double ratio, double time_fraction) {
            return check(arm.SetEndPose(InterpolatePose(start, target, ratio, time_fraction), kCommandDeadline),
                         (name + ".SetEndPose").c_str());
        })) return false;
        return WaitForTarget(name, [&]() {
            auto state = arm.GetEndPose(kCommandDeadline);
            if (!check(state, (name + ".GetEndPose").c_str())) return -1;
            auto feedback = state.Value().pose();
            if (!NormalizePose(&feedback)) return -1;
            return PositionDistance(feedback, target) <= 0.002 &&
                OrientationDistance(feedback.orientation(), target.orientation()) <= 0.02 ? 1 : 0;
        });
    }
    auto current = arm.GetJointStates(3s);
    if (!check(current, (name + ".GetJointStates").c_str())) return false;
    std::vector<double> start;
    if (!NormalizeJointFeedback(current.Value(), &start)) return false;
    std::vector<double> target;
    double distance = 0.0;
    for (std::size_t i = 0; i < start.size(); ++i) {
        const double value = zero ? 0.0 : start[i] + (i == 0 ? joint_delta : 0.0);
        if (value < kMasterJointLower[i] || value > kMasterJointUpper[i]) {
            std::cerr << name << ".SetJointPositions target joint " << i + 1
                      << " is outside [" << kMasterJointLower[i] << ", "
                      << kMasterJointUpper[i] << "]\n";
            return false;
        }
        target.push_back(value);
        distance = std::max(distance, std::abs(value - start[i]));
    }
    const trajectory::Profile profile(distance, kJointVelocity,
                                       kJointAcceleration, minimum_seconds);
    if (!SendTrajectory(profile, kJointInterval, name, [&](double ratio, double) {
        ::xr::sdk::v2::JointPositions command;
        for (std::size_t i = 0; i < start.size(); ++i) {
            command.add_positions(start[i] + (target[i] - start[i]) * ratio);
        }
        return check(arm.SetJointPositions(command, kCommandDeadline), (name + ".SetJointPositions").c_str());
    })) return false;
    return WaitForTarget(name, [&]() {
        auto state = arm.GetJointStates(kCommandDeadline);
        if (!check(state, (name + ".GetJointStates").c_str())) return -1;
        std::vector<double> feedback;
        if (!NormalizeJointFeedback(state.Value(), &feedback)) return -1;
        for (std::size_t i = 0; i < target.size(); ++i) {
            if (std::abs(feedback[i] - target[i]) > 0.01) return 0;
        }
        return 1;
    });
}
template <typename Arm>
bool Stream(Arm& arm, const std::string& which, const std::string& name)
{
    if (which == "joint_states")
    {
        auto o = arm.GetJointStatesStream(5s);
        if (!check(o, (name + ".GetJointStatesStream").c_str())) return false;
        auto s = std::move(o.Value());
        while (!stop)
        {
            auto v = s->Read(2s);
            if (!v.Ok()) break;
            std::cout << name << " joint_states: " << v.Value().ShortDebugString() << '\n';
        }
        auto cancelled = s->Cancel();
        return check(cancelled, (name + ".JointStatesStream.Cancel").c_str());
    }
    if (which == "end_pose")
    {
        auto o = arm.GetEndPoseStream(5s);
        if (!check(o, (name + ".GetEndPoseStream").c_str())) return false;
        auto s = std::move(o.Value());
        while (!stop)
        {
            auto v = s->Read(2s);
            if (!v.Ok()) break;
            std::cout << name << " end_pose: " << v.Value().ShortDebugString() << '\n';
        }
        auto cancelled = s->Cancel();
        return check(cancelled, (name + ".EndPoseStream.Cancel").c_str());
    }
    if (which == "gripper_state")
    {
        auto o = arm.GetGripperStateStream(5s);
        if (!check(o, (name + ".GetGripperStateStream").c_str())) return false;
        auto s = std::move(o.Value());
        while (!stop)
        {
            auto v = s->Read(2s);
            if (!v.Ok()) break;
            std::cout << name << " gripper_state: " << v.Value().ShortDebugString() << '\n';
        }
        auto cancelled = s->Cancel();
        return check(cancelled, (name + ".GripperStateStream.Cancel").c_str());
    }
    auto o = arm.GetGripperJointStatesStream(5s);
    if (!check(o, (name + ".GetGripperJointStatesStream").c_str())) return false;
    auto s = std::move(o.Value());
    while (!stop)
    {
        auto v = s->Read(2s);
        if (!v.Ok()) break;
        std::cout << name << " gripper_joint_states: " << v.Value().ShortDebugString() << '\n';
    }
    auto cancelled = s->Cancel();
    return check(cancelled, (name + ".GripperJointStatesStream.Cancel").c_str());
}
int main(int argc, char** argv)
{
    std::vector<std::string> positional;
    double minimum_seconds = kMinimumDuration;
    double joint_delta = kDefaultJointDelta;
    bool joint_delta_supplied = false;
    double z_delta = kDefaultZDelta;
    bool z_delta_supplied = false;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--help" || argument == "-h") {
            std::cout << "Usage: " << argv[0]
                      << " [server] [left|right|both] [read|snapshot|set_mode|move|zero|stream]"
                         " [joint_pos|end_pose|gravity] [stream_name] [--duration seconds]"
                         " [--joint-delta radians] [--z-delta meters]\n"
                         "move/zero: slave-arm TOPPRA profile; --duration is a minimum (default 4 s, max 120 s).\n"
                         "Large moves take longer to keep command speed/acceleration within example limits.\n"
                         "--joint-delta: move joint_pos only; signed J1 offset in [-0.262, 0.262] rad (default 0.262).\n"
                         "--z-delta: move end_pose only; signed Z offset in [-0.10, 0.10] m (default 0.10).\n"
                         "gravity is valid only for set_mode; both arms run sequentially.\n";
            return 0;
        }
        if (argument == "--duration") {
            if (++i == argc) { std::cerr << "--duration needs seconds\n"; return 2; }
            try {
                std::size_t used = 0;
                minimum_seconds = std::stod(argv[i], &used);
                if (used != std::string(argv[i]).size() || !std::isfinite(minimum_seconds) ||
                    minimum_seconds <= 0.0 || minimum_seconds > kMaximumDuration) throw std::invalid_argument("duration");
            } catch (const std::exception&) {
                std::cerr << "--duration must be finite and in (0, 120] seconds\n";
                return 2;
            }
        } else if (argument == "--joint-delta") {
            if (++i == argc) { std::cerr << "--joint-delta needs radians\n"; return 2; }
            try {
                std::size_t used = 0;
                joint_delta = std::stod(argv[i], &used);
                if (used != std::string(argv[i]).size() || !std::isfinite(joint_delta) ||
                    std::abs(joint_delta) > kDefaultJointDelta) throw std::invalid_argument("joint delta");
            } catch (const std::exception&) {
                std::cerr << "--joint-delta must be finite and in [-0.262, 0.262] radians\n";
                return 2;
            }
            joint_delta_supplied = true;
        } else if (argument == "--z-delta") {
            if (++i == argc) { std::cerr << "--z-delta needs meters\n"; return 2; }
            try {
                std::size_t used = 0;
                z_delta = std::stod(argv[i], &used);
                if (used != std::string(argv[i]).size() || !std::isfinite(z_delta) ||
                    std::abs(z_delta) > kMaximumZDelta) throw std::invalid_argument("Z delta");
            } catch (const std::exception&) {
                std::cerr << "--z-delta must be finite and in [-0.10, 0.10] meters\n";
                return 2;
            }
            z_delta_supplied = true;
        } else if (argument.rfind("--", 0) == 0) {
            std::cerr << "Unknown option: " << argument << '\n';
            return 2;
        } else positional.push_back(argument);
    }
    if (positional.size() > 5) { std::cerr << "Too many arguments\n"; return 2; }
    const std::string address = positional.size() > 0 ? positional[0] : "127.0.0.1:55051";
    const std::string side = positional.size() > 1 ? positional[1] : "left";
    const std::string action = positional.size() > 2 ? positional[2] : "read";
    const std::string mode = positional.size() > 3 ? positional[3] : "joint_pos";
    const std::string stream = positional.size() > 4 ? positional[4] : "joint_states";
    if ((joint_delta_supplied && (action != "move" || mode != "joint_pos")) ||
        (z_delta_supplied && (action != "move" || mode != "end_pose")) ||
        (side != "left" && side != "right" && side != "both") ||
        (action != "read" && action != "snapshot" && action != "set_mode" &&
         action != "move" && action != "zero" && action != "stream") ||
        ((action == "move" || action == "zero") && mode != "joint_pos" && mode != "end_pose") ||
        (action == "set_mode" && mode != "joint_pos" && mode != "end_pose" && mode != "gravity") ||
        (action == "stream" && stream != "joint_states" && stream != "end_pose" &&
         stream != "gripper_state" && stream != "gripper_joint_states")) {
        std::cerr << "Invalid side, action, mode or stream (see --help)\n";
        return 2;
    }
    std::signal(SIGINT, sig);
    std::signal(SIGTERM, sig);
    auto c = Robot::Create(address, true);
    if (!check(c, "Robot.Create")) return 1;
    auto robot = std::move(c.Value());
    bool ok = true;
    auto left = robot->GetMasterLeftArm();
    auto right = robot->GetMasterRightArm();
    // 主臂命令与手臂共用同一道服务端门控：没进 SDK 模式时
    // MasterArm.SetControlMode / SetJointPositions 会报 "Sdk mode is not
    // running"，所以控制前必须先进入 SDK 模式。
    if (action == "set_mode" || action == "move" || action == "zero")
    {
        ::xr::sdk::v2::RobotModeParam sdk_mode;
        sdk_mode.set_mode(::xr::sdk::v2::SDK);
        ok = check(x2robot::examples::NoteWorkMode(robot, sdk_mode, robot->GetSystem().SetWorkMode(sdk_mode, 3s)), "System.SetWorkMode") && ok;
        if (!ok)
        {
            check(robot->Close(), "Robot.Close");
            return 1;
        }
    }
    auto run = [&](auto& arm, const std::string& name) {
        if (action == "read" || action == "snapshot") return Read(arm, name);
        if (action == "set_mode") return SetMode(arm, mode, name);
        if (action == "move" || action == "zero") return Move(arm, mode, name, action == "zero", minimum_seconds, joint_delta, z_delta);
        if (action == "stream") return Stream(arm, stream, name);
        std::cerr << "Unknown action\n";
        return false;
    };
    if (side == "both")
    {
        ok = run(left, "MasterLeftArm");
        if (ok && !x2robot::examples::ExitRequestedBySignal()) ok = run(right, "MasterRightArm");
    }
    else if (side == "right")
        ok = run(right, "MasterRightArm");
    else
        ok = run(left, "MasterLeftArm");
    ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
    ok = check(robot->Close(), "Robot.Close") && ok;
    return x2robot::examples::ExitRequestedBySignal() ? 130 : (ok ? 0 : 1);
}
