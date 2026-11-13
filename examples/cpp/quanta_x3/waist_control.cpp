// Counterpart of python/v2/quanta_x3/waist_control.py: read-only by default.
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "interrupt.hpp"
#include "sdk_mode_cleanup.hpp"
#include "x2robot/sdk/robot.hpp"

namespace quanta_x3_waist {
using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using Vector = std::array<double, 3>;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

// Kept in sync with python/v2/config/quanta_x3/waist.json by the offline tests.
constexpr std::array<const char*, 3> kJointNames = {"lift_joint", "waist_pitch_joint", "waist_yaw_joint"};
constexpr Vector kJointLower = {0.0, -0.0523, -1.570796};
constexpr Vector kJointUpper = {0.371, 0.872665, 1.570796};
constexpr Vector kJointMargin = {0.002, 0.005, 0.005};
constexpr Vector kJointMaxDelta = {0.05, 0.2, 0.2};
constexpr Vector kJointVelocity = {0.02, 0.06, 0.06};
constexpr std::array<double, 2> kLinearSpeedRange = {0.001, 0.04};
constexpr std::array<double, 2> kAngularSpeedRange = {0.005, 0.12};
constexpr Vector kJointAcceleration = {0.02, 0.06, 0.06};
constexpr Vector kJointJerk = {0.08, 0.24, 0.24};
constexpr Vector kJointTrackingError = {0.01, 0.04, 0.04};
constexpr Vector kPoseLower = {-5.0, -5.0, -5.0};
constexpr Vector kPoseUpper = {5.0, 5.0, 5.0};
constexpr double kPoseMaxDelta = 0.01;
constexpr double kPoseZMaxDelta = 0.05;
constexpr double kPoseMaxAngle = 0.2;
constexpr double kPoseAngularVelocity = 0.06;
constexpr double kPoseAngularAcceleration = 0.06;
constexpr double kPoseAngularJerk = 0.24;
constexpr Vector kPoseJointMaxExcursion = {0.05, 0.15, 0.15};
constexpr double kPoseVelocity = 0.01;
constexpr double kPoseAcceleration = 0.01;
constexpr double kPoseJerk = 0.04;
constexpr double kInterval = 0.02;
constexpr double kMaxGap = 0.4;  // Whole-cycle budget, shared by feedback and all target writes.
constexpr auto kWriteDeadline = 300ms;
constexpr auto kFeedbackDeadline = 300ms;
constexpr auto kFeedbackStartWait = 2s;

template <typename T>
void Require(const SdkResult<T>& result, const char* operation) {
    if (!result.Ok())
        throw std::runtime_error(std::string(operation) + " failed: " + result.ErrorValue().error_message);
}

template <typename T>
T Value(SdkResult<T> result, const char* operation) {
    Require(result, operation);
    return std::move(result.Value());
}

void CheckInterrupted() {
    if (x2robot::examples::ExitRequestedBySignal()) throw std::runtime_error("interrupted");
}

class ControlCycle {
   public:
    void Begin() {
        CheckInterrupted();
        const auto now = Clock::now();
        if (started_ && std::chrono::duration<double>(now - *started_).count() >= kMaxGap)
            throw std::runtime_error("control cycle stalled between samples; stopping commands");
        started_ = now;
    }

    double Check() const {
        CheckInterrupted();
        const double remaining = kMaxGap - std::chrono::duration<double>(Clock::now() - *started_).count();
        if (remaining < 0.001) throw std::runtime_error("control cycle stalled; stopping commands");
        return remaining;
    }

    std::chrono::milliseconds WriteDeadline() const {
        return std::min(kWriteDeadline, std::chrono::milliseconds(static_cast<int64_t>(Check() * 1000)));
    }

    void BeforeSend() {
        // Variable feedback latency must not bunch two adjacent targets.
        if (last_dispatch_) std::this_thread::sleep_until(*last_dispatch_ + 20ms);
        Check();
        last_dispatch_ = Clock::now();
    }

    void Wait() const {
        Check();
        std::this_thread::sleep_until(*started_ + 20ms);
    }

   private:
    std::optional<Clock::time_point> started_, last_dispatch_;
};

void ValidateSpeeds(std::optional<double> linear, std::optional<double> angular) {
    const auto check = [](std::optional<double> value, const std::array<double, 2>& range, const char* name) {
        if (value && (!std::isfinite(*value) || *value < range[0] || *value > range[1]))
            throw std::invalid_argument(std::string(name) + " speed must be finite and within [" +
                                        std::to_string(range[0]) + ", " + std::to_string(range[1]) + "]");
    };
    check(linear, kLinearSpeedRange, "linear");
    check(angular, kAngularSpeedRange, "angular");
}

void Finite(const Vector& values) {
    for (double value : values) {
        if (!std::isfinite(value)) throw std::invalid_argument("expected three finite values");
    }
}

double Norm(const Vector& values) {
    return std::sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
}

Vector Add(const Vector& a, const Vector& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }

void ValidateJoints(const Vector& values, bool margin = false) {
    Finite(values);
    for (size_t i = 0; i < values.size(); ++i) {
        const double pad = margin ? kJointMargin[i] : 0.0;
        if (values[i] < kJointLower[i] + pad || values[i] > kJointUpper[i] - pad) {
            throw std::runtime_error(std::string(kJointNames[i]) + " outside waist joint limits");
        }
    }
}

template <typename Message>
void Fresh(const Message& message) {
    const auto& stamp = message.header().stamp();
    const double timestamp = stamp.sec() + stamp.nanosec() * 1e-9;
    const double now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    const double age = now - timestamp;
    if (timestamp <= 0 || age < -0.5 || age > 1.0)
        throw std::runtime_error("feedback missing/stale or robot clock unsynchronized");
}

Vector ReadJoints(Robot& robot, bool margin = false) {
    const auto state = Value(robot.GetWaistController().GetJointStates(kFeedbackDeadline), "Waist.GetJointStates");
    Fresh(state);
    if (state.name_size() != 3 || state.position_size() != 3)
        throw std::runtime_error("expected three named waist joints");
    Vector values{};
    for (size_t i = 0; i < values.size(); ++i) {
        const auto found = std::find(state.name().begin(), state.name().end(), kJointNames[i]);
        if (found == state.name().end()) throw std::runtime_error("unexpected waist joint names");
        values[i] = state.position(static_cast<int>(found - state.name().begin()));
    }
    ValidateJoints(values, margin);
    return values;
}

double QuaternionDot(const ::geometry_msgs::Quaternion& a, const ::geometry_msgs::Quaternion& b) {
    return a.x() * b.x() + a.y() * b.y() + a.z() * b.z() + a.w() * b.w();
}

void ValidateQuaternion(const ::geometry_msgs::Quaternion& q) {
    const double norm = std::sqrt(QuaternionDot(q, q));
    if (!std::isfinite(norm) || norm < 0.999 || norm > 1.001)
        throw std::runtime_error("feedback orientation is not a unit quaternion");
}

Vector ValidatePose(const ::geometry_msgs::Pose& pose) {
    const Vector p{pose.position().x(), pose.position().y(), pose.position().z()};
    Finite(p);
    ValidateQuaternion(pose.orientation());
    for (size_t i = 0; i < p.size(); ++i) {
        if (p[i] < kPoseLower[i] || p[i] > kPoseUpper[i]) throw std::runtime_error("waist pose outside command bounds");
    }
    return p;
}

::geometry_msgs::PoseStamped ReadPose(Robot& robot, bool wait_for_feedback = false) {
    CheckInterrupted();
    const auto deadline = Clock::now() + kFeedbackStartWait;
    auto result = robot.GetWaistController().GetEndPose(kFeedbackDeadline);
    while (!result.Ok() && wait_for_feedback && Clock::now() < deadline &&
           (result.ErrorValue().error_name == "UNAVAILABLE" || result.ErrorValue().error_name == "TIMEOUT")) {
        CheckInterrupted();
        std::this_thread::sleep_for(50ms);
        CheckInterrupted();
        result = robot.GetWaistController().GetEndPose(kFeedbackDeadline);
    }
    CheckInterrupted();
    auto sample = Value(std::move(result), "Waist.GetEndPose");
    Fresh(sample);
    ValidatePose(sample.pose());
    if (sample.header().frame_id().empty()) throw std::runtime_error("waist feedback has no reference frame");
    return sample;
}

::geometry_msgs::Pose PoseAt(const ::geometry_msgs::Pose& source, const Vector& position) {
    auto pose = source;
    pose.mutable_position()->set_x(position[0]);
    pose.mutable_position()->set_y(position[1]);
    pose.mutable_position()->set_z(position[2]);
    return pose;
}

struct Rotation {
    std::string axis;
    double angle;
};

::geometry_msgs::Pose PoseRotated(const ::geometry_msgs::Pose& source, const std::string& axis, double angle) {
    if ((axis != "pitch" && axis != "yaw") || !std::isfinite(angle))
        throw std::invalid_argument("rotation requires pitch or yaw and a finite angle in radians");
    // Fixed feedback-frame axis: q_target = q_delta * q_initial.
    auto pose = source;
    const auto& q = source.orientation();
    const double s = std::sin(angle / 2), c = std::cos(angle / 2);
    const double y = axis == "pitch" ? s : 0.0, z = axis == "yaw" ? s : 0.0;
    auto* out = pose.mutable_orientation();
    out->set_x(c * q.x() + y * q.z() - z * q.y());
    out->set_y(c * q.y() + y * q.w() + z * q.x());
    out->set_z(c * q.z() - y * q.x() + z * q.w());
    out->set_w(c * q.w() - y * q.y() - z * q.z());
    return pose;
}

::geometry_msgs::Pose ArmHold(x2robot::sdk::ArmControllerClient controller) {
    auto pose = Value(controller.GetEndPose(kFeedbackDeadline), "Arm.GetEndPose").pose();
    Finite({pose.position().x(), pose.position().y(), pose.position().z()});
    ValidateQuaternion(pose.orientation());
    return pose;
}

std::vector<Vector> Plan(const Vector& start, const Vector& delta, const Vector& velocity, const Vector& acceleration,
                         const Vector& jerk, bool cartesian = false) {
    Finite(start);
    Finite(delta);
    double duration = 2.0;
    for (size_t i = 0; i < delta.size(); ++i) {
        const double distance = cartesian ? Norm(delta) : std::abs(delta[i]);
        duration =
            std::max({duration, 1.875 * distance / velocity[i],
                      std::sqrt(10 / std::sqrt(3.0) * distance / acceleration[i]), std::cbrt(60 * distance / jerk[i])});
    }
    const size_t steps = static_cast<size_t>(std::ceil(duration / kInterval));
    std::vector<Vector> points;
    points.reserve(steps + 1);
    for (size_t i = 0; i <= steps; ++i) {
        const double u = static_cast<double>(i) / steps;
        const double scale = u * u * u * (10 + u * (-15 + 6 * u));
        points.push_back({start[0] + delta[0] * scale, start[1] + delta[1] * scale, start[2] + delta[2] * scale});
    }
    return points;
}

bool Within(const Vector& actual, const Vector& command, const Vector& tolerance) {
    for (size_t i = 0; i < actual.size(); ++i) {
        if (std::abs(actual[i] - command[i]) > tolerance[i]) return false;
    }
    return true;
}

void CheckTracking(const Vector& actual, const Vector& command, const Vector& tolerance) {
    if (!Within(actual, command, tolerance))
        throw std::runtime_error("feedback deviates from trajectory; stopping commands");
}

void ValidateDelta(const std::string& mode, const Vector& delta, const std::optional<Rotation>& rotation = {}) {
    Finite(delta);
    if (rotation) {
        if (mode != "end_pose" || Norm(delta) != 0)
            throw std::invalid_argument("rotation requires end_pose mode; do not mix translation or joint targets");
        if ((rotation->axis != "pitch" && rotation->axis != "yaw") || !std::isfinite(rotation->angle) ||
            std::abs(rotation->angle) == 0 || std::abs(rotation->angle) > kPoseMaxAngle)
            throw std::invalid_argument("pose rotation requires pitch or yaw and 0 < abs(angle) <= 0.2 rad");
        return;
    }
    if (std::all_of(delta.begin(), delta.end(), [](double value) { return value == 0; }))
        throw std::invalid_argument("move requires a nonzero relative displacement");
    if (mode == "joint_pos") {
        for (size_t i = 0; i < delta.size(); ++i) {
            if (std::abs(delta[i]) > kJointMaxDelta[i])
                throw std::invalid_argument("joint deltas exceed [0.05 m, 0.2 rad, 0.2 rad]");
        }
    } else {
        const double limit = delta[0] == 0 && delta[1] == 0 ? kPoseZMaxDelta : kPoseMaxDelta;
        if (mode != "end_pose" || Norm(delta) > limit)
            throw std::invalid_argument("Cartesian displacement exceeds limits: Z-only <= 0.05 m; otherwise <= 0.01 m");
    }
}

void SetMode(std::unique_ptr<Robot>& robot, bool cartesian) {
    ::xr::sdk::v2::RobotModeParam sdk;
    sdk.set_mode(::xr::sdk::v2::SDK);
    // Like Python, attempt IDLE even if SDK entry times out ambiguously.
    x2robot::examples::MarkSdkModeEntered(sdk);
    x2robot::examples::RegisterIdleRestoreOnCtrlC(robot);
    Require(robot->GetSystem().SetWorkMode(sdk, 5s), "System.SetWorkMode(SDK)");
    CheckInterrupted();
    ::xr::sdk::v2::ManipulatorControlModeParam control;
    control.set_mode(cartesian ? ::xr::sdk::v2::MANIPULATOR_END_POSE : ::xr::sdk::v2::MANIPULATOR_JOINT_POSITIONS);
    Require(robot->GetRobotControl().SetManipulatorControlMode(control, 5s), "SetManipulatorControlMode");
}

void Move(std::unique_ptr<Robot>& robot, const std::string& mode, const Vector& delta,
          const std::optional<Rotation>& rotation = {}, std::optional<double> linear_speed = {},
          std::optional<double> angular_speed = {}) {
    ValidateDelta(mode, delta, rotation);
    ValidateSpeeds(linear_speed, angular_speed);
    auto joint_velocity = kJointVelocity;
    if (linear_speed) joint_velocity[0] = *linear_speed;
    if (angular_speed) joint_velocity[1] = joint_velocity[2] = *angular_speed;
    const double pose_velocity = linear_speed.value_or(kPoseVelocity);
    const double pose_angular_velocity = angular_speed.value_or(kPoseAngularVelocity);
    ControlCycle cycle;
    auto pose_joint_excursion = kPoseJointMaxExcursion;
    if (rotation) {
        const size_t index = rotation->axis == "pitch" ? 1 : 2;
        pose_joint_excursion[index] = std::max(pose_joint_excursion[index], std::abs(rotation->angle) + kJointTrackingError[index]);
    }
    const bool cartesian = mode == "end_pose";
    auto start = ReadJoints(*robot, cartesian);
    std::this_thread::sleep_for(100ms);
    CheckInterrupted();
    CheckTracking(ReadJoints(*robot), start, {0.0005, 0.001, 0.001});
    if (!cartesian) ValidateJoints(Add(start, delta), true);
    SetMode(robot, cartesian);
    const auto initial_joints = start;
    auto observed_joints = ReadJoints(*robot, cartesian);
    if (!cartesian)
        CheckTracking(observed_joints, start, {0.0005, 0.001, 0.001});
    else if (!Within(observed_joints, initial_joints, pose_joint_excursion))
        throw std::runtime_error("waist joint excursion exceeds Cartesian demonstration limits");
    start = observed_joints;
    std::vector<Vector> points;
    std::function<void(const Vector&)> send;
    std::function<Vector(const Vector&)> monitor;
    std::function<::geometry_msgs::Pose(const Vector&)> pose_for_point;
    Vector tolerance;
    if (!cartesian) {
        ValidateJoints(Add(start, delta), true);
        points = Plan(start, delta, joint_velocity, kJointAcceleration, kJointJerk);
        for (const auto& point : points) ValidateJoints(point);
        send = [&](const Vector& point) {
            ValidateJoints(point);
            ::xr::sdk::v2::JointPositions command;
            for (double value : point) command.add_positions(value);
            CheckInterrupted();
            Require(robot->GetWaistController().SetJointPositions(command, cycle.WriteDeadline()), "Waist.SetJointPositions");
            cycle.Check();
        };
        monitor = [&](const Vector& previous) {
            auto actual = ReadJoints(*robot);
            CheckTracking(actual, previous, kJointTrackingError);
            return actual;
        };
        tolerance = {0.002, 0.005, 0.005};
    } else {
        const auto sample = ReadPose(*robot, true);
        const auto source = sample.pose();
        start = ValidatePose(source);
        if (!rotation) {
            points = Plan(start, delta, {pose_velocity, pose_velocity, pose_velocity},
                          {kPoseAcceleration, kPoseAcceleration, kPoseAcceleration},
                          {kPoseJerk, kPoseJerk, kPoseJerk}, true);
            pose_for_point = [source](const Vector& point) { return PoseAt(source, point); };
        } else {
            const size_t index = rotation->axis == "pitch" ? 1 : 2;
            Vector angles{};
            angles[index] = rotation->angle;
            points = Plan({}, angles, {pose_angular_velocity, pose_angular_velocity, pose_angular_velocity},
                          {kPoseAngularAcceleration, kPoseAngularAcceleration, kPoseAngularAcceleration},
                          {kPoseAngularJerk, kPoseAngularJerk, kPoseAngularJerk}, true);
            pose_for_point = [source, rotation, index](const Vector& point) {
                return PoseRotated(source, rotation->axis, point[index]);
            };
        }
        for (const auto& point : points) ValidatePose(pose_for_point(point));
        const auto left = ArmHold(robot->GetLeftArmController());
        const auto right = ArmHold(robot->GetRightArmController());
        send = [&, left, right](const Vector& point) {
            const auto pose = pose_for_point(point);
            ValidatePose(pose);
            CheckInterrupted();
            Require(robot->GetLeftArmController().SetEndPose(left, cycle.WriteDeadline()), "LeftArm.SetEndPose");
            CheckInterrupted();
            Require(robot->GetRightArmController().SetEndPose(right, cycle.WriteDeadline()), "RightArm.SetEndPose");
            CheckInterrupted();
            Require(robot->GetWaistController().SetEndPose(pose, cycle.WriteDeadline()), "Waist.SetEndPose");
            cycle.Check();
        };
        monitor = [&, sample](const Vector&) {
            observed_joints = ReadJoints(*robot, true);
            if (!Within(observed_joints, initial_joints, pose_joint_excursion))
                throw std::runtime_error("waist joint excursion exceeds Cartesian demonstration limits");
            const auto current = ReadPose(*robot);
            if (current.header().frame_id() != sample.header().frame_id())
                throw std::runtime_error("waist reference frame changed during motion");
            // WBC pose error is diagnostic; measured joint limits/excursion
            // still bound the physical motion of this demonstration.
            return ValidatePose(current.pose());
        };
    }
    std::cout << "Planned " << points.size() << " samples, at least " << (points.size() - 1) * kInterval << "s\n";
    std::cout << "Velocity limits: joints [m/s, rad/s, rad/s]=" << joint_velocity[0] << ", " << joint_velocity[1]
              << ", " << joint_velocity[2] << "; pose=" << pose_velocity << " m/s, " << pose_angular_velocity << " rad/s\n";
    if (rotation) std::cout << "Feedback-frame rotation: " << rotation->axis << ' ' << rotation->angle << " rad\n";
    auto previous = points.front();
    const auto trajectory_started = Clock::now();
    for (const auto& point : points) {
        cycle.Begin();
        monitor(previous);
        cycle.BeforeSend();
        send(point);
        previous = point;
        cycle.Wait();
    }
    std::cout << "Trajectory samples completed in " << std::chrono::duration<double>(Clock::now() - trajectory_started).count() << "s\n";
    if (cartesian) {
        const auto deadline = Clock::now() + 1s;
        Vector actual{};
        while (true) {
            cycle.Begin();
            actual = monitor(previous);
            cycle.Check();
            if (Clock::now() >= deadline) break;
            cycle.BeforeSend();
            send(previous);
            cycle.Wait();
        }
        const auto target_pose = pose_for_point(previous);
        const auto target = ValidatePose(target_pose);
        std::cout << "Cartesian command sequence completed; target xyz(m)=" << target[0] << ", " << target[1]
                  << ", " << target[2] << "; feedback xyz(m)=" << actual[0] << ", " << actual[1] << ", " << actual[2]
                  << "\nMeasured waist joint delta [m, rad, rad]: " << observed_joints[0] - initial_joints[0] << ", "
                  << observed_joints[1] - initial_joints[1] << ", " << observed_joints[2] - initial_joints[2]
                  << "\nNOTE: WBC balances multiple tasks; completion does not imply precise target arrival.\n";
        if (rotation) {
            const auto& q = target_pose.orientation();
            std::cout << "Target quaternion xyzw=" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << '\n';
        }
        if (Within(observed_joints, initial_joints, {0.0005, 0.001, 0.001}))
            std::cout << "WARNING: no net waist joint movement observed; command acceptance alone does not confirm motion.\n";
        return;
    }
    const auto deadline = Clock::now() + 2s;
    while (true) {
        cycle.Begin();
        const auto actual = monitor(previous);
        cycle.Check();
        if (Within(actual, previous, tolerance)) return;
        if (Clock::now() >= deadline) throw std::runtime_error("waist did not reach the trajectory endpoint");
        cycle.BeforeSend();
        send(previous);
        cycle.Wait();
    }
}

template <typename Stream>
void ReadStream(Stream stream, double seconds) {
    const auto start = Clock::now();
    try {
        while (seconds == 0 || std::chrono::duration<double>(Clock::now() - start).count() < seconds) {
            CheckInterrupted();
            auto value = Value(stream->Read(1s), "Waist stream read");
            std::cout << value.ShortDebugString() << '\n';
        }
    } catch (...) {
        (void)stream->Cancel();
        throw;
    }
    Require(stream->Cancel(), "Waist stream cancel");
}

struct Options {
    std::string server = "127.0.0.1:55051", action = "verify", mode = "joint_pos";
    std::optional<Vector> joint_delta, pose_delta;
    std::optional<Rotation> pose_rotation;
    std::optional<double> linear_speed, angular_speed;
    double seconds = 0;
    bool yes = false, help = false;
};

double Number(const std::string& text) {
    size_t end = 0;
    const double value = std::stod(text, &end);
    if (end != text.size() || !std::isfinite(value)) throw std::invalid_argument("expected a finite number: " + text);
    return value;
}

Options Parse(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("missing value for " + arg);
            return argv[i];
        };
        if (arg == "--help" || arg == "-h")
            options.help = true;
        else if (arg == "--server")
            options.server = next();
        else if (arg == "--action")
            options.action = next();
        else if (arg == "--mode")
            options.mode = next();
        else if (arg == "--seconds")
            options.seconds = Number(next());
        else if (arg == "--linear-speed")
            options.linear_speed = Number(next());
        else if (arg == "--angular-speed")
            options.angular_speed = Number(next());
        else if (arg == "--yes")
            options.yes = true;
        else if (arg == "--pose-rotation")
            options.pose_rotation = Rotation{next(), Number(next())};
        else if (arg == "--joint-delta" || arg == "--pose-delta") {
            const Vector delta{Number(next()), Number(next()), Number(next())};
            (arg == "--joint-delta" ? options.joint_delta : options.pose_delta) = delta;
        } else
            throw std::invalid_argument("unknown argument: " + arg);
    }
    if (options.help) return options;
    if (options.action != "verify" && options.action != "move" && options.action != "stream")
        throw std::invalid_argument("action must be verify, move or stream");
    if (options.mode != "joint_pos" && options.mode != "end_pose")
        throw std::invalid_argument("mode must be joint_pos or end_pose");
    if (options.seconds < 0) throw std::invalid_argument("seconds must be non-negative");
    ValidateSpeeds(options.linear_speed, options.angular_speed);
    if (options.action != "move" && (options.linear_speed || options.angular_speed))
        throw std::invalid_argument("speed options require --action move");
    if (options.action == "move") {
        const auto& delta = options.mode == "joint_pos" ? options.joint_delta : options.pose_delta;
        const int count = options.joint_delta.has_value() + options.pose_delta.has_value() + options.pose_rotation.has_value();
        if (count != 1 || (!delta && !options.pose_rotation))
            throw std::invalid_argument("move requires exactly one delta matching the selected mode");
        ValidateDelta(options.mode, delta.value_or(Vector{}), options.pose_rotation);
    }
    return options;
}

bool ConfirmMotion(bool moving = true) {
    std::cout << "Check clear space and emergency stop. Press Enter for "
              << (moving ? "smooth waist motion" : "SDK end-pose feedback (mode entry may move the robot)")
              << ", Ctrl+C to cancel: "
              << std::flush;
    // No connection or exit watchdog exists yet. Poll so Ctrl+C can cancel
    // confirmation without waiting for another line of terminal input.
    while (!x2robot::examples::ExitRequestedBySignal()) {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const int ready = ::poll(&input, 1, 100);
        if (ready < 0) return false;
        if (ready == 0) continue;
        if (!(input.revents & (POLLIN | POLLHUP))) return false;
        char ch;
        if (::read(STDIN_FILENO, &ch, 1) != 1) return false;
        if (ch == '\n') return !x2robot::examples::ExitRequestedBySignal();
    }
    return false;
}
}  // namespace quanta_x3_waist

int main(int argc, char** argv) {
    using namespace quanta_x3_waist;
    Options options;
    try {
        options = Parse(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << error.what() << "; use --help\n";
        return 2;
    }
    if (options.help) {
        std::cout << "quanta_x3_waist_control [--server ADDRESS] [--action verify|move|stream]\n"
                     "  [--mode joint_pos|end_pose] [--joint-delta LIFT_M PITCH_RAD YAW_RAD]\n"
                     "  [--pose-delta X_M Y_M Z_M] [--seconds SECONDS] [--yes]\n"
                     "  [--pose-rotation pitch|yaw ANGLE_RAD] (end_pose only; fixed feedback-frame axis)\n"
                     "  [--linear-speed M_S] [0.001, 0.04]; default joint=0.02, pose=0.01\n"
                     "  [--angular-speed RAD_S] [0.005, 0.12]; default=0.06 for pitch/yaw\n"
                     "Default joint verify is read-only. End-pose verify/stream enters SDK end-pose mode\n"
                     "and restores IDLE on exit. Motion requires an explicit bounded relative delta.\n";
        return 0;
    }
    if ((options.action == "move" || options.mode == "end_pose") && !options.yes &&
        !ConfirmMotion(options.action == "move")) return 1;
    std::unique_ptr<Robot> robot;
    int status = 0;
    try {
        CheckInterrupted();
        robot = Value(Robot::Create(options.server, true), "Robot.Create");
        if (robot->ModelName() != "quanta_x3")
            throw std::runtime_error("expected Quanta X3, connected to " + robot->ModelName());
        if (options.action == "move") {
            Move(robot, options.mode, (options.mode == "joint_pos" ? options.joint_delta : options.pose_delta).value_or(Vector{}),
                 options.pose_rotation, options.linear_speed, options.angular_speed);
        } else if (options.action == "stream") {
            if (options.mode == "end_pose") {
                ReadJoints(*robot);
                SetMode(robot, true);
                ReadPose(*robot, true);
            }
            auto waist = robot->GetWaistController();
            if (options.mode == "end_pose")
                ReadStream(Value(waist.GetEndPoseStream(1s), "Waist.GetEndPoseStream"), options.seconds);
            else
                ReadStream(Value(waist.GetJointStatesStream(1s), "Waist.GetJointStatesStream"), options.seconds);
        } else {
            const auto joints = ReadJoints(*robot);
            std::cout << "joint_pos: " << joints[0] << ", " << joints[1] << ", " << joints[2] << '\n';
            if (options.mode == "end_pose") {
                SetMode(robot, true);
                std::cout << "end_pose: " << ReadPose(*robot, true).ShortDebugString() << '\n';
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        status = 1;
    }
    if (robot) {
        if (!x2robot::examples::RestoreIdleModeOnExit(robot)) status = 1;
        try {
            Require(robot->Close(), "Robot.Close");
        } catch (const std::exception& error) {
            std::cerr << "FAIL: " << error.what() << '\n';
            status = 1;
        }
    }
    if (status == 0) std::cout << "PASS: Quanta X3 waist control\n";
    return status;
}
