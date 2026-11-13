#pragma once

#include "data_collection/replay.hpp"
#include "options.hpp"

namespace sdk_examples::v2 {

using StartLimits = std::map<std::string, double>;
inline StartLimits LoadStartLimits(const std::filesystem::path& path)
{
    StartLimits limits{{"joint_rad", .1}, {"position_m", .03}, {"orientation_rad", .1}, {"lift_m", .02},
                       {"gripper", .1}, {"chassis_position_m", .05}, {"chassis_yaw_rad", .1}, {"read_timeout_s", 3}};
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot read start-check config: " + path.string());
    std::set<std::string> seen;
    bool section = false;
    std::string line;
    while (std::getline(input, line))
    {
        line = sdk_recording::Trim(line);
        if (line.empty() || line.front() == '#' || line.front() == ';') continue;
        if (line == "[start_check]" && !section) { section = true; continue; }
        const auto equal = line.find('=');
        if (!section || equal == std::string::npos) throw std::runtime_error("expected [start_check] and key=value");
        const auto key = sdk_recording::Trim(line.substr(0, equal));
        const auto text = sdk_recording::Trim(line.substr(equal + 1));
        if (!limits.count(key) || !seen.insert(key).second) throw std::runtime_error("unknown/duplicate start-check key: " + key);
        size_t parsed = 0;
        const double value = std::stod(text, &parsed);
        if (parsed != text.size() || !std::isfinite(value) || value <= 0)
            throw std::runtime_error("start-check limits must be finite and positive");
        limits.at(key) = value;
    }
    if (input.bad() || seen.size() != limits.size()) throw std::runtime_error("incomplete start-check configuration");
    if (limits.at("read_timeout_s") < .001 || limits.at("read_timeout_s") > 60)
        throw std::runtime_error("read_timeout_s must be between 0.001 and 60");
    return limits;
}

template <typename T>
inline T ReadStart(x2robot::sdk::SdkResult<T> result, const std::string& operation)
{
    if (!result.Ok()) throw std::runtime_error(operation + ": " + result.ErrorValue().error_message);
    return std::move(result.Value());
}

inline void FiniteStart(const std::vector<double>& values)
{
    if (values.empty() || !std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); }))
        throw std::runtime_error("expected nonempty finite values");
}

inline double RotationError(const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b)
{
    const std::vector<double> left{a.x(), a.y(), a.z(), a.w()}, right{b.x(), b.y(), b.z(), b.w()};
    FiniteStart(left); FiniteStart(right);
    const double aa = std::hypot(std::hypot(a.x(), a.y()), std::hypot(a.z(), a.w()));
    const double bb = std::hypot(std::hypot(b.x(), b.y()), std::hypot(b.z(), b.w()));
    if (!std::isfinite(aa+bb) || std::abs(aa-1) > .01 || std::abs(bb-1) > .01)
        throw std::runtime_error("orientation quaternion must be finite and approximately unit length");
    double dot = 0;
    for (size_t i = 0; i < 4; ++i) dot += (left[i]/aa)*(right[i]/bb);
    return 2 * std::acos(std::min(1.0, std::abs(dot)));
}

inline void CheckStartState(x2robot::sdk::Robot& robot, const ReplayEpisode& episode, bool joint_mode, const StartLimits& limits)
{
    std::map<std::string, const ReplayRecord*> first;
    for (const auto& record : episode.records)
        if (ReplayRecordApplies(record.stream, joint_mode)) first.emplace(record.stream, &record);
    std::vector<std::string> errors;
    const x2robot::sdk::Deadline deadline{static_cast<int64_t>(limits.at("read_timeout_s") * 1000)};
    const auto compare = [&](const std::string& name, const std::string& metric, double error, double limit) {
        const bool passed = std::isfinite(error) && error <= limit + std::max(1e-9, limit*1e-6);
        std::cout << "Start check: component=" << name << " metric=" << metric << " error=" << error
                  << " limit=" << limit << " " << (passed ? "OK" : "BLOCKED") << std::endl;
        if (!passed) errors.push_back(name + " " + metric + " exceeds " + std::to_string(limit));
    };
    const auto vector = [&](const std::string& name, const std::vector<double>& current,
                            const std::vector<double>& target, double limit) {
        FiniteStart(current); FiniteStart(target);
        if (current.size() != target.size()) throw std::runtime_error("current/recorded position count mismatch");
        double error = 0;
        for (size_t i = 0; i < current.size(); ++i) error = std::max(error, std::abs(current[i]-target[i]));
        compare(name, "max_position_error", error, limit);
    };
    const auto pose = [&](const std::string& name, const geometry_msgs::Pose& current, const geometry_msgs::Pose& target) {
        const auto& c = current.position(); const auto& t = target.position();
        FiniteStart({c.x(), c.y(), c.z(), t.x(), t.y(), t.z()});
        compare(name, "translation_m", std::hypot(c.x()-t.x(), c.y()-t.y(), c.z()-t.z()), limits.at("position_m"));
        compare(name, "rotation_rad", RotationError(current.orientation(), target.orientation()), limits.at("orientation_rad"));
    };
    for (const auto& [name, record] : first)
    {
        try
        {
            if (name == "left_arm_end_pose" || name == "right_arm_end_pose" || name == "waist_end_pose")
            {
                geometry_msgs::PoseStamped target;
                if (!target.ParseFromString(record->payload)) throw std::runtime_error("invalid pose record");
                auto current = name == "waist_end_pose" ? ReadStart(robot.GetWaistController().GetEndPose(deadline), name) :
                    (name == "left_arm_end_pose" ? ReadStart(robot.GetLeftArmController().GetEndPose(deadline), name) :
                                                  ReadStart(robot.GetRightArmController().GetEndPose(deadline), name));
                if (!target.header().frame_id().empty() && !current.header().frame_id().empty() &&
                    target.header().frame_id() != current.header().frame_id())
                    throw std::runtime_error("recorded/current end-pose frames differ");
                pose(name, current.pose(), target.pose());
            }
            else if (name == "odometry")
            {
                nav_msgs::Odometry target;
                if (!target.ParseFromString(record->payload)) throw std::runtime_error("invalid odometry record");
                const auto current = ReadStart(robot.GetChassisController().GetOdometry(deadline), name);
                const auto& c = current.pose().pose(); const auto& t = target.pose().pose();
                FiniteStart({c.position().x(), c.position().y(), t.position().x(), t.position().y()});
                RotationError(c.orientation(), t.orientation());
                compare(name, "translation_m", std::hypot(c.position().x()-t.position().x(), c.position().y()-t.position().y()),
                        limits.at("chassis_position_m"));
                compare(name, "yaw_rad", std::abs(std::remainder(QuaternionYaw(c.orientation())-QuaternionYaw(t.orientation()),
                        2*std::acos(-1.0))), limits.at("chassis_yaw_rad"));
            }
            else if (name == "left_gripper_position" || name == "right_gripper_position")
            {
                ::xr::sdk::v2::GripperPosition target;
                if (!target.ParseFromString(record->payload)) throw std::runtime_error("invalid gripper position");
                const auto current = name == "left_gripper_position" ? ReadStart(robot.GetLeftGripperController().GetPosition(deadline), name) :
                    ReadStart(robot.GetRightGripperController().GetPosition(deadline), name);
                vector(name, {current.position()}, {target.position()}, limits.at("gripper"));
            }
            else
            {
                sensor_msgs::JointState target;
                if (!target.ParseFromString(record->payload) || !target.position_size()) throw std::runtime_error("invalid/empty joint record");
                const std::vector<double> positions(target.position().begin(), target.position().end());
                if (name == "head_joint_states")
                {
                    const auto current = ReadStart(robot.GetHeadController().GetPose(deadline), name);
                    vector(name, {current.pitch(), current.yaw()}, positions, limits.at("joint_rad"));
                }
                else if (name == "lift_joint_states")
                {
                    const auto current = ReadStart(robot.GetLiftController().GetLiftPosition(deadline), name);
                    vector(name, {current.position()}, positions, limits.at("lift_m"));
                }
                else if (name == "left_gripper_joint_states" || name == "right_gripper_joint_states")
                {
                    const auto current = name == "left_gripper_joint_states" ? ReadStart(robot.GetLeftGripperController().GetPosition(deadline), name) :
                        ReadStart(robot.GetRightGripperController().GetPosition(deadline), name);
                    FiniteStart(positions);
                    vector(name, {current.position()}, {positions[0]}, limits.at("gripper"));
                }
                else
                {
                    const auto current = name == "waist_joint_states" ? ReadStart(robot.GetWaistController().GetJointStates(deadline), name) :
                        (name == "left_arm_joint_states" ? ReadStart(robot.GetLeftArmController().GetJointStates(deadline), name) :
                                                          ReadStart(robot.GetRightArmController().GetJointStates(deadline), name));
                    for (const auto* state : {&current, static_cast<const sensor_msgs::JointState*>(&target)})
                        if (state->name_size() && (state->name_size() != state->position_size() ||
                            std::set<std::string>(state->name().begin(), state->name().end()).size() != static_cast<size_t>(state->name_size())))
                            throw std::runtime_error("joint names must be unique and match the position count");
                    if (current.name_size() && target.name_size() &&
                        (current.name_size() != target.name_size() ||
                         !std::equal(current.name().begin(), current.name().end(), target.name().begin())))
                        throw std::runtime_error("joint names/order differ from the SDK command order");
                    vector(name, {current.position().begin(), current.position().end()}, positions, limits.at("joint_rad"));
                }
            }
        }
        catch (const std::exception& error)
        {
            errors.push_back(name + ": " + error.what());
            std::cerr << "Start check BLOCKED: " << errors.back() << std::endl;
        }
    }
    if (!errors.empty()) throw std::runtime_error("Replay start check failed; no motion commands sent: " + errors.front());
    std::cout << "Start check passed: " << first.size() << " recorded streams; unrecorded streams skipped" << std::endl;
}
}  // namespace sdk_examples::v2
