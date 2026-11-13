// Counterpart of python/v1/quanta_x2/end_pose_replay_new.py.  The episode
// format is JSON; this standalone example intentionally uses the standard
// library only and replays every pose object it can find.
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include "x2robot/sdk/robot.hpp"
#include "sdk_mode_cleanup.hpp"
using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using namespace std::chrono_literals;
template <typename T>
bool check(const SdkResult<T>& r, const char* n)
{
    if (r.Ok()) return true;
    std::cerr << n << " failed: " << r.ErrorValue().error_message << '\n';
    return false;
}
struct PoseData
{
    double x{}, y{}, z{}, qx{}, qy{}, qz{}, qw{1};
};
std::vector<PoseData> Load(const std::string& path)
{
    std::ifstream in(path);
    if (!in) return {};
    const std::string text((std::istreambuf_iterator<char>(in)), {});
    std::regex p(
        "\\\"position\\\"\\s*:\\s*\\{[^}]*\\\"x\\\"\\s*:\\s*([-+0-9.eE]+)[^}]*\\\"y\\\"\\s*:\\s*([-+0-9.eE]+)[^}]*\\\"z\\\"\\s*:\\s*([-+0-9.eE]+)[^}]"
        "*\\}\\s*,\\s*\\\"orientation\\\"\\s*:\\s*\\{[^}]*\\\"x\\\"\\s*:\\s*([-+0-9.eE]+)[^}]*\\\"y\\\"\\s*:\\s*([-+0-9.eE]+)[^}]*\\\"z\\\"\\s*:\\s*("
        "[-+0-9.eE]+)[^}]*\\\"w\\\"\\s*:\\s*([-+0-9.eE]+)");
    std::vector<PoseData> out;
    for (std::sregex_iterator i(text.begin(), text.end(), p), end; i != end; ++i)
    {
        PoseData q;
        for (int n = 0; n < 7; ++n)
        {
            double* dst[] = {&q.x, &q.y, &q.z, &q.qx, &q.qy, &q.qz, &q.qw};
            *dst[n] = std::stod((*i)[n + 1]);
        }
        out.push_back(q);
    }
    return out;
}
int main(int argc, char** argv)
{
    const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051", episode = argc > 2 ? argv[2] : "episode.json",
                      side = argc > 3 ? argv[3] : "both";
    const double speed = argc > 4 ? std::stod(argv[4]) : 1.0;
    auto poses = Load(episode);
    if (poses.empty())
    {
        std::cerr << "No valid pose frames found in " << episode << '\n';
        return 1;
    }
    auto c = Robot::Create(address, true);
    if (!check(c, "Robot.Create")) return 1;
    auto robot = std::move(c.Value());
    ::xr::sdk::v2::RobotModeParam sdk;
    sdk.set_mode(::xr::sdk::v2::SDK);
    bool ok = check(x2robot::examples::NoteWorkMode(robot, sdk, robot->GetSystem().SetWorkMode(sdk, 3s)), "System.SetWorkMode");
    ::xr::sdk::v2::ManipulatorControlModeParam mode;
    mode.set_mode(::xr::sdk::v2::MANIPULATOR_END_POSE);
    ok = check(robot->GetRobotControl().SetManipulatorControlMode(mode, 3s), "RobotControl.SetManipulatorControlMode") && ok;
    auto left = robot->GetLeftArmController();
    auto right = robot->GetRightArmController();
    std::cout << "Replaying " << poses.size() << " end-pose frames at " << speed << "x\n";
    for (std::size_t i = 0; i < poses.size() && ok; ++i)
    {
        const auto& q = poses[i];
        ::geometry_msgs::Pose p;
        p.mutable_position()->set_x(q.x);
        p.mutable_position()->set_y(q.y);
        p.mutable_position()->set_z(q.z);
        p.mutable_orientation()->set_x(q.qx);
        p.mutable_orientation()->set_y(q.qy);
        p.mutable_orientation()->set_z(q.qz);
        p.mutable_orientation()->set_w(q.qw);
        if (side == "left" || side == "both") ok = check(left.SetEndPose(p, 1s), "LeftArm.SetEndPose") && ok;
        if (side == "right" || side == "both") ok = check(right.SetEndPose(p, 1s), "RightArm.SetEndPose") && ok;
        std::this_thread::sleep_for(std::chrono::duration<double>(0.033 / std::max(0.01, speed)));
        if (i % 10 == 0) std::cout << "Progress: " << i + 1 << "/" << poses.size() << '\r';
    }
    std::cout << '\n';
    ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
    ok = check(robot->Close(), "Robot.Close") && ok;
    return ok ? 0 : 1;
}
