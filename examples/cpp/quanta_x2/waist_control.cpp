// Counterpart of python/v1/quanta_x2/waist_control.py.
#include <atomic>
#include "interrupt.hpp"
#include "sdk_mode_cleanup.hpp"
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "x2robot/sdk/robot.hpp"
using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using namespace std::chrono_literals;
namespace {
std::atomic_bool stop{false};
void sig(int signal)
{
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
int main(int argc, char** argv)
{
    std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051", action = argc > 2 ? argv[2] : "move", mode = argc > 3 ? argv[3] : "joint_pos";
    std::signal(SIGINT, sig);
    auto c = Robot::Create(address, true);
    if (!check(c, "Robot.Create")) return 1;
    auto robot = std::move(c.Value());
    auto waist = robot->GetWaistController();
    bool ok = true;
    ::xr::sdk::v2::RobotModeParam sdk;
    sdk.set_mode(::xr::sdk::v2::SDK);
    if (action == "move")
    {
        if (mode == "end_pose")
        {
            std::cerr << "end_pose mode is not supported for waist control\n";
            ok = false;
        }
        else
        {
            ok = check(x2robot::examples::NoteWorkMode(robot, sdk, robot->GetSystem().SetWorkMode(sdk, 3s)), "System.SetWorkMode");
            ::xr::sdk::v2::ManipulatorControlModeParam m;
            m.set_mode(::xr::sdk::v2::MANIPULATOR_JOINT_POSITIONS);
            ok = check(robot->GetRobotControl().SetManipulatorControlMode(m, 3s), "RobotControl.SetManipulatorControlMode") && ok;
            auto cur = waist.GetJointStates(3s);
            ok = check(cur, "Waist.GetJointStates") && ok;
            ::xr::sdk::v2::JointPositions target;
            for (int i = 0; i < (cur.Ok() ? cur.Value().position_size() : 4); ++i) target.add_positions(i == 3 ? 0.3 : 0);
            if (cur.Ok())
                for (int i = 1; i <= 20; ++i)
                {
                    ::xr::sdk::v2::JointPositions q;
                    for (int j = 0; j < target.positions_size(); ++j)
                        q.add_positions(cur.Value().position(j) * (1 - i / 20.0) + target.positions(j) * (i / 20.0));
                    ok = check(waist.SetJointPositions(q, 1s), "Waist.SetJointPositions") && ok;
                    std::this_thread::sleep_for(20ms);
                }
            std::cout << "Movement Finished.\n";
        }
    }
    else if (action == "stream")
    {
        if (mode == "end_pose")
        {
            auto o = waist.GetEndPoseStream(5s);
            ok = check(o, "Waist.GetEndPoseStream") && ok;
            if (o.Ok())
            {
                auto s = std::move(o.Value());
                while (!stop)
                {
                    auto v = s->Read(2s);
                    if (!check(v, "Waist stream read")) { ok = false; break; }
                    std::cout << "end_pose: " << v.Value().ShortDebugString() << '\n';
                }
                ok = check(s->Cancel(), "Waist.EndPoseStream.Cancel") && ok;
            }
        }
        else
        {
            auto o = waist.GetJointStatesStream(5s);
            ok = check(o, "Waist.GetJointStatesStream");
            if (o.Ok())
            {
                auto s = std::move(o.Value());
                while (!stop)
                {
                    auto v = s->Read(2s);
                    if (!check(v, "Waist stream read")) { ok = false; break; }
                    std::cout << "joint_state: " << v.Value().ShortDebugString() << '\n';
                }
                ok = check(s->Cancel(), "Waist.JointStatesStream.Cancel") && ok;
            }
        }
    }
    else
    {
        std::cerr << "Valid actions: move, stream; modes: joint_pos, end_pose\n";
        ok = false;
    }
    ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
    ok = check(robot->Close(), "Robot.Close") && ok;
    return ok ? 0 : 1;
}
