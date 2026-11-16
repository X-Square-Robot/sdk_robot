// Counterpart of python/v1/quanta_x1/lift_control.py.
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
    std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051", action = argc > 2 ? argv[2] : "move", direction = argc > 3 ? argv[3] : "down";
    double distance = argc > 4 ? std::stod(argv[4]) : 0.05;
    std::signal(SIGINT, sig);
    auto c = Robot::Create(address, true);
    if (!check(c, "Robot.Create")) return 1;
    auto robot = std::move(c.Value());
    auto lift = robot->GetLiftController();
    bool ok = true;
    if (action == "move")
    {
        ::xr::sdk::v2::RobotModeParam sdk;
        sdk.set_mode(::xr::sdk::v2::SDK);
        ok = check(x2robot::examples::NoteWorkMode(robot, sdk, robot->GetSystem().SetWorkMode(sdk, 3s)), "System.SetWorkMode");
        ::xr::sdk::v2::ManipulatorControlModeParam m;
        m.set_mode(::xr::sdk::v2::MANIPULATOR_JOINT_POSITIONS);
        ok = check(robot->GetRobotControl().SetManipulatorControlMode(m, 3s), "RobotControl.SetManipulatorControlMode") && ok;
        auto cur = lift.GetLiftPosition(3s);
        ok = check(cur, "Lift.GetLiftPosition") && ok;
        if (cur.Ok())
        {
            std::cout << "current_position: " << cur.Value().position() << '\n';
            ::xr::sdk::v2::LiftPosition target;
            target.set_position(cur.Value().position() + (direction == "up" ? distance : -distance));
            ok = check(lift.SetLiftPosition(target, 3s), "Lift.SetLiftPosition") && ok;
            std::this_thread::sleep_for(500ms);
            cur = lift.GetLiftPosition(3s);
            ok = check(cur, "Lift.GetLiftPosition(after)") && ok;
            if (cur.Ok()) std::cout << "current_position: " << cur.Value().position() << '\n';
        }
    }
    else if (action == "stream")
    {
        std::cout << "Starting lift joint states streaming (Ctrl+C to stop)...\n";
        auto o = lift.GetJointStatesStream(5s);
        ok = check(o, "Lift.GetJointStatesStream");
        if (o.Ok())
        {
            auto s = std::move(o.Value());
            while (!stop)
            {
                auto v = s->Read(2s);
                if (!check(v, "Lift stream read")) { ok = false; break; }
                std::cout << "joint_state: " << v.Value().ShortDebugString() << '\n';
            }
            ok = check(s->Cancel(), "Lift.JointStatesStream.Cancel") && ok;
        }
    }
    else
    {
        std::cerr << "Valid actions: move, stream\n";
        ok = false;
    }
    ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
    ok = check(robot->Close(), "Robot.Close") && ok;
    return ok ? 0 : 1;
}
