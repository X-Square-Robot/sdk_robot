// Counterpart of python/v1/robot_control.py.
#include <charconv>
#include <chrono>
#include <iostream>
#include <string>

#include "sdk_mode_cleanup.hpp"
#include "x2robot/sdk/robot.hpp"

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;

template <typename T>
bool check(const SdkResult<T>& result, const char* operation)
{
    if (result.Ok()) return true;
    const auto& e = result.ErrorValue();
    std::cerr << operation << " failed [" << e.error_name << "]: " << e.error_message << '\n';
    return false;
}

int main(int argc, char** argv)
{
    constexpr const char* usage =
        "Usage: robot_control [host:port] [homing [--timeout SECONDS]|stop|recover]\n"
        "Homing timeout defaults to 60 seconds; SECONDS must be a positive integer.\n";
    if (argc == 2 && std::string(argv[1]) == "--help")
    {
        std::cout << usage;
        return 0;
    }
    const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
    const std::string action = argc > 2 ? argv[2] : "homing";
    // 回零可能超过普通 RPC 的 3 秒预算，需覆盖服务端 30 秒等待及模式切换开销。
    int homing_timeout_seconds = 60;
    if (argc > 3)
    {
        bool valid = action == "homing" && argc == 5 && std::string(argv[3]) == "--timeout";
        if (valid)
        {
            const std::string value = argv[4];
            const auto parsed = std::from_chars(value.data(), value.data() + value.size(), homing_timeout_seconds);
            valid = parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size() && homing_timeout_seconds > 0;
        }
        if (!valid)
        {
            std::cerr << usage;
            return 2;
        }
    }
    auto connected = Robot::Create(address, true);
    if (!check(connected, "Robot.Create")) return 1;
    auto robot = std::move(connected.Value());
    bool ok = true;
    if (action == "stop")
    {
        std::cout << "Calling emergency_stop...\n";
        auto result = robot->GetRobotControl().EmergencyStop(std::chrono::seconds{3});
        ok = check(result, "RobotControl.EmergencyStop") && ok;
        std::cout << "emergency_stop result: " << (result.Ok() ? "true" : "false") << '\n';
    }
    else if (action == "recover")
    {
        std::cout << "Calling recover_emergency_stop...\n";
        auto result = robot->GetRobotControl().RecoverEmergencyStop(std::chrono::seconds{3});
        ok = check(result, "RobotControl.RecoverEmergencyStop") && ok;
        std::cout << "recover_emergency_stop result: " << (result.Ok() ? "true" : "false") << '\n';
    }
    else if (action == "homing")
    {
        ::xr::sdk::v2::RobotModeParam mode;
        mode.set_mode(::xr::sdk::v2::SDK);
        ok = check(x2robot::examples::NoteWorkMode(robot, mode,
                                                   robot->GetSystem().SetWorkMode(mode, std::chrono::seconds{3})),
                   "System.SetWorkMode") &&
             ok;
        std::cout << "Calling homing...\n";
        auto result = robot->GetRobotControl().Homing(std::chrono::seconds{homing_timeout_seconds});
        ok = check(result, "RobotControl.Homing") && ok;
        std::cout << "homing result: " << (result.Ok() ? "true" : "false") << '\n';
    }
    else
    {
        std::cerr << "Valid actions: stop, recover, homing\n";
        ok = false;
    }
    ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
    ok = check(robot->Close(), "Robot.Close") && ok;
    return ok ? 0 : 1;
}
