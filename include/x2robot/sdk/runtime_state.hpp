#pragma once

#include <optional>
#include <string>

#include "x2robot/sdk/error_code.hpp"

namespace x2robot::sdk {

enum class RuntimeState
{
    kUnknown,
    kInit,
    kReady,
    kRunning,
    kBlocked,
    kError,
    kDisabled,
    kUnavailable
};

struct ModuleStatus
{
    std::string module;
    RuntimeState state{RuntimeState::kUnknown};
    double timestamp{0};
    std::string robot_id;
    std::string detail;
    std::optional<ErrorCode> last_error;
};

inline const char* RuntimeStateToString(RuntimeState s)
{
    switch (s)
    {
        case RuntimeState::kUnknown:
            return "UNKNOWN";
        case RuntimeState::kInit:
            return "INIT";
        case RuntimeState::kReady:
            return "READY";
        case RuntimeState::kRunning:
            return "RUNNING";
        case RuntimeState::kBlocked:
            return "BLOCKED";
        case RuntimeState::kError:
            return "ERROR";
        case RuntimeState::kDisabled:
            return "DISABLED";
        case RuntimeState::kUnavailable:
            return "UNAVAILABLE";
    }
    return "UNKNOWN";
}

}  // namespace x2robot::sdk
