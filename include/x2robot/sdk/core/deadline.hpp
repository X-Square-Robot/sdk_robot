#pragma once

#include <optional>

#include "x2robot/sdk/error_code.hpp"
#include "x2robot/sdk/types.hpp"

namespace x2robot::sdk::client {

// Public wait budgets must be positive. Internal stream lifetimes may still
// use BeginCall's zero sentinel; validate the caller's budget before opening.
inline std::optional<ErrorCode> ValidateDeadline(Deadline deadline)
{
    if (deadline.count() > 0) return std::nullopt;
    return MakeClientError(errc::kInvalidArgument, "deadline must be positive");
}

}  // namespace x2robot::sdk::client
