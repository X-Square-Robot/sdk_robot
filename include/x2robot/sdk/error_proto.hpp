#pragma once

// Format a structured SDK error for the gRPC status details field. The public
// v2 contract carries the original integer error code in SdkResultHeader, so
// there is no legacy enum conversion here.

#include <string>

#include "x2robot/sdk/error_code.hpp"

namespace x2robot::sdk {

inline std::string ToProtoErrorMessage(const ErrorCode& e) {
  std::string out = e.error_name + "(" + std::to_string(e.error_code) + ")";
  if (!e.error_message.empty()) out += ": " + e.error_message;
  if (!e.suggestion.empty()) out += " | " + e.suggestion;
  return out;
}

}  // namespace x2robot::sdk
