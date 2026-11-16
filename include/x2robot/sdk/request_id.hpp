#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

namespace x2robot::sdk {

inline std::string NewRequestId() {
  static std::atomic<uint64_t> sequence{0};
  const auto now = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  return "sdk-" + std::to_string(now) + "-" + std::to_string(++sequence);
}

}  // namespace x2robot::sdk
