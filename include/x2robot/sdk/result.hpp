#pragma once

#include <optional>
#include <string>
#include <utility>

#include "x2robot/sdk/error_code.hpp"

namespace x2robot::sdk {

template <typename T>
struct [[nodiscard]] SdkResult {
  bool is_success{false};
  std::optional<T> data;
  std::optional<ErrorCode> error;
  std::string request_id;

  SdkResult() = default;
  SdkResult(T value, std::string id = {})
      : is_success(true), data(std::move(value)), request_id(std::move(id)) {}

  static SdkResult Success(T value, std::string id = {}) {
    return SdkResult(std::move(value), std::move(id));
  }
  static SdkResult Err(ErrorCode value, std::string id = {}) {
    SdkResult out;
    out.request_id = std::move(id);
    out.error = WithErrorContext(std::move(value), out.request_id);
    return out;
  }

  bool Ok() const noexcept { return is_success; }
  explicit operator bool() const noexcept { return Ok(); }
  const T& Value() const { return data.value(); }
  T& Value() { return data.value(); }
  ErrorCode ErrorValue() const { return error.value_or(ErrorCode{}); }
};

template <>
struct [[nodiscard]] SdkResult<void> {
  bool is_success{false};
  std::optional<ErrorCode> error;
  std::string request_id;

  SdkResult() : is_success(true) {}

  static SdkResult Success(std::string id = {}) {
    SdkResult out;
    out.request_id = std::move(id);
    return out;
  }
  static SdkResult Err(ErrorCode value, std::string id = {}) {
    SdkResult out;
    out.is_success = false;
    out.request_id = std::move(id);
    out.error = WithErrorContext(std::move(value), out.request_id);
    return out;
  }

  bool Ok() const noexcept { return is_success; }
  explicit operator bool() const noexcept { return Ok(); }
  ErrorCode ErrorValue() const { return error.value_or(ErrorCode{}); }
};

}  // namespace x2robot::sdk
