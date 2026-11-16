#pragma once

#include <string>
#include <type_traits>
#include <utility>

#include "v2/common/result.pb.h"
#include "x2robot/sdk/result.hpp"

namespace x2robot::sdk {

namespace result_proto_detail {

// Most v2 responses carry their payload in a message-valued `data` field.
// RobotStatus deliberately follows the v1 wire contract and carries a direct
// `string json` field, so keep the common result conversion generic while
// recognizing that one payload shape without coupling sdk_common to a
// service-specific generated header.
template <typename Response, typename = void>
struct HasJsonStringField : std::false_type {};

template <typename Response>
struct HasJsonStringField<
    Response,
    std::void_t<decltype(std::declval<Response*>()->set_json(
                    std::declval<const std::string&>())),
                decltype(std::declval<const Response&>().json())>>
    : std::true_type {};

}  // namespace result_proto_detail

inline ErrorLevel ErrorLevelFromString(const std::string& value) {
  if (value == "INFO") return ErrorLevel::kInfo;
  if (value == "WARNING") return ErrorLevel::kWarning;
  if (value == "FATAL") return ErrorLevel::kFatal;
  return ErrorLevel::kError;
}

inline void ToProtoError(const ErrorCode& error, ::xr::sdk::v2::SdkError* out) {
  out->set_error_code(error.error_code);
  out->set_error_name(error.error_name);
  out->set_error_message(error.error_message);
  out->set_module(error.module);
  out->set_level(LevelToString(error.level));
  out->set_recoverable(error.recoverable);
  out->set_retryable(error.retryable);
  out->set_suggestion(error.suggestion);
  out->set_timestamp(error.timestamp);
  out->set_robot_id(error.robot_id);
  out->set_request_id(error.request_id);
  out->set_source(error.source);
  for (const auto& detail : error.detail_errors) {
    auto* proto_detail = out->add_detail_errors();
    proto_detail->set_detail_error_name(detail.detail_error_name);
    proto_detail->set_detail_error_message(detail.detail_error_message);
    proto_detail->set_detail_code(detail.detail_code);
    proto_detail->set_module(detail.module);
  }
}

inline ErrorCode FromProtoError(const ::xr::sdk::v2::SdkError& error) {
  ErrorCode out;
  out.error_code = error.error_code();
  out.error_name = error.error_name();
  out.error_message = error.error_message();
  out.module = error.module();
  out.level = ErrorLevelFromString(error.level());
  out.recoverable = error.recoverable();
  out.retryable = error.retryable();
  out.suggestion = error.suggestion();
  out.timestamp = error.timestamp();
  out.robot_id = error.robot_id();
  out.request_id = error.request_id();
  out.source = error.source();
  out.detail_errors.reserve(static_cast<size_t>(error.detail_errors_size()));
  for (const auto& detail : error.detail_errors()) {
    out.detail_errors.push_back({detail.detail_error_name(), detail.detail_error_message(),
                                 detail.detail_code(), detail.module()});
  }
  return out;
}

inline void ToProtoFailureMetadata(ErrorCode error, const std::string& request_id,
                                   ::xr::sdk::v2::SdkResultHeader* out,
                                   const char* source = "sdk_server") {
  out->set_is_success(false);
  out->set_request_id(request_id);
  error = WithErrorContext(std::move(error), request_id, source);
  ToProtoError(error, out->mutable_error());
}

template <typename Response>
void ToProtoFailure(ErrorCode error, const std::string& request_id, Response* out,
                    const char* source = "sdk_server") {
  ToProtoFailureMetadata(std::move(error), request_id, out->mutable_header(), source);
}

template <typename Response, typename T>
void ToProtoResult(const SdkResult<T>& result, const std::string& request_id,
                   Response* out, const char* source = "sdk_server") {
  auto* metadata = out->mutable_header();
  metadata->set_request_id(request_id.empty() ? result.request_id : request_id);
  if (result.is_success && result.data) {
    metadata->set_is_success(true);
    if constexpr (std::is_same_v<std::decay_t<T>, std::string> &&
                  result_proto_detail::HasJsonStringField<Response>::value) {
      out->set_json(*result.data);
    } else {
      out->mutable_data()->CopyFrom(*result.data);
    }
    return;
  }
  metadata->set_is_success(false);
  auto error = result.error.value_or(MakeError(errc::kInternal, "SdkResult 缺少错误信息"));
  error = WithErrorContext(std::move(error), metadata->request_id(), source);
  ToProtoError(error, metadata->mutable_error());
}

inline void ToProtoResult(const SdkResult<void>& result, const std::string& request_id,
                          ::xr::sdk::v2::EmptyResult* out,
                          const char* source = "sdk_server") {
  auto* metadata = out->mutable_header();
  metadata->set_request_id(request_id.empty() ? result.request_id : request_id);
  if (result.is_success) {
    metadata->set_is_success(true);
    out->mutable_data();
    return;
  }
  metadata->set_is_success(false);
  auto error = result.error.value_or(MakeError(errc::kInternal, "SdkResult 缺少错误信息"));
  error = WithErrorContext(std::move(error), metadata->request_id(), source);
  ToProtoError(error, metadata->mutable_error());
}

template <typename T, typename Response>
SdkResult<T> FromProtoResult(const Response& response) {
  if (!response.has_header()) {
    return SdkResult<T>::Err(
        MakeClientError(errc::kInternal, "服务端响应缺少 header 字段"));
  }
  const auto& metadata = response.header();
  if (!metadata.is_success()) {
    auto error = metadata.has_error()
                     ? FromProtoError(metadata.error())
                     : MakeClientError(errc::kInternal, "服务端失败响应缺少 error 字段");
    return SdkResult<T>::Err(std::move(error), metadata.request_id());
  }
  if constexpr (std::is_same_v<std::decay_t<T>, std::string> &&
                result_proto_detail::HasJsonStringField<Response>::value) {
    return SdkResult<T>::Success(response.json(), metadata.request_id());
  } else {
    if (!response.has_data()) {
      return SdkResult<T>::Err(
          MakeClientError(errc::kInternal, "服务端成功响应缺少 data 字段"),
          metadata.request_id());
    }
    return SdkResult<T>::Success(response.data(), metadata.request_id());
  }
}

inline SdkResult<void> FromProtoResult(const ::xr::sdk::v2::EmptyResult& response) {
  if (!response.has_header()) {
    return SdkResult<void>::Err(
        MakeClientError(errc::kInternal, "服务端响应缺少 header 字段"));
  }
  const auto& metadata = response.header();
  if (!metadata.is_success()) {
    auto error = metadata.has_error()
                     ? FromProtoError(metadata.error())
                     : MakeClientError(errc::kInternal, "服务端失败响应缺少 error 字段");
    return SdkResult<void>::Err(std::move(error), metadata.request_id());
  }
  if (!response.has_data()) {
    return SdkResult<void>::Err(
        MakeClientError(errc::kInternal, "服务端成功响应缺少 data 字段"),
        metadata.request_id());
  }
  return SdkResult<void>::Success(metadata.request_id());
}

}  // namespace x2robot::sdk
