#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace x2robot::sdk {

enum class ErrorLevel
{
    kInfo,
    kWarning,
    kError,
    kFatal
};

struct DetailError
{
    std::string detail_error_name;
    std::string detail_error_message;
    std::string detail_code;
    std::string module;
};

// 标准错误：6 位数字码 + 富字段（设计文档 §4.1.1）
struct ErrorCode
{
    int32_t error_code{0};
    std::string error_name;
    std::string error_message;
    std::string module;
    ErrorLevel level{ErrorLevel::kError};
    bool recoverable{false};
    bool retryable{false};
    std::string suggestion;
    double timestamp{0};
    std::string robot_id;
    std::string request_id;
    std::string source;
    std::vector<DetailError> detail_errors;

    bool ok() const
    {
        return error_code == 0;
    }
};

// 常用 6 位错误码（范围|模块|具体）
namespace errc {
constexpr int32_t kOk = 0;
constexpr int32_t kInvalidArgument = 100201;
constexpr int32_t kUnauthenticated = 100401;
constexpr int32_t kCancelled = 100499;
constexpr int32_t kUnavailable = 100503;
constexpr int32_t kTimeout = 100504;
constexpr int32_t kUnimplemented = 100501;
constexpr int32_t kAlreadyExists = 100202;
constexpr int32_t kJointLimit = 201103;
constexpr int32_t kSafetyViolation = 201101;
constexpr int32_t kControlMode = 201102;
constexpr int32_t kInternal = 100500;
}  // namespace errc

inline const char* LevelToString(ErrorLevel l)
{
    switch (l)
    {
        case ErrorLevel::kInfo:
            return "INFO";
        case ErrorLevel::kWarning:
            return "WARNING";
        case ErrorLevel::kError:
            return "ERROR";
        case ErrorLevel::kFatal:
            return "FATAL";
    }
    return "ERROR";
}

struct ErrorMeta
{
    const char* name;
    const char* module;
    ErrorLevel level;
    bool recoverable;
    bool retryable;
    const char* suggestion;
};

inline const ErrorMeta* LookupErrorMeta(int32_t code)
{
    static const std::unordered_map<int32_t, ErrorMeta> kTable = {
        {errc::kOk, {"SUCCESS", "system", ErrorLevel::kInfo, true, false, ""}},
        {errc::kInvalidArgument, {"INVALID_ARGUMENT", "system", ErrorLevel::kError, true, false, "检查请求参数"}},
        {errc::kUnauthenticated, {"UNAUTHENTICATED", "session", ErrorLevel::kError, true, true, "重新鉴权或使用本机会话"}},
        {errc::kCancelled, {"CANCELLED", "transport", ErrorLevel::kWarning, true, true, "根据需要重新发起请求"}},
        {errc::kUnavailable, {"UNAVAILABLE", "system", ErrorLevel::kError, true, true, "稍后重试或检查服务状态"}},
        {errc::kTimeout, {"TIMEOUT", "transport", ErrorLevel::kError, true, true, "增大 deadline 后重试"}},
        {errc::kUnimplemented, {"UNIMPLEMENTED", "system", ErrorLevel::kError, false, false, "当前机型不支持该能力"}},
        {errc::kAlreadyExists, {"ALREADY_EXISTS", "system", ErrorLevel::kError, false, false, "该资源已经打开，无需重复打开"}},
        {errc::kJointLimit, {"JOINT_LIMIT_EXCEEDED", "arm", ErrorLevel::kError, true, false, "将位姿调整到限位内"}},
        {errc::kSafetyViolation, {"SAFETY_VIOLATION", "robot_control", ErrorLevel::kError, true, false, "解除硬件安全条件后重试"}},
        {errc::kControlMode, {"CONTROL_MODE_ERROR", "arm", ErrorLevel::kError, true, true, "先切换到目标控制模式"}},
        {errc::kInternal, {"INTERNAL", "system", ErrorLevel::kFatal, false, true, "查看服务端日志"}},
    };
    auto it = kTable.find(code);
    return it == kTable.end() ? nullptr : &it->second;
}

inline ErrorCode MakeError(int32_t error_code, std::string message = {})
{
    ErrorCode e;
    e.error_code = error_code;
    e.error_message = std::move(message);
    if (const auto* meta = LookupErrorMeta(error_code))
    {
        e.error_name = meta->name;
        e.module = meta->module;
        e.level = meta->level;
        e.recoverable = meta->recoverable;
        e.retryable = meta->retryable;
        e.suggestion = meta->suggestion;
    }
    else
    {
        e.error_name = "UNKNOWN";
        e.module = "system";
        e.level = ErrorLevel::kError;
    }
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    e.timestamp = std::chrono::duration<double>(now).count();
    e.source = "sdk";
    return e;
}

inline ErrorCode MakeClientError(int32_t error_code, std::string message = {})
{
    auto error = MakeError(error_code, std::move(message));
    error.source = "sdk_client";
    return error;
}

inline ErrorCode WithErrorContext(ErrorCode error, std::string request_id, std::string source = {})
{
    error.request_id = std::move(request_id);
    if (!source.empty()) error.source = std::move(source);
    if (error.timestamp == 0.0)
    {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        error.timestamp = std::chrono::duration<double>(now).count();
    }
    return error;
}

}  // namespace x2robot::sdk
