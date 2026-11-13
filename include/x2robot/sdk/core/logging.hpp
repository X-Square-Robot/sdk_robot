#pragma once

#include <string_view>
#include <grpcpp/support/status.h>
#include "x2robot/sdk/result.hpp"

namespace x2robot::sdk::client {

// Initialized once before any client worker starts. Both C++ and Python use
// the server's logger backend; no request/response payloads are logged.
void InitClientLogging();
void LogClientInfo(std::string_view event, std::string_view detail = {});
void LogRpcStart(std::string_view method, std::string_view request_id, long deadline_ms);
void LogRpcEnd(std::string_view method, std::string_view request_id, long elapsed_ms, bool cancelled);
void LogRpcError(std::string_view method, std::string_view request_id, const ErrorCode& error);
void LogRpcStatus(std::string_view method, std::string_view request_id,
                  const grpc::Status& status, bool cancelled = false);
// Generic Python RPCs return wire bytes. Inspect only the shared v2 header,
// without decoding/copying large image, audio or point-cloud payloads.
void LogRpcResponse(std::string_view method, std::string_view request_id, std::string_view payload);

template <typename T>
SdkResult<T> LogRpcResult(std::string_view method, SdkResult<T> result) {
    if (!result.Ok()) LogRpcError(method, result.request_id, result.ErrorValue());
    return result;
}

}  // namespace x2robot::sdk::client
