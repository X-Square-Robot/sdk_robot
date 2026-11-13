#pragma once

// gRPC-only 分支仍保留地址解析与本机判定，供 Robot::IsLocal() 使用。

#include <string>
#include <string_view>

#include "x2robot/sdk/types.hpp"

namespace x2robot::sdk::client {

struct ConnectOptions {
  // 为保持已有调用方的源码兼容而保留；gRPC-only 分支中没有其他传输可选。
  bool force_grpc{false};
  bool control_authorized{false};
};

bool IsLocalHost(std::string_view host);
TransportType DecideTransport(std::string_view address, const ConnectOptions& opts = {});
bool IsLocalAddress(std::string_view address);

// 解析 "host:port"
bool ParseAddress(std::string_view address, std::string* host, uint16_t* port);

}  // namespace x2robot::sdk::client
