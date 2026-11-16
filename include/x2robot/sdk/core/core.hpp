#pragma once

// C2 连接上下文：保存服务端地址、固定的 gRPC 传输以及地址是否指向本机。
// 不持有任何业务域通道，那些归 Robot（见 x2robot/sdk/robot.hpp），
// 否则每加一个业务域都要改这里。后续共享 gRPC channel、会话 id、
// 重连等连接级资源属于本类。

#include <string>
#include <utility>

#include "x2robot/sdk/types.hpp"

namespace x2robot::sdk::client {

class Core {
 public:
  Core(std::string address, TransportType transport, bool is_local)
      : address_(std::move(address)), transport_(transport), is_local_(is_local) {}

  const std::string& Address() const noexcept { return address_; }
  TransportType transport() const noexcept { return transport_; }
  bool IsLocal() const noexcept { return is_local_; }

 private:
  std::string address_;
  TransportType transport_;
  bool is_local_{false};
};

}  // namespace x2robot::sdk::client
