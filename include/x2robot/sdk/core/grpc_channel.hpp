#pragma once

// Robot endpoints are direct gRPC targets. gRPC C++ automatically imports
// HTTP(S)_PROXY from the process environment, but its NO_PROXY handling does
// not consistently understand private-network CIDR entries such as
// 192.168.0.0/16. A workstation can therefore send a robot connection to a
// local proxy and report a misleading connect timeout.

#include <grpcpp/channel.h>

namespace x2robot::sdk::client {

inline void ConfigureDirectChannel(grpc::ChannelArguments* arguments)
{
    // Keep the literal here instead of depending on gRPC's internal channel
    // argument header; this is part of the public ChannelArguments API.
    arguments->SetInt("grpc.enable_http_proxy", 0);
    // Allow more receive lookahead for large camera messages, following the
    // v1 client's tuning. Keep the setting bounded at 1 MiB per stream;
    // actual throughput still depends on the source, network and decoder.
    arguments->SetInt("grpc.http2.lookahead_bytes", 1024 * 1024);
}

}  // namespace x2robot::sdk::client
