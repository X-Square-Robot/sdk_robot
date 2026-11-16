#pragma once

#include <vector>

#include "v2/common/benchmark.grpc.pb.h"
#include "x2robot/sdk/service/grpc_service.hpp"

namespace x2robot::sdk {

class Benchmark final : public client::detail::GrpcServiceClient<::xr::sdk::v2::Benchmark>
{
    using Service = ::xr::sdk::v2::Benchmark;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit Benchmark(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    SdkResult<ResultStreamHandle<::xr::sdk::v2::ros::Message>> SubscribeLimitedBag(const ::xr::sdk::v2::ros::SubscribeParams& request,
                                                                               Deadline first_deadline = Deadline{3000}) const
    {
        return ServerStream<::xr::sdk::v2::ros::Message, ::xr::sdk::v2::ros::RosMessageResult>(request, &Stub::SubscribeLimitedBag, first_deadline, "SubscribeLimitedBag");
    }

    SdkResult<ResultStreamHandle<::xr::sdk::v2::ros::Message>> SubscribeUnlimitedBag(const ::xr::sdk::v2::ros::SubscribeParams& request,
                                                                                 Deadline first_deadline = Deadline{3000}) const
    {
        return ServerStream<::xr::sdk::v2::ros::Message, ::xr::sdk::v2::ros::RosMessageResult>(request, &Stub::SubscribeUnlimitedBag, first_deadline, "SubscribeUnlimitedBag");
    }

    SdkResult<ResultStreamHandle<::xr::sdk::v2::PongResponse>> SubscribePong(const ::xr::sdk::v2::PingRequest& request,
                                                                         Deadline first_deadline = Deadline{3000}) const
    {
        return ServerStream<::xr::sdk::v2::PongResponse, ::xr::sdk::v2::PongResult>(request, &Stub::SubscribePong, first_deadline, "SubscribePong");
    }

    SdkResult<::xr::sdk::v2::PongResponse> PublishPing(const std::vector<::xr::sdk::v2::PingRequest>& requests, Deadline deadline = Deadline{30000}) const
    {
        return ClientStream<::xr::sdk::v2::PingRequest, ::xr::sdk::v2::PongResult, ::xr::sdk::v2::PongResponse>(requests, &Stub::PublishPing, deadline, "PublishPing");
    }

    SdkResult<BidiResultStreamHandle<::xr::sdk::v2::PingRequest, ::xr::sdk::v2::PongResponse>> StreamPingPong() const
    {
        return BidiStream<::xr::sdk::v2::PingRequest, ::xr::sdk::v2::PongResult, ::xr::sdk::v2::PongResponse>(&Stub::StreamPingPong, "StreamPingPong");
    }

    SdkResult<::xr::sdk::v2::PongResponse> Ping(const ::xr::sdk::v2::PingRequest& request, Deadline deadline = Deadline{1000}) const
    {
        return Unary<::xr::sdk::v2::PongResponse, ::xr::sdk::v2::PongResult>(request, &Stub::Ping, deadline, "Ping");
    }
};

}  // namespace x2robot::sdk
