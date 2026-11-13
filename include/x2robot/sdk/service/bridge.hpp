#pragma once

#include <vector>

#include "v2/common/ros_bridge.grpc.pb.h"
#include "x2robot/sdk/service/grpc_service.hpp"

namespace x2robot::sdk {

class RosBridge final : public client::detail::GrpcServiceClient<::xr::sdk::v2::ros::RosBridge>
{
    using Service = ::xr::sdk::v2::ros::RosBridge;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit RosBridge(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    SdkResult<void> PublishTopic(const std::vector<::xr::sdk::v2::ros::Message>& messages, Deadline deadline = Deadline{30000}) const
    {
        return ClientStreamVoid<::xr::sdk::v2::ros::Message, ::xr::sdk::v2::EmptyResult>(messages, &Stub::PublishTopic, deadline, "PublishTopic");
    }

    SdkResult<ResultStreamHandle<::xr::sdk::v2::ros::Message>> SubscribeTopic(const ::xr::sdk::v2::ros::SubscribeParams& request,
                                                                          Deadline first_deadline = Deadline{3000}) const
    {
        return ServerStream<::xr::sdk::v2::ros::Message, ::xr::sdk::v2::ros::RosMessageResult>(request, &Stub::SubscribeTopic, first_deadline, "SubscribeTopic");
    }
};

}  // namespace x2robot::sdk
