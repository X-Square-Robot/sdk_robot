#pragma once

#include "v2/sensors/vr.grpc.pb.h"
#include "x2robot/sdk/service/grpc_service.hpp"

namespace x2robot::sdk {

class VR final : public client::detail::GrpcServiceClient<::xr::sdk::v2::VR>
{
    using Service = ::xr::sdk::v2::VR;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit VR(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    SdkResult<ResultStreamHandle<::geometry_msgs::PoseStamped>> GetLeftArmPoseStream(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return this->template ServerStream<::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult>(request, &Stub::GetLeftArmPoseStream,
                                                                                                           deadline, "GetLeftArmPoseStream");
    }

    SdkResult<ResultStreamHandle<::geometry_msgs::PoseStamped>> GetRightArmPoseStream(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return this->template ServerStream<::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult>(request, &Stub::GetRightArmPoseStream,
                                                                                                           deadline, "GetRightArmPoseStream");
    }

    SdkResult<ResultStreamHandle<::geometry_msgs::PoseStamped>> GetHeadPoseStream(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return this->template ServerStream<::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult>(request, &Stub::GetHeadPoseStream,
                                                                                                           deadline, "GetHeadPoseStream");
    }

    SdkResult<ResultStreamHandle<::geometry_msgs::PoseStamped>> GetLeftArmPoseRelatedStream(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return this->template ServerStream<::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult>(
            request, &Stub::GetLeftArmPoseRelatedStream, deadline, "GetLeftArmPoseRelatedStream");
    }

    SdkResult<ResultStreamHandle<::geometry_msgs::PoseStamped>> GetRightArmPoseRelatedStream(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return this->template ServerStream<::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult>(
            request, &Stub::GetRightArmPoseRelatedStream, deadline, "GetRightArmPoseRelatedStream");
    }

    SdkResult<ResultStreamHandle<::geometry_msgs::PoseStamped>> GetHeadPoseRelatedStream(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return this->template ServerStream<::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult>(request, &Stub::GetHeadPoseRelatedStream,
                                                                                                           deadline, "GetHeadPoseRelatedStream");
    }
};

}  // namespace x2robot::sdk
