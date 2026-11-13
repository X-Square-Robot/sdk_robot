#pragma once

#include "v2/sensors/exo_hand.grpc.pb.h"
#include "x2robot/sdk/service/grpc_service.hpp"

namespace x2robot::sdk {

class ExoHand final : public client::detail::GrpcServiceClient<::xr::sdk::v2::ExoHand>
{
    using Service = ::xr::sdk::v2::ExoHand;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit ExoHand(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    SdkResult<::sensor_msgs::JointState> GetLeftJointState(Deadline deadline = Deadline{200}) const
    {
        ::google::protobuf::Empty request;
        return this->template Unary<::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult>(request, &Stub::GetLeftJointState, deadline,
                                                                                                "GetLeftJointState");
    }

    SdkResult<ResultStreamHandle<::sensor_msgs::JointState>> GetLeftJointStateStream(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return this->template ServerStream<::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult>(request, &Stub::GetLeftJointStateStream,
                                                                                                       deadline, "GetLeftJointStateStream");
    }

    SdkResult<::sensor_msgs::JointState> GetRightJointState(Deadline deadline = Deadline{200}) const
    {
        ::google::protobuf::Empty request;
        return this->template Unary<::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult>(request, &Stub::GetRightJointState, deadline,
                                                                                                "GetRightJointState");
    }

    SdkResult<ResultStreamHandle<::sensor_msgs::JointState>> GetRightJointStateStream(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return this->template ServerStream<::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult>(request, &Stub::GetRightJointStateStream,
                                                                                                       deadline, "GetRightJointStateStream");
    }
    SdkResult<ResultStreamHandle<::sensor_msgs::CompressedImage>> GetLeftCameraStream(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return this->template ServerStream<::sensor_msgs::CompressedImage, ::xr::sdk::v2::CompressedImageResult>(request, &Stub::GetLeftCameraStream,
                                                                                                                 deadline, "GetLeftCameraStream");
    }

    SdkResult<ResultStreamHandle<::sensor_msgs::CompressedImage>> GetRightCameraStream(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return this->template ServerStream<::sensor_msgs::CompressedImage, ::xr::sdk::v2::CompressedImageResult>(request, &Stub::GetRightCameraStream,
                                                                                                                 deadline, "GetRightCameraStream");
    }
};

}  // namespace x2robot::sdk
