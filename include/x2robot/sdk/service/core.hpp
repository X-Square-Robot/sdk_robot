#pragma once

#include <string>

#include "v2/core/robot_status.grpc.pb.h"
#include "v2/core/state.grpc.pb.h"
#include "v2/core/system.grpc.pb.h"
#include "v2/core/tf_bridge.grpc.pb.h"
#include "x2robot/sdk/service/grpc_service.hpp"

namespace x2robot::sdk {

class System final : public client::detail::GrpcServiceClient<::xr::sdk::v2::System>
{
    using Service = ::xr::sdk::v2::System;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit System(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    SdkResult<void> SetWorkMode(const ::xr::sdk::v2::RobotModeParam& request, Deadline deadline = Deadline{200}) const
    {
        const auto id = NewRequestId();
        if (auto error = client::ValidateDeadline(deadline)) return SdkResult<void>::Err(*error, id);
        auto context = Context();
        if (!context) return SdkResult<void>::Err(client::detail::ClosedError(), id);
        return context->SetWorkMode(request, deadline, id);
    }
    SdkResult<::xr::sdk::v2::RobotStaticInfoData> GetStaticInfo(Deadline deadline = Deadline{200}) const
    {
        ::google::protobuf::Empty request;
        return Unary<::xr::sdk::v2::RobotStaticInfoData, ::xr::sdk::v2::RobotStaticInfoResult>(request, &Stub::GetStaticInfo, deadline, "GetStaticInfo");
    }
    SdkResult<::xr::sdk::v2::RobotDynamicInfo> GetDynamicInfo(Deadline deadline = Deadline{200}) const
    {
        ::google::protobuf::Empty request;
        return Unary<::xr::sdk::v2::RobotDynamicInfo, ::xr::sdk::v2::RobotDynamicInfoResult>(request, &Stub::GetDynamicInfo, deadline, "GetDynamicInfo");
    }
    SdkResult<::xr::sdk::v2::ModelTypeData> GetModelType(Deadline deadline = Deadline{200}) const
    {
        ::google::protobuf::Empty request;
        return Unary<::xr::sdk::v2::ModelTypeData, ::xr::sdk::v2::ModelTypeResult>(request, &Stub::GetModelType, deadline, "GetModelType");
    }
    SdkResult<::xr::sdk::v2::ErrorLogList> QueryErrorLogs(const ::xr::sdk::v2::ErrorLogQuery& request, Deadline deadline = Deadline{1000}) const
    {
        return Unary<::xr::sdk::v2::ErrorLogList, ::xr::sdk::v2::ErrorLogListResult>(request, &Stub::QueryErrorLogs, deadline, "QueryErrorLogs");
    }
    SdkResult<::xr::sdk::v2::RuntimeStatusList> QueryRuntimeStatus(const ::xr::sdk::v2::RuntimeStatusQuery& request,
                                                                   Deadline deadline = Deadline{1000}) const
    {
        return Unary<::xr::sdk::v2::RuntimeStatusList, ::xr::sdk::v2::RuntimeStatusListResult>(request, &Stub::QueryRuntimeStatus, deadline, "QueryRuntimeStatus");
    }
};

class State final : public client::detail::GrpcServiceClient<::xr::sdk::v2::State>
{
    using Service = ::xr::sdk::v2::State;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit State(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    SdkResult<::xr::sdk::v2::HeartBeatResponse> HeartBeat(const ::xr::sdk::v2::HeartBeatRequest& request, Deadline deadline = Deadline{1000}) const
    {
        return Unary<::xr::sdk::v2::HeartBeatResponse, ::xr::sdk::v2::HeartBeatResult>(request, &Stub::HeartBeat, deadline, "HeartBeat");
    }
    SdkResult<::sensor_msgs::JointState> GetAllJointStates(Deadline deadline = Deadline{200}) const
    {
        ::google::protobuf::Empty request;
        return Unary<::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult>(request, &Stub::GetAllJointStates, deadline, "GetAllJointStates");
    }
    SdkResult<ResultStreamHandle<::sensor_msgs::JointState>> GetAllJointStatesStream(Deadline first_deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return ServerStream<::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult>(request, &Stub::GetAllJointStatesStream, first_deadline, "GetAllJointStatesStream");
    }
};

class RobotStatus final : public client::detail::GrpcServiceClient<::xr::sdk::v2::RobotStatus>
{
    using Service = ::xr::sdk::v2::RobotStatus;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit RobotStatus(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    SdkResult<std::string> GetRobotStatus(const ::xr::sdk::v2::GetRobotStatusRequest& request, Deadline deadline = Deadline{1000}) const
    {
        return UnaryScalar<std::string, ::xr::sdk::v2::RobotStatusResult>(request, &Stub::GetRobotStatus, deadline,
                                                                          [](const auto& response) { return response.json(); });
    }
};

class TfBridge final : public client::detail::GrpcServiceClient<::xr::sdk::v2::TfBridge>
{
    using Service = ::xr::sdk::v2::TfBridge;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit TfBridge(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    SdkResult<ResultStreamHandle<::tf2_msgs::TFMessage>> SubscribeTf(Deadline first_deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return ServerStream<::tf2_msgs::TFMessage, ::xr::sdk::v2::TfMessageResult>(request, &Stub::SubscribeTf, first_deadline, "SubscribeTf");
    }
    SdkResult<ResultStreamHandle<::tf2_msgs::TFMessage>> SubscribeTfStatic(Deadline first_deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return ServerStream<::tf2_msgs::TFMessage, ::xr::sdk::v2::TfMessageResult>(request, &Stub::SubscribeTfStatic, first_deadline, "SubscribeTfStatic");
    }
};

}  // namespace x2robot::sdk
