#pragma once

#include "v2/controller/chassis_controller.grpc.pb.h"
#include "v2/controller/gripper_controller.grpc.pb.h"
#include "v2/controller/hand_controller.grpc.pb.h"
#include "v2/controller/head_controller.grpc.pb.h"
#include "v2/controller/lift_controller.grpc.pb.h"
#include "v2/controller/master_arm_controller.grpc.pb.h"
#include "v2/controller/robot_control.grpc.pb.h"
#include "v2/controller/waist_controller.grpc.pb.h"
#include "x2robot/sdk/service/grpc_service.hpp"

namespace x2robot::sdk {

#define X2ROBOT_RPC_VOID(Method, RequestType)                                                          \
    SdkResult<void> Method(const RequestType& request, Deadline deadline = Deadline{200}) const        \
    {                                                                                                  \
        return this->template UnaryVoid<::xr::sdk::v2::EmptyResult>(request, &Stub::Method, deadline, #Method); \
    }

#define X2ROBOT_RPC_GET(Method, DataType, ResponseType)                                        \
    SdkResult<DataType> Method(Deadline deadline = Deadline{200}) const                        \
    {                                                                                          \
        ::google::protobuf::Empty request;                                                     \
        return this->template Unary<DataType, ResponseType>(request, &Stub::Method, deadline, #Method); \
    }

#define X2ROBOT_RPC_STREAM(Method, DataType, ResponseType)                                                  \
    SdkResult<ResultStreamHandle<DataType>> Method(Deadline first_deadline = Deadline{3000}) const          \
    {                                                                                                       \
        ::google::protobuf::Empty request;                                                                  \
        return this->template ServerStream<DataType, ResponseType>(request, &Stub::Method, first_deadline, #Method); \
    }

template <typename Service>
class GripperControllerClient : public client::detail::GrpcServiceClient<Service>
{
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = typename Service::StubInterface;

   public:
    explicit GripperControllerClient(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_RPC_VOID(SetPosition, ::xr::sdk::v2::GripperPosition)
    X2ROBOT_RPC_GET(GetPosition, ::xr::sdk::v2::GripperPosition, ::xr::sdk::v2::GripperPositionResult)
    X2ROBOT_RPC_STREAM(GetPositionStream, ::xr::sdk::v2::GripperPosition, ::xr::sdk::v2::GripperPositionResult)
    X2ROBOT_RPC_GET(GetMotorStatus, ::status_code::JointMotorStatus, ::xr::sdk::v2::JointMotorStatusResult)
    X2ROBOT_RPC_STREAM(GetJointStatesStream, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
};

using LeftGripperController = GripperControllerClient<::xr::sdk::v2::LeftGripperController>;
using RightGripperController = GripperControllerClient<::xr::sdk::v2::RightGripperController>;

template <typename Service>
class HandControllerClient : public client::detail::GrpcServiceClient<Service>
{
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = typename Service::StubInterface;

   public:
    explicit HandControllerClient(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_RPC_VOID(SetJointPositions, ::xr::sdk::v2::JointPositions)
    X2ROBOT_RPC_GET(GetJointStates, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
    SdkResult<void> Reset(Deadline deadline = Deadline{200}) const
    {
        ::google::protobuf::Empty request;
        return this->template UnaryVoid<::xr::sdk::v2::EmptyResult>(request, &Stub::Reset, deadline, "Reset");
    }
    X2ROBOT_RPC_STREAM(GetJointStatesStream, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
};

using LeftHandController = HandControllerClient<::xr::sdk::v2::LeftHandController>;
using RightHandController = HandControllerClient<::xr::sdk::v2::RightHandController>;

class HeadController final : public client::detail::GrpcServiceClient<::xr::sdk::v2::HeadController>
{
    using Service = ::xr::sdk::v2::HeadController;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit HeadController(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_RPC_VOID(SetPose, ::xr::sdk::v2::HeadPose)
    X2ROBOT_RPC_GET(GetPose, ::xr::sdk::v2::HeadPose, ::xr::sdk::v2::HeadPoseResult)
    SdkResult<void> Reset(Deadline deadline = Deadline{200}) const
    {
        ::google::protobuf::Empty request;
        return UnaryVoid<::xr::sdk::v2::EmptyResult>(request, &Stub::Reset, deadline, "Reset");
    }
    X2ROBOT_RPC_STREAM(GetJointStatesStream, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
};

class LiftController final : public client::detail::GrpcServiceClient<::xr::sdk::v2::LiftController>
{
    using Service = ::xr::sdk::v2::LiftController;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit LiftController(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_RPC_VOID(SetLiftPosition, ::xr::sdk::v2::LiftPosition)
    X2ROBOT_RPC_GET(GetLiftPosition, ::xr::sdk::v2::LiftPosition, ::xr::sdk::v2::LiftPositionResult)
    X2ROBOT_RPC_GET(GetMotorStatus, ::status_code::JointMotorStatus, ::xr::sdk::v2::JointMotorStatusResult)
    X2ROBOT_RPC_STREAM(GetJointStatesStream, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
};

class WaistController final : public client::detail::GrpcServiceClient<::xr::sdk::v2::WaistController>
{
    using Service = ::xr::sdk::v2::WaistController;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit WaistController(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_RPC_VOID(SetJointPositions, ::xr::sdk::v2::JointPositions)
    X2ROBOT_RPC_VOID(SetEndPose, ::geometry_msgs::Pose)
    X2ROBOT_RPC_GET(GetJointStates, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
    X2ROBOT_RPC_STREAM(GetJointStatesStream, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
    X2ROBOT_RPC_GET(GetEndPose, ::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult)
    X2ROBOT_RPC_STREAM(GetEndPoseStream, ::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult)
};

class ChassisController final : public client::detail::GrpcServiceClient<::xr::sdk::v2::ChassisController>
{
    using Service = ::xr::sdk::v2::ChassisController;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit ChassisController(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_RPC_VOID(SetControlMode, ::xr::sdk::v2::ChassisControlModeParam)
    X2ROBOT_RPC_GET(GetControlMode, ::xr::sdk::v2::ChassisControlModeParam, ::xr::sdk::v2::ChassisControlModeResult)
    X2ROBOT_RPC_VOID(MoveToGlobalPosition, ::xr::sdk::v2::ChassisPosition)
    X2ROBOT_RPC_VOID(MoveToRelativePosition, ::xr::sdk::v2::ChassisPosition)
    X2ROBOT_RPC_VOID(SetVelocity, ::xr::sdk::v2::ChassisVelocity)
    X2ROBOT_RPC_VOID(SetVirtualZeroPoint, ::xr::sdk::v2::ChassisPosition)
    X2ROBOT_RPC_GET(GetVirtualZeroPoint, ::xr::sdk::v2::ChassisPosition, ::xr::sdk::v2::ChassisPositionResult)
    X2ROBOT_RPC_GET(GetGlobalPosition, ::xr::sdk::v2::ChassisPosition, ::xr::sdk::v2::ChassisPositionResult)
    X2ROBOT_RPC_GET(GetRelativePosition, ::xr::sdk::v2::ChassisPosition, ::xr::sdk::v2::ChassisPositionResult)
    X2ROBOT_RPC_GET(GetOdometry, ::nav_msgs::Odometry, ::xr::sdk::v2::OdometryResult)
    X2ROBOT_RPC_STREAM(GetOdometryStream, ::nav_msgs::Odometry, ::xr::sdk::v2::OdometryResult)
    X2ROBOT_RPC_STREAM(GetPoseStream, ::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult)
    X2ROBOT_RPC_VOID(SendRelativePoseToNavigation, ::xr::sdk::v2::ChassisPositionList)
    SdkResult<void> ResetNavigationChunkId(Deadline deadline = Deadline{200}) const
    {
        ::google::protobuf::Empty request;
        return UnaryVoid<::xr::sdk::v2::EmptyResult>(request, &Stub::ResetNavigationChunkId, deadline, "ResetNavigationChunkId");
    }
    X2ROBOT_RPC_VOID(SendRelativePoseBasedRobotToNavigation, ::xr::sdk::v2::ChassisPositionList)
    X2ROBOT_RPC_VOID(SetTrajectoryCoordSystemMode, ::xr::sdk::v2::CoordinateSystemModeParam)
};

class RobotControl final : public client::detail::GrpcServiceClient<::xr::sdk::v2::RobotControl>
{
    using Service = ::xr::sdk::v2::RobotControl;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit RobotControl(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_RPC_VOID(SetManipulatorControlMode, ::xr::sdk::v2::ManipulatorControlModeParam)
    X2ROBOT_RPC_GET(GetManipulatorControlMode, ::xr::sdk::v2::ManipulatorControlModeParam, ::xr::sdk::v2::ManipulatorControlModeResult)

#define X2ROBOT_ROBOT_CONTROL_EMPTY(Method)                                             \
    SdkResult<void> Method(Deadline deadline = Deadline{200}) const                     \
    {                                                                                   \
        ::google::protobuf::Empty request;                                              \
        return UnaryVoid<::xr::sdk::v2::EmptyResult>(request, &Stub::Method, deadline, #Method); \
    }
    X2ROBOT_ROBOT_CONTROL_EMPTY(Homing)
    X2ROBOT_ROBOT_CONTROL_EMPTY(EmergencyStop)
    X2ROBOT_ROBOT_CONTROL_EMPTY(RecoverEmergencyStop)
#undef X2ROBOT_ROBOT_CONTROL_EMPTY
};

template <typename Service>
class MasterArmClient : public client::detail::GrpcServiceClient<Service>
{
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = typename Service::StubInterface;

   public:
    explicit MasterArmClient(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_RPC_GET(GetControlMode, ::xr::sdk::v2::ManipulatorControlModeParam, ::xr::sdk::v2::ManipulatorControlModeResult)
    X2ROBOT_RPC_VOID(SetControlMode, ::xr::sdk::v2::ManipulatorControlModeParam)
    X2ROBOT_RPC_GET(GetJointStates, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
    X2ROBOT_RPC_GET(GetEndPose, ::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult)
    X2ROBOT_RPC_GET(GetGripperPosition, ::xr::sdk::v2::GripperPosition, ::xr::sdk::v2::GripperPositionResult)
    X2ROBOT_RPC_STREAM(GetJointStatesStream, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
    X2ROBOT_RPC_STREAM(GetEndPoseStream, ::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult)
    X2ROBOT_RPC_STREAM(GetGripperStateStream, ::xr::sdk::v2::GripperPosition, ::xr::sdk::v2::GripperPositionResult)
    X2ROBOT_RPC_VOID(SetJointPositions, ::xr::sdk::v2::JointPositions)
    X2ROBOT_RPC_VOID(SetEndPose, ::geometry_msgs::Pose)
    X2ROBOT_RPC_VOID(SetGripperPosition, ::xr::sdk::v2::GripperPosition)
    X2ROBOT_RPC_STREAM(GetGripperJointStatesStream, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
};

using MasterLeftArm = MasterArmClient<::xr::sdk::v2::MasterLeftArm>;
using MasterRightArm = MasterArmClient<::xr::sdk::v2::MasterRightArm>;

#undef X2ROBOT_RPC_STREAM
#undef X2ROBOT_RPC_GET
#undef X2ROBOT_RPC_VOID

}  // namespace x2robot::sdk
