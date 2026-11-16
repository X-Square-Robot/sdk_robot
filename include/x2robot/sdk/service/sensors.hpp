#pragma once

#include "v2/sensors/data_collection.grpc.pb.h"
#include "v2/sensors/depth_points.grpc.pb.h"
#include "v2/sensors/imu.grpc.pb.h"
#include "v2/sensors/radar.grpc.pb.h"
#include "v2/sensors/tactile.grpc.pb.h"
#include "v2/sensors/tof.grpc.pb.h"
#include "v2/sensors/ultrasonic.grpc.pb.h"
#include "x2robot/sdk/service/grpc_service.hpp"

namespace x2robot::sdk {

#define X2ROBOT_SENSOR_GET(Method, StreamMethod, DataType, ResponseType)                                       \
    SdkResult<DataType> Method(Deadline deadline = Deadline{200}) const                                        \
    {                                                                                                          \
        ::google::protobuf::Empty request;                                                                     \
        return this->template UnaryFromStream<DataType, ResponseType>(request, &Stub::StreamMethod, deadline); \
    }

#define X2ROBOT_SENSOR_UNARY(Method, DataType, ResponseType)                                            \
    SdkResult<DataType> Method(Deadline deadline = Deadline{200}) const                                 \
    {                                                                                                   \
        ::google::protobuf::Empty request;                                                              \
        return this->template Unary<DataType, ResponseType>(request, &Stub::Method, deadline, #Method); \
    }

#define X2ROBOT_SENSOR_STREAM(Method, DataType, ResponseType)                                                        \
    SdkResult<ResultStreamHandle<DataType>> Method(Deadline first_deadline = Deadline{3000}) const                   \
    {                                                                                                                \
        ::google::protobuf::Empty request;                                                                           \
        return this->template ServerStream<DataType, ResponseType>(request, &Stub::Method, first_deadline, #Method); \
    }

class DataCollection final : public client::detail::GrpcServiceClient<::xr::sdk::v2::DataCollection>
{
    using Service = ::xr::sdk::v2::DataCollection;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit DataCollection(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_SENSOR_STREAM(GetVRLeftArmPoseStream, ::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult)
    X2ROBOT_SENSOR_STREAM(GetVRRightArmPoseStream, ::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult)
    X2ROBOT_SENSOR_STREAM(GetVRLeftGripperJointStateStream, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
    X2ROBOT_SENSOR_STREAM(GetVRRightGripperJointStateStream, ::sensor_msgs::JointState, ::xr::sdk::v2::JointStateResult)
    X2ROBOT_SENSOR_STREAM(GetVRHeadPoseStream, ::geometry_msgs::PoseStamped, ::xr::sdk::v2::PoseStampedResult)
};

class DepthPoints final : public client::detail::GrpcServiceClient<::xr::sdk::v2::DepthPoints>
{
    using Service = ::xr::sdk::v2::DepthPoints;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit DepthPoints(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_SENSOR_GET(GetChassisDepthPoints, GetChassisDepthPointsStream, ::sensor_msgs::PointCloud2, ::xr::sdk::v2::PointCloud2Result)
    X2ROBOT_SENSOR_STREAM(GetChassisDepthPointsStream, ::sensor_msgs::PointCloud2, ::xr::sdk::v2::PointCloud2Result)
    // 三维点云首包较大，默认等待时间与流式读取一致，调用者仍可指定更短的 deadline。
    SdkResult<::sensor_msgs::PointCloud2> GetLidarPoints(Deadline deadline = Deadline{3000}) const
    {
        ::google::protobuf::Empty request;
        return UnaryFromStream<::sensor_msgs::PointCloud2, ::xr::sdk::v2::PointCloud2Result>(
            request, &Stub::GetLidarPointsStream, deadline);
    }
    X2ROBOT_SENSOR_STREAM(GetLidarPointsStream, ::sensor_msgs::PointCloud2, ::xr::sdk::v2::PointCloud2Result)
};

class Imu final : public client::detail::GrpcServiceClient<::xr::sdk::v2::Imu>
{
    using Service = ::xr::sdk::v2::Imu;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit Imu(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_SENSOR_GET(GetChassisImu, GetChassisImuStream, ::sensor_msgs::Imu, ::xr::sdk::v2::ImuResult)
    X2ROBOT_SENSOR_STREAM(GetChassisImuStream, ::sensor_msgs::Imu, ::xr::sdk::v2::ImuResult)
};

class RadarService final : public client::detail::GrpcServiceClient<::xr::sdk::v2::RadarService>
{
    using Service = ::xr::sdk::v2::RadarService;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit RadarService(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_SENSOR_GET(GetLaserScan, GetLaserScanStream, ::sensor_msgs::LaserScan, ::xr::sdk::v2::LaserScanResult)
    X2ROBOT_SENSOR_STREAM(GetLaserScanStream, ::sensor_msgs::LaserScan, ::xr::sdk::v2::LaserScanResult)
};

template <typename Service>
class TactileClient : public client::detail::GrpcServiceClient<Service>
{
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = typename Service::StubInterface;

   public:
    explicit TactileClient(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_SENSOR_GET(GetTactileSensorData, GetTactileSensorDataStream, ::xr::sdk::v2::TactileSensorData, ::xr::sdk::v2::TactileSensorDataResult)
    X2ROBOT_SENSOR_STREAM(GetTactileSensorDataStream, ::xr::sdk::v2::TactileSensorData, ::xr::sdk::v2::TactileSensorDataResult)
};

using LeftHandTactile = TactileClient<::xr::sdk::v2::LeftHandTactile>;
using RightHandTactile = TactileClient<::xr::sdk::v2::RightHandTactile>;
using LeftGripperTactile = TactileClient<::xr::sdk::v2::LeftGripperTactile>;
using RightGripperTactile = TactileClient<::xr::sdk::v2::RightGripperTactile>;

class Tof final : public client::detail::GrpcServiceClient<::xr::sdk::v2::Tof>
{
    using Service = ::xr::sdk::v2::Tof;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit Tof(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_SENSOR_GET(GetChassisTof1, GetChassisTof1Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
    X2ROBOT_SENSOR_GET(GetChassisTof2, GetChassisTof2Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
    X2ROBOT_SENSOR_STREAM(GetChassisTof1Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
    X2ROBOT_SENSOR_STREAM(GetChassisTof2Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
};

class Ultrasonic final : public client::detail::GrpcServiceClient<::xr::sdk::v2::Ultrasonic>
{
    using Service = ::xr::sdk::v2::Ultrasonic;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit Ultrasonic(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    X2ROBOT_SENSOR_GET(GetChassisUltrasonic1, GetChassisUltrasonic1Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
    X2ROBOT_SENSOR_GET(GetChassisUltrasonic2, GetChassisUltrasonic2Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
    X2ROBOT_SENSOR_GET(GetChassisUltrasonic3, GetChassisUltrasonic3Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
    X2ROBOT_SENSOR_GET(GetChassisUltrasonic4, GetChassisUltrasonic4Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
    X2ROBOT_SENSOR_STREAM(GetChassisUltrasonic1Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
    X2ROBOT_SENSOR_STREAM(GetChassisUltrasonic2Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
    X2ROBOT_SENSOR_STREAM(GetChassisUltrasonic3Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
    X2ROBOT_SENSOR_STREAM(GetChassisUltrasonic4Stream, ::sensor_msgs::Range, ::xr::sdk::v2::RangeResult)
};

#undef X2ROBOT_SENSOR_STREAM
#undef X2ROBOT_SENSOR_UNARY
#undef X2ROBOT_SENSOR_GET

}  // namespace x2robot::sdk
