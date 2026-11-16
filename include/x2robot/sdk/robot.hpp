#pragma once

// 对外 C++ 接口层：值语义子句柄（设计文档 §4.2.2 / §4.2.3）

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "x2robot/sdk/arm/i_arm.hpp"
#include "x2robot/sdk/camera/i_camera.hpp"
#include "x2robot/sdk/core/core.hpp"
#include "x2robot/sdk/core/transport_router.hpp"
#include "x2robot/sdk/result.hpp"
#include "x2robot/sdk/service/audio.hpp"
#include "x2robot/sdk/service/benchmark.hpp"
#include "x2robot/sdk/service/body.hpp"
#include "x2robot/sdk/service/bridge.hpp"
#include "x2robot/sdk/service/core.hpp"
#include "x2robot/sdk/service/exo_hand.hpp"
#include "x2robot/sdk/service/files.hpp"
#include "x2robot/sdk/service/recording.hpp"
#include "x2robot/sdk/service/navigation.hpp"
#include "x2robot/sdk/service/sensors.hpp"
#include "x2robot/sdk/service/vr.hpp"
#include "x2robot/sdk/types.hpp"

namespace x2robot::sdk {

namespace client {
class IArm;
}

// 手臂 controller 的共享实现；公开 service 类型见 LeftArmController/RightArmController。
// 而调用进行中不会被并发 Close 抽走通道。
class ArmControllerClient
{
   public:
    explicit ArmControllerClient(std::weak_ptr<client::IArm> arm) : arm_(std::move(arm))
    {
    }

    SdkResult<void> SetEndPose(const Pose& pose, Deadline deadline = Deadline{200});
    SdkResult<void> SetJointPositions(const JointPositions& positions, Deadline deadline = Deadline{200});
    // 一元：复用 Stream 通道，取首帧后取消
    SdkResult<JointState> GetJointStates(Deadline deadline = Deadline{200});
    SdkResult<PoseStamped> GetEndPose(Deadline deadline = Deadline{200});
    SdkResult<void> Reset(Deadline deadline = Deadline{200});
    SdkResult<JointMotorStatus> GetMotorStatus(Deadline deadline = Deadline{200});
    SdkResult<WrenchStamped> GetWrenchExtWorld(Deadline deadline = Deadline{200});
    SdkResult<WrenchStamped> GetWrenchExtLocal(Deadline deadline = Deadline{200});

    SdkResult<std::unique_ptr<client::ArmEndPoseStream>> GetEndPoseStream(Deadline first_deadline = Deadline{200});
    SdkResult<std::unique_ptr<client::ArmJointStatesStream>> GetJointStatesStream(Deadline first_deadline = Deadline{200});
    SdkResult<std::unique_ptr<client::ArmWrenchStream>> GetWrenchExtWorldStream(Deadline first_deadline = Deadline{200});
    SdkResult<std::unique_ptr<client::ArmWrenchStream>> GetWrenchExtLocalStream(Deadline first_deadline = Deadline{200});

   private:
    std::weak_ptr<client::IArm> arm_;
};

class LeftArmController final : public ArmControllerClient
{
   public:
    using ArmControllerClient::ArmControllerClient;
};

class RightArmController final : public ArmControllerClient
{
   public:
    using ArmControllerClient::ArmControllerClient;
};

// 相机公开接口逐项对应 camera.proto 中的 service/RPC；传输层仍按 CameraId 复用实现。
class CameraServiceClient
{
   protected:
    explicit CameraServiceClient(std::weak_ptr<client::ICamera> camera, bool ex002 = false, std::weak_ptr<client::ServiceContext> context = {})
        : camera_(std::move(camera)), ex002_(ex002), context_(std::move(context))
    {
    }
    SdkResult<void> GetImage(CameraId id, const client::CameraFrameCallback& on_frame, Deadline deadline);
    SdkResult<client::VideoStream> GetVideoStream(CameraId id, client::CameraFrameCallback on_frame, Deadline first_deadline = Deadline{3000});
    std::weak_ptr<client::ICamera> camera_;
    bool ex002_{false};
    std::weak_ptr<client::ServiceContext> context_;
};

class HeadCamera final : private CameraServiceClient
{
   public:
    explicit HeadCamera(std::weak_ptr<client::ICamera> camera, bool ex002 = false, std::weak_ptr<client::ServiceContext> context = {}) : CameraServiceClient(std::move(camera), ex002, std::move(context))
    {
    }
    SdkResult<void> GetRGBImage(const client::CameraFrameCallback& cb, Deadline deadline = Deadline{3000});
    SdkResult<void> GetDepthImage(const client::CameraFrameCallback& cb, Deadline deadline = Deadline{3000});
    SdkResult<client::VideoStream> GetRGBVideoStream(client::CameraFrameCallback cb, Deadline first_deadline = Deadline{3000});
    SdkResult<client::VideoStream> GetDepthVideoStream(client::CameraFrameCallback cb, Deadline first_deadline = Deadline{3000});
    SdkResult<void> GetLeftEyeImage(const client::CameraFrameCallback& cb, Deadline deadline = Deadline{3000});
    SdkResult<void> GetRightEyeImage(const client::CameraFrameCallback& cb, Deadline deadline = Deadline{3000});
    SdkResult<client::VideoStream> GetLeftEyeVideoStream(client::CameraFrameCallback cb, Deadline first_deadline = Deadline{3000});
    SdkResult<client::VideoStream> GetRightEyeVideoStream(client::CameraFrameCallback cb, Deadline first_deadline = Deadline{3000});
};

class LeftArmCamera final : private CameraServiceClient
{
   public:
    explicit LeftArmCamera(std::weak_ptr<client::ICamera> camera, bool ex002 = false, std::weak_ptr<client::ServiceContext> context = {}) : CameraServiceClient(std::move(camera), ex002, std::move(context))
    {
    }
    SdkResult<void> GetRawImage(const client::CameraFrameCallback& cb, Deadline deadline = Deadline{3000});
    SdkResult<client::VideoStream> GetVideoStream(client::CameraFrameCallback cb, Deadline first_deadline = Deadline{3000});
    SdkResult<void> GetElbowImage(const client::CameraFrameCallback& cb, Deadline deadline = Deadline{3000});
    SdkResult<client::VideoStream> GetElbowVideoStream(client::CameraFrameCallback cb, Deadline first_deadline = Deadline{3000});
};

class RightArmCamera final : private CameraServiceClient
{
   public:
    explicit RightArmCamera(std::weak_ptr<client::ICamera> camera, bool ex002 = false, std::weak_ptr<client::ServiceContext> context = {}) : CameraServiceClient(std::move(camera), ex002, std::move(context))
    {
    }
    SdkResult<void> GetRawImage(const client::CameraFrameCallback& cb, Deadline deadline = Deadline{3000});
    SdkResult<client::VideoStream> GetVideoStream(client::CameraFrameCallback cb, Deadline first_deadline = Deadline{3000});
    SdkResult<void> GetElbowImage(const client::CameraFrameCallback& cb, Deadline deadline = Deadline{3000});
    SdkResult<client::VideoStream> GetElbowVideoStream(client::CameraFrameCallback cb, Deadline first_deadline = Deadline{3000});
};

// 机器人连接句柄：持有连接上下文 + 各业务域的通道
class Robot
{
   public:
    static SdkResult<std::unique_ptr<Robot>> Create(std::string_view address, bool force_grpc = false);

    static SdkResult<std::unique_ptr<Robot>> Create(std::string_view address, const client::ConnectOptions& options);
    ~Robot();
    // 内部通用桥复用这份上下文；不另建身份和控制状态。
    std::shared_ptr<client::ServiceContext> ConnectionContext() const { return services_; }
    SdkResult<void> Close();
    std::string ModelName() const;
    SdkResult<bool> SupportsService(std::string_view service) const;
    SdkResult<bool> SupportsRpc(std::string_view service, std::string_view rpc) const;
    std::string Address() const noexcept;
    bool IsLocal() const noexcept;
    std::string Transport() const noexcept;

    LeftArmController GetLeftArmController() const
    {
        return LeftArmController{left_arm_};
    }
    RightArmController GetRightArmController() const
    {
        return RightArmController{right_arm_};
    }

    HeadCamera GetHeadCamera() const
    {
        return HeadCamera{camera_, services_->IsEx002(), services_};
    }
    LeftArmCamera GetLeftArmCamera() const
    {
        return LeftArmCamera{camera_, services_->IsEx002(), services_};
    }
    RightArmCamera GetRightArmCamera() const
    {
        return RightArmCamera{camera_, services_->IsEx002(), services_};
    }

    LeftGripperController GetLeftGripperController() const
    {
        return LeftGripperController{services_};
    }
    RightGripperController GetRightGripperController() const
    {
        return RightGripperController{services_};
    }
    LeftHandController GetLeftHandController() const
    {
        return LeftHandController{services_};
    }
    RightHandController GetRightHandController() const
    {
        return RightHandController{services_};
    }
    HeadController GetHeadController() const
    {
        return HeadController{services_};
    }
    LiftController GetLiftController() const
    {
        return LiftController{services_};
    }
    WaistController GetWaistController() const
    {
        return WaistController{services_};
    }
    ChassisController GetChassisController() const
    {
        return ChassisController{services_};
    }
    RobotControl GetRobotControl() const
    {
        return RobotControl{services_};
    }
    MasterLeftArm GetMasterLeftArm() const
    {
        return MasterLeftArm{services_};
    }
    MasterRightArm GetMasterRightArm() const
    {
        return MasterRightArm{services_};
    }
    Recording GetRecording() const { return Recording{services_}; }

    Files GetFiles() const
    {
        return Files{services_};
    }

    Navigation GetNavigation() const
    {
        return Navigation{services_};
    }
    System GetSystem() const
    {
        return System{services_};
    }
    State GetState() const
    {
        return State{services_};
    }
    RobotStatus GetRobotStatusService() const
    {
        return RobotStatus{services_};
    }
    TfBridge GetTfBridge() const
    {
        return TfBridge{services_};
    }
    Audio GetAudio() const
    {
        return Audio{services_};
    }
    VR GetVR() const
    {
        return VR{services_};
    }
    ExoHand GetExoHand() const
    {
        return ExoHand{services_};
    }
    DataCollection GetDataCollection() const
    {
        return DataCollection{services_};
    }
    DepthPoints GetDepthPoints() const
    {
        return DepthPoints{services_};
    }
    Imu GetImu() const
    {
        return Imu{services_};
    }
    RadarService GetRadarService() const
    {
        return RadarService{services_};
    }
    Tof GetTof() const
    {
        return Tof{services_};
    }
    Ultrasonic GetUltrasonic() const
    {
        return Ultrasonic{services_};
    }
    LeftHandTactile GetLeftHandTactile() const
    {
        return LeftHandTactile{services_};
    }
    RightHandTactile GetRightHandTactile() const
    {
        return RightHandTactile{services_};
    }
    LeftGripperTactile GetLeftGripperTactile() const
    {
        return LeftGripperTactile{services_};
    }
    RightGripperTactile GetRightGripperTactile() const
    {
        return RightGripperTactile{services_};
    }
    RosBridge GetRosBridge() const
    {
        return RosBridge{services_};
    }
    Benchmark GetBenchmark() const
    {
        return Benchmark{services_};
    }

   private:
    Robot(std::shared_ptr<client::Core> core,
          std::shared_ptr<client::ServiceContext> services,
          std::shared_ptr<client::IArm> left,
          std::shared_ptr<client::IArm> right,
          std::shared_ptr<client::ICamera> camera)
        : core_(std::move(core)), services_(std::move(services)), left_arm_(std::move(left)), right_arm_(std::move(right)), camera_(std::move(camera))
    {
    }

    std::shared_ptr<client::Core> core_;
    std::shared_ptr<client::ServiceContext> services_;
    std::shared_ptr<client::IArm> left_arm_;
    std::shared_ptr<client::IArm> right_arm_;
    std::shared_ptr<client::ICamera> camera_;
};

}  // namespace x2robot::sdk
