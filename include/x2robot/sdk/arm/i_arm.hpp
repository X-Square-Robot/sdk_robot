#pragma once

// Arm 域客户端抽象。gRPC-only 分支由 ArmGrpcClient 实现，
// Core 在连接时选定其一，之后每次调用不再做传输分支。
//
// 注意与 server::IArm（L4 原子接口）区分：那是服务端本地驱动的能力抽象，
// 无 deadline、可以有尚未对外暴露的方法；两者靠 proto 契约对齐，不共享基类。

#include <memory>

#include "x2robot/sdk/result.hpp"
#include "x2robot/sdk/types.hpp"

namespace x2robot::sdk::client {

// GetEndPoseStream 句柄：持有 PendingResponse，Read 即 NextResponse；
// 析构/Cancel = 客户端取消，服务端停止推送。
class ArmEndPoseStream {
 public:
  virtual ~ArmEndPoseStream() = default;
  virtual SdkResult<PoseStamped> Read(Deadline deadline) = 0;
  virtual SdkResult<void> Cancel() = 0;
};

class ArmJointStatesStream {
 public:
  virtual ~ArmJointStatesStream() = default;
  virtual SdkResult<JointState> Read(Deadline deadline) = 0;
  virtual SdkResult<void> Cancel() = 0;
};

class ArmWrenchStream {
 public:
  virtual ~ArmWrenchStream() = default;
  virtual SdkResult<WrenchStamped> Read(Deadline deadline) = 0;
  virtual SdkResult<void> Cancel() = 0;
};

class IArm {
 public:
  virtual ~IArm() = default;

  virtual SdkResult<void> SetEndPose(const Pose& pose, Deadline deadline) = 0;
  virtual SdkResult<void> SetJointPositions(const JointPositions& positions,
                                            Deadline deadline) = 0;
  // 一元 Get*：复用对应 Stream 通道，取首帧后取消。
  virtual SdkResult<JointState> GetJointStates(Deadline deadline) = 0;
  virtual SdkResult<PoseStamped> GetEndPose(Deadline deadline) = 0;
  virtual SdkResult<void> Reset(Deadline deadline) = 0;
  virtual SdkResult<JointMotorStatus> GetMotorStatus(Deadline deadline) = 0;
  virtual SdkResult<WrenchStamped> GetWrenchExtWorld(Deadline deadline) = 0;
  virtual SdkResult<WrenchStamped> GetWrenchExtLocal(Deadline deadline) = 0;

  virtual SdkResult<std::unique_ptr<ArmEndPoseStream>> GetEndPoseStream(
      Deadline first_deadline = Deadline{200}) = 0;
  virtual SdkResult<std::unique_ptr<ArmJointStatesStream>> GetJointStatesStream(
      Deadline first_deadline = Deadline{200}) = 0;
  virtual SdkResult<std::unique_ptr<ArmWrenchStream>> GetWrenchExtWorldStream(
      Deadline first_deadline = Deadline{200}) = 0;
  virtual SdkResult<std::unique_ptr<ArmWrenchStream>> GetWrenchExtLocalStream(
      Deadline first_deadline = Deadline{200}) = 0;
};

}  // namespace x2robot::sdk::client
