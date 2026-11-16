#pragma once

#include <memory>
#include <string>

#include "x2robot/sdk/arm/i_arm.hpp"
#include "x2robot/sdk/result.hpp"
#include "x2robot/sdk/types.hpp"

namespace x2robot::sdk::client {

class ServiceContext;

// Arm 域的 gRPC 客户端：protobuf 编解码并调用 proto 生成的 stub。
// 每条手臂在 proto 里是独立 service，故按臂持有各自的 stub。
class ArmGrpcClient final : public IArm {
 public:
  ArmGrpcClient(std::string target, ArmSide side, std::shared_ptr<ServiceContext> context = {});
  ~ArmGrpcClient() override;

  SdkResult<void> Connect();

  SdkResult<void> SetEndPose(const Pose& pose, Deadline deadline) override;
  SdkResult<void> SetJointPositions(const JointPositions& positions,
                                    Deadline deadline) override;
  SdkResult<JointState> GetJointStates(Deadline deadline) override;
  SdkResult<PoseStamped> GetEndPose(Deadline deadline) override;
  SdkResult<void> Reset(Deadline deadline) override;
  SdkResult<JointMotorStatus> GetMotorStatus(Deadline deadline) override;
  SdkResult<WrenchStamped> GetWrenchExtWorld(Deadline deadline) override;
  SdkResult<WrenchStamped> GetWrenchExtLocal(Deadline deadline) override;

  SdkResult<std::unique_ptr<ArmEndPoseStream>> GetEndPoseStream(Deadline first_deadline) override;
  SdkResult<std::unique_ptr<ArmJointStatesStream>> GetJointStatesStream(
      Deadline first_deadline) override;
  SdkResult<std::unique_ptr<ArmWrenchStream>> GetWrenchExtWorldStream(Deadline) override;
  SdkResult<std::unique_ptr<ArmWrenchStream>> GetWrenchExtLocalStream(Deadline) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::string target_;
  ArmSide side_;
  std::shared_ptr<ServiceContext> context_;
};

}  // namespace x2robot::sdk::client
