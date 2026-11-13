#pragma once

#include <chrono>
#include <cstdint>

#include "v2/common/types.pb.h"
#include "ros2/joint_state.pb.h"
#include "ros2/joint_motor_status.pb.h"
#include "ros2/pose.pb.h"
#include "ros2/pose_stamped.pb.h"
#include "ros2/wrench_stamped.pb.h"

namespace x2robot::sdk {

enum class TransportType {
  kGrpcTcp,
};

inline const char* TransportToString(TransportType /*transport*/) { return "grpc"; }

enum class ArmSide {
  kLeft,
  kRight,
};

inline const char* ArmSideToString(ArmSide s) {
  return s == ArmSide::kLeft ? "left" : "right";
}

// gRPC service 名（对应 sdk_protocol 的 v2/controller/arm_controller.proto），
// 同时用作 CallCtx.method_id 的前缀
inline const char* ArmServiceName(ArmSide s) {
  return s == ArmSide::kLeft ? "LeftArmController" : "RightArmController";
}

// 业务数据类型只来自 proto。这里仅提供兼容别名，不再维护第二份字段定义。
using Pose = ::geometry_msgs::Pose;
using PoseStamped = ::geometry_msgs::PoseStamped;
using JointState = ::sensor_msgs::JointState;
using JointPositions = ::xr::sdk::v2::JointPositions;
using JointMotorStatus = ::status_code::JointMotorStatus;
using WrenchStamped = ::geometry_msgs::WrenchStamped;

using Deadline = std::chrono::milliseconds;

enum class MethodId : uint32_t {
  kSetEndPose = 1,
  kGetJointStates = 2,
};

// 相机位号。哪些位号在本机型上真实存在由服务端档案决定，客户端要到 Open 才知道。
// 放在这里而不是服务端的 robot_profile.hpp：客户端也使用同一套位号。
enum class CameraId {
  kHeadRgb,
  kHeadDepth,
  kHeadLeftEye,
  kHeadRightEye,
  kLeftWrist,
  kLeftElbow,
  kRightWrist,
  kRightElbow,
};

inline constexpr uint64_t kCameraIdCount = 8;

// yml 里的键名，如 kLeftWrist → "left_wrist"
inline const char* CameraIdKey(CameraId id) {
  switch (id) {
    case CameraId::kHeadRgb:
      return "head_rgb";
    case CameraId::kHeadDepth:
      return "head_depth";
    case CameraId::kHeadLeftEye:
      return "head_left_eye";
    case CameraId::kHeadRightEye:
      return "head_right_eye";
    case CameraId::kLeftWrist:
      return "left_wrist";
    case CameraId::kLeftElbow:
      return "left_elbow";
    case CameraId::kRightWrist:
      return "right_wrist";
    case CameraId::kRightElbow:
      return "right_elbow";
  }
  return "unknown";
}

inline constexpr uint16_t kDefaultGrpcPort = 55051;

}  // namespace x2robot::sdk
