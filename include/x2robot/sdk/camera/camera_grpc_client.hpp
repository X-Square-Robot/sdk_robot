#pragma once

// 相机域的 gRPC 客户端。
//
// gRPC 的 server-streaming 本身就带"开/关"语义：发起调用即开流，取消 context 即
// 关流，服务端 reactor 的 OnCancel 会去退掉 ROS2 订阅。所以这里不需要额外的控制面，
// 每路流一条读线程把帧喂给回调。
//
// 帧是 protobuf 解出来的一块进程内内存，回调看到的 CameraFrameView 指向它，
// 只在回调期间有效。
//
// 【线程安全】两个接口都可以多线程调，同一位号并发开多路也没问题——每路流各自
// 一条 gRPC 调用，本来就是独立的。

#include <memory>
#include <string>

#include "x2robot/sdk/camera/i_camera.hpp"
#include "x2robot/sdk/result.hpp"
#include "x2robot/sdk/types.hpp"

namespace x2robot::sdk::client {

class ServiceContext;

class CameraGrpcClient final : public ICamera {
 public:
  explicit CameraGrpcClient(std::string target, std::shared_ptr<ServiceContext> context = {});
  ~CameraGrpcClient() override;

  SdkResult<void> Connect();

  SdkResult<VideoStream> GetVideoStream(CameraId id, CameraFrameCallback on_frame,
                                        Deadline first_deadline) override;
  SdkResult<void> GetImage(CameraId id, const CameraFrameCallback& on_frame,
                           Deadline deadline) override;

 private:
  struct Impl;

  std::string target_;
  std::shared_ptr<Impl> impl_;
  std::shared_ptr<ServiceContext> context_;
};

}  // namespace x2robot::sdk::client
