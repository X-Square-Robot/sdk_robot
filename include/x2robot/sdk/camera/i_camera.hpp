#pragma once

// 相机域客户端抽象。gRPC-only 分支由 CameraGrpcClient 实现。
//
// 形状照着 gRPC 的流式调用来：开流一次调用拿到一个句柄，句柄活着就一直收，
// 丢掉就是关流。没有"按位号关"的接口，因此也不存在漏关、重复关、
// 或者"这颗相机已经在用了"这种由实现漏出来的错误——同一位号可以同时开多路。
//
// 取单帧不是第三种动作，而是开流拿到第一帧就关，两种传输的 GetImage 都这么实现。

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string_view>

#include "x2robot/sdk/result.hpp"
#include "x2robot/sdk/types.hpp"

namespace x2robot::sdk::client {

// 一帧图像。data 指向 gRPC 解码后的消息缓冲区，只在回调返回前有效；
// 要留到回调之外需自行复制。
struct CameraFrameView {
  std::string_view format;
  const uint8_t* data{nullptr};
  size_t size{0};
  uint64_t seq{0};
  int64_t stamp_ns{0};  // proto CompressedImage.header.stamp，0 表示源未提供
};

// 在传输层的收帧线程上调用，别在里面做重活或反过来调 SDK 的其他接口
using CameraFrameCallback = std::function<void(const SdkResult<CameraFrameView>&)>;

// 一路已开的视频流。析构即关流：返回后帧回调不会再被调到。
//
// 【不要在自己的帧回调里销毁它】关流要等收帧线程离开，而那条线程正是回调所在，
// 等于等自己。与 gRPC 不能在 reactor 回调里同步销毁 ClientContext 是同一条规矩。
class IVideoStream {
 public:
  virtual ~IVideoStream() = default;
};

using VideoStream = std::unique_ptr<IVideoStream>;

class ICamera {
 public:
  virtual ~ICamera() = default;

  // 开一路流。同一位号可以同时开多路，各收各的。
  virtual SdkResult<VideoStream> GetVideoStream(
      CameraId id, CameraFrameCallback on_frame,
      Deadline first_deadline = Deadline{3000}) = 0;

  // 取单帧：内部开流、等第一帧、关流。deadline 内没等到帧返回 kTimeout。
  virtual SdkResult<void> GetImage(CameraId id, const CameraFrameCallback& on_frame,
                                   Deadline deadline) = 0;
};

}  // namespace x2robot::sdk::client
