#pragma once

// 压测帧的载荷布局：由合成帧源写入，由 bench_camera 解析。

#include <cstdint>
#include <cstring>

namespace x2robot::sdk {

// "X2BENCH1" 的小端字节序
inline constexpr uint64_t kCameraBenchMagic = 0x3148434E45423258ULL;

struct CameraBenchPrefix {
  uint64_t magic;
  uint64_t seq;
  int64_t dispatch_ns;  // steady_clock（CLOCK_MONOTONIC），同机跨进程可比
};

// 载荷够长才放前缀，否则退化成普通假数据
inline constexpr size_t kCameraBenchMinPayload = sizeof(CameraBenchPrefix);

inline bool ReadCameraBenchPrefix(const void* data, size_t size, CameraBenchPrefix* out) {
  if (data == nullptr || size < sizeof(CameraBenchPrefix)) return false;
  std::memcpy(out, data, sizeof(*out));
  return out->magic == kCameraBenchMagic;
}

}  // namespace x2robot::sdk
