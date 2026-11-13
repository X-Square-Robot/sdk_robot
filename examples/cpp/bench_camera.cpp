// Camera gRPC server-streaming benchmark: measures throughput, latency, and dropped frames.
// Usage: ./bench_camera <grpc_addr> <camera_key|all|key1,key2> [seconds]
// Example: ./bench_camera 127.0.0.1:55051 left_wrist 10
//          ./bench_camera 10.100.20.173:55051 all 20     # every camera at once
//
// "all" (or a comma-separated list) subscribes to several cameras over ONE Robot
// connection, which is the shape of the reported "all data streams, a few run at
// low fps" scenario; a single camera run would hide cross-stream contention.
//
// Uses the public client API, so the results include the full cost of starting the stream,
// receiving frames, and stopping the stream.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "x2robot/sdk/camera_bench_frame.hpp"
#include "x2robot/sdk/robot.hpp"

namespace {
using Clock = std::chrono::steady_clock;
using x2robot::sdk::CameraBenchPrefix;
using x2robot::sdk::CameraId;
using x2robot::sdk::ReadCameraBenchPrefix;
using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using x2robot::sdk::client::CameraFrameView;

struct BenchResult {
  uint64_t frames{0};
  uint64_t bytes{0};
  double elapsed_s{0};
  double mbps{0};
  double fps{0};
  uint64_t gaps{0};
  uint64_t lost{0};

  // Latency comes from the benchmark prefix in the payload. Real camera frames do not contain
  // this prefix, so latency is reported as n/a for those frames.
  bool has_latency{false};
  double mean_us{0};
  double p99_us{0};
};

auto GetVideoStream(Robot& robot, CameraId id, x2robot::sdk::client::CameraFrameCallback cb) {
  switch (id) {
    case CameraId::kHeadRgb: return robot.GetHeadCamera().GetRGBVideoStream(std::move(cb));
    case CameraId::kHeadDepth: return robot.GetHeadCamera().GetDepthVideoStream(std::move(cb));
    case CameraId::kHeadLeftEye: return robot.GetHeadCamera().GetLeftEyeVideoStream(std::move(cb));
    case CameraId::kHeadRightEye: return robot.GetHeadCamera().GetRightEyeVideoStream(std::move(cb));
    case CameraId::kLeftWrist: return robot.GetLeftArmCamera().GetVideoStream(std::move(cb));
    case CameraId::kLeftElbow: return robot.GetLeftArmCamera().GetElbowVideoStream(std::move(cb));
    case CameraId::kRightWrist: return robot.GetRightArmCamera().GetVideoStream(std::move(cb));
    case CameraId::kRightElbow: return robot.GetRightArmCamera().GetElbowVideoStream(std::move(cb));
  }
  return robot.GetHeadCamera().GetRGBVideoStream(std::move(cb));
}

// Calculate latency and dropped frames from the benchmark prefix. Both transports read the
// timestamp and sequence number written during the same dispatch, so this measures the time each
// frame spends after leaving the source, including any transport-specific queuing.
class LatencyTracker {
 public:
  // Returns false when the frame has no benchmark prefix, as with a real camera frame. Such frames
  // are excluded from latency statistics.
  bool Feed(const void* data, size_t size, BenchResult* r) {
    CameraBenchPrefix p;
    if (!ReadCameraBenchPrefix(data, size, &p)) return false;

    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               Clock::now().time_since_epoch())
                               .count();
    lat_us_.push_back(static_cast<double>(now_ns - p.dispatch_ns) / 1000.0);

    if (!first_ && p.seq != last_seq_ + 1) {
      ++r->gaps;
      r->lost += (p.seq > last_seq_) ? (p.seq - last_seq_ - 1) : 0;
    }
    last_seq_ = p.seq;
    first_ = false;
    return true;
  }

  void Finalize(BenchResult* r) {
    if (lat_us_.empty()) return;
    r->has_latency = true;
    r->mean_us = std::accumulate(lat_us_.begin(), lat_us_.end(), 0.0) / lat_us_.size();
    std::sort(lat_us_.begin(), lat_us_.end());
    r->p99_us = lat_us_[std::min(lat_us_.size() - 1, size_t(0.99 * lat_us_.size()))];
  }

 private:
  std::vector<double> lat_us_;
  uint64_t last_seq_{0};
  bool first_{true};
};

void Report(const char* name, const BenchResult& r) {
  char lat[64];
  if (r.has_latency) {
    std::snprintf(lat, sizeof(lat), "latency mean %6.1f p99 %7.1f us", r.mean_us, r.p99_us);
  } else {
    std::snprintf(lat, sizeof(lat), "%-42s", "latency n/a (frame has no benchmark timestamp)");
  }
  std::printf("  %-22s frames %7llu  %.1f fps  %6.2f MB/s  %s  gaps %llu lost %llu\n",
              name, (unsigned long long)r.frames, r.fps, r.mbps, lat,
              (unsigned long long)r.gaps, (unsigned long long)r.lost);
}

// Start a stream, receive frames for the requested duration, and stop the stream. Statistics are
// accumulated directly on the frame delivery thread. Each stream has one delivery thread, and
// stopping the stream waits for that thread to exit, so the main thread can safely read the fields.
BenchResult Bench(const std::string& address, CameraId id, double seconds) {
  auto created = Robot::Create(address);
  if (!created.Ok()) {
    std::fprintf(stderr, "connect failed: %s\n", created.ErrorValue().error_message.c_str());
    return {};
  }
  auto robot = std::move(created.Value());
  BenchResult r;
  LatencyTracker tracker;

  auto opened = GetVideoStream(*robot, id, [&](const SdkResult<CameraFrameView>& frame) {
    if (!frame.Ok()) return;
    const auto& f = frame.Value();
    tracker.Feed(f.data, f.size, &r);
    ++r.frames;
    r.bytes += f.size;
  });
  if (!opened.Ok()) {
    std::fprintf(stderr, "open video stream failed: %s\n", opened.ErrorValue().error_message.c_str());
    return {};
  }

  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  opened.Value().reset();  // Stop the stream and wait for the delivery thread to exit.

  tracker.Finalize(&r);
  r.elapsed_s = seconds;
  r.fps = r.frames / seconds;
  r.mbps = r.bytes / seconds / 1e6;
  return r;
}

CameraId ParseCameraId(const std::string& key) {
  for (auto id : {CameraId::kHeadRgb, CameraId::kHeadDepth, CameraId::kHeadLeftEye,
                  CameraId::kHeadRightEye, CameraId::kLeftWrist, CameraId::kLeftElbow,
                  CameraId::kRightWrist, CameraId::kRightElbow}) {
    if (key == x2robot::sdk::CameraIdKey(id)) return id;
  }
  std::fprintf(stderr, "Unknown camera ID %s; falling back to left_wrist\n", key.c_str());
  return CameraId::kLeftWrist;
}

std::vector<CameraId> AllCameraIds() {
  return {CameraId::kHeadRgb,     CameraId::kHeadDepth, CameraId::kHeadLeftEye,
          CameraId::kHeadRightEye, CameraId::kLeftWrist, CameraId::kLeftElbow,
          CameraId::kRightWrist,  CameraId::kRightElbow};
}

std::vector<CameraId> ParseCameraList(const std::string& keys) {
  if (keys == "all") return AllCameraIds();
  std::vector<CameraId> ids;
  size_t start = 0;
  while (start <= keys.size()) {
    const size_t comma = keys.find(',', start);
    const std::string key = keys.substr(
        start, comma == std::string::npos ? std::string::npos : comma - start);
    if (!key.empty()) {
      for (auto id : AllCameraIds()) {
        if (key == x2robot::sdk::CameraIdKey(id)) {
          ids.push_back(id);
          break;
        }
      }
    }
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  return ids;
}

// Open several camera streams on the same Robot and measure them independently.
// Statistics are written on each stream's delivery thread and read only after
// that stream is stopped (which joins the thread).
int MultiBench(const std::string& address, const std::string& keys, double seconds) {
  const auto ids = ParseCameraList(keys);
  if (ids.empty()) {
    std::fprintf(stderr, "no known camera in '%s'\n", keys.c_str());
    return 2;
  }
  auto created = Robot::Create(address);
  if (!created.Ok()) {
    std::fprintf(stderr, "connect failed: %s\n", created.ErrorValue().error_message.c_str());
    return 1;
  }
  auto robot = std::move(created.Value());

  struct Stream {
    std::string name;
    BenchResult result;
    LatencyTracker tracker;
    std::unique_ptr<x2robot::sdk::client::VideoStream> stream;
  };
  std::vector<std::unique_ptr<Stream>> streams;
  for (auto id : ids) {
    auto item = std::make_unique<Stream>();
    item->name = x2robot::sdk::CameraIdKey(id);
    auto* raw = item.get();
    auto opened = GetVideoStream(*robot, id, [raw](const SdkResult<CameraFrameView>& frame) {
      if (!frame.Ok()) return;
      const auto& f = frame.Value();
      raw->tracker.Feed(f.data, f.size, &raw->result);
      ++raw->result.frames;
      raw->result.bytes += f.size;
    });
    if (!opened.Ok()) {
      // Cameras missing from the robot profile are expected (e.g. EX001 Orin has
      // no head_rgb).
      std::fprintf(stderr, "  %-22s unavailable: %s\n", raw->name.c_str(),
                   opened.ErrorValue().error_message.c_str());
      continue;
    }
    raw->stream = std::make_unique<x2robot::sdk::client::VideoStream>(std::move(opened.Value()));
    streams.push_back(std::move(item));
  }
  if (streams.empty()) {
    std::fprintf(stderr, "no camera stream could be opened\n");
    return 1;
  }
  std::printf("  %zu stream(s) open, measuring %.0f s\n", streams.size(), seconds);
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  for (auto& item : streams) item->stream.reset();  // stop and join delivery threads

  uint64_t total_frames = 0;
  double total_bytes = 0;
  for (auto& item : streams) {
    item->tracker.Finalize(&item->result);
    item->result.elapsed_s = seconds;
    item->result.fps = static_cast<double>(item->result.frames) / seconds;
    item->result.mbps = static_cast<double>(item->result.bytes) / seconds / 1e6;
    Report(item->name.c_str(), item->result);
    total_frames += item->result.frames;
    total_bytes += static_cast<double>(item->result.bytes);
  }
  std::printf("  %-22s frames %7llu  %.1f fps  %6.2f MB/s  (%zu streams)\n", "ALL STREAMS",
              static_cast<unsigned long long>(total_frames),
              static_cast<double>(total_frames) / seconds, total_bytes / seconds / 1e6,
              streams.size());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string camera_key = argc > 2 ? argv[2] : "left_wrist";
  const double seconds = argc > 3 ? std::atof(argv[3]) : 10.0;

  std::printf("\nCamera stream benchmark (%.0f s)\n\n", seconds);

  if (camera_key == "all" || camera_key.find(',') != std::string::npos) {
    return MultiBench(address, camera_key, seconds);
  }

  const CameraId camera_id = ParseCameraId(camera_key);
  auto grpc_r = Bench(address, camera_id, seconds);
  Report("gRPC stream", grpc_r);

  // Also write the result to a file to avoid losing output to terminal buffering.
  if (const char* out = std::getenv("X2BENCH_OUT")) {
    FILE* f = std::fopen(out, "w");
    if (f) {
      std::fprintf(f, "grpc frames=%llu fps=%.1f mbps=%.2f mean_us=%.1f p99_us=%.1f lost=%llu\n",
                   (unsigned long long)grpc_r.frames, grpc_r.fps, grpc_r.mbps, grpc_r.mean_us,
                   grpc_r.p99_us, (unsigned long long)grpc_r.lost);
      std::fclose(f);
    }
  }
  return 0;
}
