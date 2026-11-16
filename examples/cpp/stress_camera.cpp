// Camera client concurrency stress test: repeatedly starts and stops streams from multiple
// threads, verifies that no deadlock or crash occurs, and checks that the server's stream
// accounting returns to zero (the server log should show no remaining streams after the test).
//
// Usage: ./stress_camera [address] [rounds]

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "x2robot/sdk/robot.hpp"

using namespace std::chrono_literals;
using x2robot::sdk::CameraId;
using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using x2robot::sdk::client::CameraFrameView;

int main(int argc, char** argv) {
  if (argc > 1 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    std::printf("Usage: %s [address] [rounds]\n", argv[0]);
    return 0;
  }
  std::string address = "127.0.0.1:55051";
  int rounds = 20;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a.find_first_not_of("0123456789") == std::string::npos) {
      rounds = std::atoi(a.c_str());
    } else {
      address = a;
    }
  }

  auto created = Robot::Create(address);
  if (!created.Ok()) {
    std::printf("Connection failed: %s\n", created.ErrorValue().error_message.c_str());
    return 1;
  }
  auto robot = std::move(created.Value());
  const bool x3 = robot->ModelName() == "quanta_x3";

  // 两路相机独占启停，一路相机测试并发订阅，左腕相机测试单帧采集。
  // X3 使用已公开的双眼视频流，其余机型保留原有 RGB/深度接口。
  std::atomic<uint64_t> frames{0};
  std::atomic<uint64_t> opens{0}, open_fail{0}, images{0}, image_fail{0};
  std::atomic<uint64_t> shared_opens{0}, shared_fail{0};

  std::vector<std::thread> workers;

  // Repeatedly start and stop streams, with one camera assigned to each worker.
  for (int camera_index = 0; camera_index < 2; ++camera_index) {
    workers.emplace_back([&, camera_index] {
      for (int i = 0; i < rounds; ++i) {
        auto cb = [&](const SdkResult<CameraFrameView>& frame) {
          if (frame.Ok()) frames.fetch_add(1, std::memory_order_relaxed);
        };
        auto r = camera_index == 0 ? (x3 ? robot->GetHeadCamera().GetRightEyeVideoStream(cb)
                                        : robot->GetHeadCamera().GetDepthVideoStream(cb))
                                   : robot->GetRightArmCamera().GetVideoStream(cb);
        if (r.Ok()) {
          opens.fetch_add(1, std::memory_order_relaxed);
        } else {
          open_fail.fetch_add(1, std::memory_order_relaxed);
          std::printf("  Failed to start exclusive stream: %s %s\n",
                      r.ErrorValue().error_name.c_str(),
                      r.ErrorValue().error_message.c_str());
        }
        std::this_thread::sleep_for(30ms);
        // The stream stops when r goes out of scope.
      }
    });
  }

  // Start streams from the same camera on three threads. This is supported by the handle model,
  // where each stream receives its own frames, so all operations must succeed. This also stresses
  // the window where stream startup and shutdown overlap.
  for (int t = 0; t < 3; ++t) {
    workers.emplace_back([&] {
      for (int i = 0; i < rounds; ++i) {
        auto callback = [&](const SdkResult<CameraFrameView>& frame) {
          if (frame.Ok()) frames.fetch_add(1, std::memory_order_relaxed);
        };
        auto r = x3 ? robot->GetHeadCamera().GetLeftEyeVideoStream(callback)
                    : robot->GetHeadCamera().GetRGBVideoStream(callback);
        if (r.Ok()) {
          shared_opens.fetch_add(1, std::memory_order_relaxed);
        } else {
          shared_fail.fetch_add(1, std::memory_order_relaxed);
          std::printf("  Failed to start concurrent stream for the same camera: %s %s\n",
                      r.ErrorValue().error_name.c_str(),
                      r.ErrorValue().error_message.c_str());
        }
        std::this_thread::sleep_for(2ms);
      }
    });
  }

  // Capture single frames. Internally, this starts a stream, waits for the first frame, and then
  // stops the stream. It has exclusive access to one camera, so every capture must succeed.
  workers.emplace_back([&] {
    for (int i = 0; i < rounds; ++i) {
      if (x3) {
        // X3 仅公开视频流：收到一帧后关闭，不能调用未公开的单图接口。
        std::mutex mutex;
        std::condition_variable condition;
        bool done = false;
        bool captured = false;
        auto sample = robot->GetLeftArmCamera().GetVideoStream(
            [&](const SdkResult<CameraFrameView>& frame) {
              std::lock_guard lock(mutex);
              captured = captured || frame.Ok();
              done = true;
              condition.notify_all();
            }, 1000ms);
        std::unique_lock lock(mutex);
        if (sample.Ok()) condition.wait_for(lock, 1000ms, [&] { return done; });
        if (sample.Ok() && captured) {
          images.fetch_add(1, std::memory_order_relaxed);
        } else {
          image_fail.fetch_add(1, std::memory_order_relaxed);
          std::printf("  Failed to capture a video frame\n");
        }
        continue;
      }
      auto r = robot->GetLeftArmCamera().GetRawImage(
          [&](const SdkResult<CameraFrameView>&) {}, std::chrono::milliseconds{1000});
      if (r.Ok()) {
        images.fetch_add(1, std::memory_order_relaxed);
      } else {
        image_fail.fetch_add(1, std::memory_order_relaxed);
        std::printf("  Failed to capture a single frame: %s %s\n",
                    r.ErrorValue().error_name.c_str(),
                    r.ErrorValue().error_message.c_str());
      }
    }
  });

  for (auto& w : workers) w.join();

  const auto expect_opens = static_cast<uint64_t>(rounds) * 2;
  const auto expect_shared = static_cast<uint64_t>(rounds) * 3;
  const auto expect_images = static_cast<uint64_t>(rounds);
  std::printf("\nExclusive streams %llu/%llu, concurrent same-camera streams %llu/%llu, "
              "single-frame captures %llu/%llu, total frames received %llu\n",
              (unsigned long long)opens.load(), (unsigned long long)expect_opens,
              (unsigned long long)shared_opens.load(), (unsigned long long)expect_shared,
              (unsigned long long)images.load(), (unsigned long long)expect_images,
              (unsigned long long)frames.load());

  const bool pass = opens.load() == expect_opens && shared_opens.load() == expect_shared &&
                    images.load() == expect_images && frames.load() > 0 && !open_fail &&
                    !shared_fail && !image_fail;
  std::printf("%s\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
