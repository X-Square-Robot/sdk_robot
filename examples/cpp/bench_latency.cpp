// Latency benchmark: measures the round-trip time of individual gRPC calls.
// Usage: ./bench_latency [host:port] [iterations]

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <numeric>
#include <string>
#include <vector>

#include "x2robot/sdk/robot.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct Stats {
  double mean{0}, p50{0}, p90{0}, p99{0}, max{0};
};

Stats Summarize(std::vector<double> samples) {
  std::sort(samples.begin(), samples.end());
  const auto n = samples.size();
  auto pct = [&](double p) { return samples[std::min(n - 1, static_cast<size_t>(p * n))]; };
  Stats s;
  s.mean = std::accumulate(samples.begin(), samples.end(), 0.0) / static_cast<double>(n);
  s.p50 = pct(0.50);
  s.p90 = pct(0.90);
  s.p99 = pct(0.99);
  s.max = samples.back();
  return s;
}

void PrintRow(const char* transport, const char* method, const Stats& s) {
  std::printf("  %-6s %-16s %9.1f %9.1f %9.1f %9.1f %10.1f\n", transport, method, s.mean, s.p50,
              s.p90, s.p99, s.max);
}

bool Measure(const std::string& address, const char* label, int iters) {
  auto robot_r = x2robot::sdk::Robot::Create(address);
  if (!robot_r.Ok()) {
    std::fprintf(stderr, "connect failed: %s\n", robot_r.ErrorValue().error_message.c_str());
    return false;
  }
  auto robot = std::move(robot_r.Value());
  auto arm = robot->GetLeftArmController();

  x2robot::sdk::Pose pose;
  pose.mutable_position()->set_x(0.4);
  pose.mutable_position()->set_y(0.2);
  pose.mutable_position()->set_z(1.0);
  pose.mutable_orientation()->set_w(1.0);

  const int warmup = std::max(50, iters / 10);
  for (int i = 0; i < warmup; ++i) {
    auto set_result = arm.SetEndPose(pose);
    auto get_result = arm.GetJointStates();
    if (!set_result.Ok() || !get_result.Ok()) {
      std::fprintf(stderr, "warmup failed at iter %d set=%s get=%s\n", i,
                   set_result.Ok() ? "ok" : set_result.ErrorValue().error_message.c_str(),
                   get_result.Ok() ? "ok" : get_result.ErrorValue().error_message.c_str());
      (void)robot->Close();
      return false;
    }
  }

  std::vector<double> set_us;
  std::vector<double> get_us;
  set_us.reserve(iters);
  get_us.reserve(iters);

  for (int i = 0; i < iters; ++i) {
    pose.mutable_position()->set_x(0.4 + i * 1e-6);

    auto t0 = Clock::now();
    auto r1 = arm.SetEndPose(pose);
    auto t1 = Clock::now();
    auto r2 = arm.GetJointStates();
    auto t2 = Clock::now();

    if (!r1.Ok() || !r2.Ok()) {
      std::fprintf(stderr, "call failed at iter %d set=%s get=%s\n", i,
                   r1.Ok() ? "ok" : r1.ErrorValue().error_message.c_str(),
                   r2.Ok() ? "ok" : r2.ErrorValue().error_message.c_str());
      (void)robot->Close();
      return false;
    }
    set_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    get_us.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
  }

  PrintRow(label, "set_end_pose", Summarize(std::move(set_us)));
  PrintRow(label, "get_joint_states", Summarize(std::move(get_us)));
  auto closed = robot->Close();
  if (!closed.Ok()) {
    std::fprintf(stderr, "close failed: %s\n", closed.ErrorValue().error_message.c_str());
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    std::printf("usage: %s [address] [iterations]\n", argv[0]);
    return 0;
  }
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const int iters = argc > 2 ? std::atoi(argv[2]) : 2000;

  std::printf("\nRound-trip latency per call (us), %d iterations per method\n\n", iters);
  std::printf("  %-6s %-16s %9s %9s %9s %9s %10s\n", "transport", "method", "mean", "p50",
              "p90", "p99", "max");
  std::printf("  %s\n", std::string(72, '-').c_str());

  const bool grpc_ok = Measure(address, "grpc", iters);
  std::printf("\n");

  return grpc_ok ? 0 : 1;
}
