// Counterpart of python/v1/tf.py.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using TFMessage = ::tf2_msgs::TFMessage;
using namespace std::chrono_literals;
namespace { std::atomic_bool stop{false}; void OnSignal(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); } }

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  const auto& e = result.ErrorValue();
  std::cerr << operation << " failed [" << e.error_name << "]: " << e.error_message << '\n';
  return false;
}

std::string Format(const ::geometry_msgs::TransformStamped& value) {
  const auto& t = value.transform().translation();
  const auto& q = value.transform().rotation();
  return value.header().frame_id() + " -> " + value.child_frame_id() + " t=(" +
         std::to_string(t.x()) + ", " + std::to_string(t.y()) + ", " + std::to_string(t.z()) +
         ") q=(" + std::to_string(q.x()) + ", " + std::to_string(q.y()) + ", " +
         std::to_string(q.z()) + ", " + std::to_string(q.w()) + ")";
}

void PrintFrame(const TFMessage& msg, const char* label, std::map<std::string, std::string>* edges = nullptr) {
  for (const auto& value : msg.transforms()) {
    std::cout << '[' << label << "] " << Format(value) << '\n';
    if (edges) (*edges)[value.child_frame_id()] = value.header().frame_id();
  }
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string action = argc > 2 ? argv[2] : "static";
  std::signal(SIGINT, OnSignal);
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  auto bridge = robot->GetTfBridge();
  bool ok = true;
  std::mutex frame_mutex;
  auto collect = [&](bool is_static, std::chrono::seconds duration,
                     std::map<std::string, std::string>* edges) {
    auto opened = is_static ? bridge.SubscribeTfStatic(5s) : bridge.SubscribeTf(5s);
    if (!check(opened, is_static ? "TfBridge.SubscribeTfStatic" : "TfBridge.SubscribeTf")) return false;
    auto stream = std::move(opened.Value());
    const auto until = std::chrono::steady_clock::now() + duration;
    bool received = false;
    bool success = true;
    while (!stop.load() &&
           (duration.count() <= 0 || std::chrono::steady_clock::now() < until)) {
      auto wait = 2000ms;
      if (duration.count() > 0) {
        wait = std::min(wait, std::chrono::ceil<std::chrono::milliseconds>(
                                  until - std::chrono::steady_clock::now()));
        if (wait.count() <= 0) break;
      }
      auto frame = stream->Read(wait);
      if (!frame.Ok()) {
        // A bounded dynamic collection ends at its deadline. Other read
        // failures, including a timeout before any data, must reach main.
        const bool window_complete = !is_static && received && duration.count() > 0 &&
            std::chrono::steady_clock::now() >= until &&
            frame.ErrorValue().error_name == "TIMEOUT";
        if (!window_complete) success = check(frame, "TF stream read");
        break;
      }
      if (frame.Value().transforms_size() == 0) {
        std::cerr << "TF stream returned an empty transform set\n";
        success = false;
        break;
      }
      received = true;
      if (edges) {
        std::lock_guard<std::mutex> lock(frame_mutex);
        PrintFrame(frame.Value(), is_static ? "static" : "tf", edges);
      } else {
        PrintFrame(frame.Value(), is_static ? "static" : "tf", nullptr);
      }
      // Static TF is a snapshot: another update is not required. This
      // displays the first non-empty message, not a discovery of all publishers.
      if (is_static) break;
    }
    success = check(stream->Cancel(), "TF stream cancel") && success;
    return success && received;
  };
  if (action == "static") {
    std::cout << "Reading one static transform snapshot...\n"; ok = collect(true, 3s, nullptr);
  } else if (action == "stream") {
    std::cout << "Streaming dynamic transforms (Ctrl+C to stop)...\n"; ok = collect(false, std::chrono::seconds{0}, nullptr);
  } else if (action == "tree") {
    std::cout << "Building TF tree from /tf_static + /tf (collecting 3s)...\n";
    std::map<std::string, std::string> edges;
    bool static_ok = false;
    bool dynamic_ok = false;
    std::thread a([&] { static_ok = collect(true, 3s, &edges); });
    std::thread b([&] { dynamic_ok = collect(false, 3s, &edges); });
    a.join(); b.join();
    ok = static_ok && dynamic_ok;
    if (edges.empty()) std::cout << "No transforms received.\n";
    std::map<std::string, std::vector<std::string>> children;
    for (const auto& [child, parent] : edges) children[parent].push_back(child);
    std::map<std::string, bool> has_parent;
    for (const auto& [child, parent] : edges) has_parent[child] = true;
    // Render every descendant (the Python v1 example prints the complete
    // tree, rather than stopping after one level).
    std::function<void(const std::string&, const std::string&)> render =
        [&](const std::string& frame, const std::string& prefix) {
          auto it = children.find(frame);
          if (it == children.end()) return;
          auto list = it->second;
          std::sort(list.begin(), list.end());
          for (std::size_t i = 0; i < list.size(); ++i) {
            const bool last = i + 1 == list.size();
            std::cout << prefix << (last ? "└── " : "├── ") << list[i] << '\n';
            render(list[i], prefix + (last ? "    " : "│   "));
          }
        };
    for (const auto& [parent, list] : children) {
      if (!has_parent[parent]) {
        std::cout << parent << '\n';
        render(parent, "");
      }
    }
    std::cout << "Total frames: " << edges.size() << '\n';
  } else { std::cerr << "Valid actions: static, stream, tree\n"; ok = false; }
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
