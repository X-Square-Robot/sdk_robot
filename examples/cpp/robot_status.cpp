// Counterpart of python/v1/robot_status.py.  RobotStatus is intentionally a
// JSON string in the public v2 contract; this example prints the same logical
// sections while keeping parsing dependency-free for the standalone artifact.
#include "x2robot/sdk/robot.hpp"

#include <chrono>
#include <initializer_list>
#include <iostream>
#include <string>
#include <cctype>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  const auto& e = result.ErrorValue();
  std::cerr << operation << " failed [" << e.error_name << "]: " << e.error_message << '\n';
  return false;
}

std::string JsonValue(const std::string& json, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  std::size_t cursor = 0;
  while ((cursor = json.find(needle, cursor)) != std::string::npos) {
    std::size_t colon = cursor + needle.size();
    while (colon < json.size() && std::isspace(static_cast<unsigned char>(json[colon]))) ++colon;
    if (colon >= json.size() || json[colon] != ':') { cursor += needle.size(); continue; }
    std::size_t begin = colon + 1;
    while (begin < json.size() && std::isspace(static_cast<unsigned char>(json[begin]))) ++begin;
    if (begin >= json.size()) return "null";
    if (json[begin] == '{' || json[begin] == '[') {
      const char open = json[begin], close = open == '{' ? '}' : ']';
      int depth = 0; bool quoted = false, escaped = false;
      for (std::size_t i = begin; i < json.size(); ++i) {
        const char ch = json[i];
        if (quoted) {
          if (escaped) escaped = false;
          else if (ch == '\\') escaped = true;
          else if (ch == '\"') quoted = false;
          continue;
        }
        if (ch == '\"') { quoted = true; continue; }
        if (ch == open) ++depth;
        else if (ch == close && --depth == 0) return json.substr(begin, i - begin + 1);
      }
      return "null";
    }
    if (json[begin] == '\"') {
      bool escaped = false;
      for (std::size_t i = begin + 1; i < json.size(); ++i) {
        if (escaped) escaped = false;
        else if (json[i] == '\\') escaped = true;
        else if (json[i] == '\"') return json.substr(begin, i - begin + 1);
      }
      return "null";
    }
    std::size_t end = begin;
    while (end < json.size() && json[end] != ',' && json[end] != '}' && json[end] != ']') ++end;
    while (end > begin && std::isspace(static_cast<unsigned char>(json[end - 1]))) --end;
    return json.substr(begin, end - begin);
  }
  return "null";
}

std::string JsonObject(const std::string& json, const std::string& key) {
  const std::string value = JsonValue(json, key);
  return !value.empty() && value.front() == '{' ? value : "{}";
}

void PrintSection(const std::string& json, const char* title,
                  std::initializer_list<const char*> fields) {
  std::cout << "\n=== " << title << " ===\n";
  for (const char* field : fields) std::cout << field << ": " << JsonValue(json, field) << '\n';
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  ::xr::sdk::v2::GetRobotStatusRequest request;
  auto result = robot->GetRobotStatusService().GetRobotStatus(request, std::chrono::seconds{5});
  bool ok = check(result, "RobotStatus.GetRobotStatus");
  if (ok) {
    const auto& json = result.Value();
    PrintSection(JsonObject(json, "energy"), "energy", {"battery_level", "is_charging", "battery_charging_method",
                                   "battery_temperature", "est_runtime_left", "has_power_fault"});
    PrintSection(JsonObject(json, "motion"), "motion", {"localization", "current_pose", "is_moving", "linear_velocity",
                                   "angular_velocity", "is_navigating", "navigation_target",
                                   "distance_to_goal", "is_path_blocked", "nearest_obstacle_distance",
                                   "blocked_reason"});
    PrintSection(JsonObject(json, "execution"), "execution", {"operation_mode", "control_owner", "is_speaking",
                                      "audio_output_volume", "arm_state_left", "arm_state_right",
                                      "end_effector_pose", "gripper_force", "gripper_width",
                                      "is_holding_object", "system_uptime"});
    PrintSection(JsonObject(json, "safety"), "safety", {"emergency_stop_active", "safety_still", "fault_alerts",
                                   "component_health"});
    PrintSection(JsonObject(json, "health"), "health", {"battery_soh", "network_connected", "network_latency",
                                   "cpu_usage", "memory_usage", "disk_usage", "active_error_count",
                                   "highest_error_level", "active_error_codes"});
    std::cout << "\nrobot_status_json=" << json << '\n';
  }
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
