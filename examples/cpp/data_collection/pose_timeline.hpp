#pragma once

#include <x2robot/sdk/robot.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>
#include <utility>
#include <vector>

namespace sdk_examples {

// Same linear-position / shortest-path SLERP alignment as the Python collector.
class ReplayPoseTimeline {
 public:
  bool Add(double stamp, geometry_msgs::Pose pose) {
    const auto& p = pose.position();
    auto* q = pose.mutable_orientation();
    if (!std::isfinite(stamp) || !std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()) ||
        !std::isfinite(q->x()) || !std::isfinite(q->y()) || !std::isfinite(q->z()) || !std::isfinite(q->w())) {
      return false;
    }
    const double norm = std::sqrt(q->x() * q->x() + q->y() * q->y() + q->z() * q->z() + q->w() * q->w());
    if (!std::isfinite(norm) || norm < 1e-12) return false;
    q->set_x(q->x() / norm);
    q->set_y(q->y() / norm);
    q->set_z(q->z() / norm);
    q->set_w(q->w() / norm);
    samples_.push_back({stamp, std::move(pose)});
    return true;
  }

  geometry_msgs::Pose At(double stamp) const {
    const auto next = std::upper_bound(samples_.begin(), samples_.end(), stamp,
        [](double value, const Sample& sample) { return value < sample.stamp; });
    if (next == samples_.begin()) return samples_.front().pose;
    if (next == samples_.end()) return samples_.back().pose;
    const auto& before = *(next - 1);
    const double ratio = (stamp - before.stamp) / (next->stamp - before.stamp);
    geometry_msgs::Pose result;
    auto* p = result.mutable_position();
    p->set_x(before.pose.position().x() + ratio * (next->pose.position().x() - before.pose.position().x()));
    p->set_y(before.pose.position().y() + ratio * (next->pose.position().y() - before.pose.position().y()));
    p->set_z(before.pose.position().z() + ratio * (next->pose.position().z() - before.pose.position().z()));
    const auto& qa = before.pose.orientation();
    const auto& qb = next->pose.orientation();
    std::array<double, 4> a{qa.x(), qa.y(), qa.z(), qa.w()}, b{qb.x(), qb.y(), qb.z(), qb.w()}, q{};
    double dot = 0;
    for (size_t i = 0; i < 4; ++i) dot += a[i] * b[i];
    if (dot < 0) {
      for (auto& value : b) value = -value;
      dot = -dot;
    }
    double wa = 1 - ratio, wb = ratio;
    if (dot <= 0.9995) {
      const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
      wa = std::sin((1 - ratio) * theta) / std::sin(theta);
      wb = std::sin(ratio * theta) / std::sin(theta);
    }
    double norm = 0;
    for (size_t i = 0; i < 4; ++i) {
      q[i] = wa * a[i] + wb * b[i];
      norm += q[i] * q[i];
    }
    norm = std::sqrt(norm);
    auto* orientation = result.mutable_orientation();
    orientation->set_x(q[0] / norm);
    orientation->set_y(q[1] / norm);
    orientation->set_z(q[2] / norm);
    orientation->set_w(q[3] / norm);
    return result;
  }

 private:
  struct Sample {
    double stamp;
    geometry_msgs::Pose pose;
  };
  std::vector<Sample> samples_;  // LoadEpisode supplies timestamp-sorted records.
};

inline int ReplayPoseIndex(std::string_view name) {
  if (name == "left_arm_end_pose") return 0;
  if (name == "right_arm_end_pose") return 1;
  if (name == "waist_end_pose") return 2;
  return -1;
}

}  // namespace sdk_examples
