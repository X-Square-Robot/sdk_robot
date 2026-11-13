#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>

namespace x2robot::examples::linear_toppra {

// TOPP-RA on a two-point linear path has a constant path derivative.  Its
// zero-start/zero-end solution is therefore exactly this bounded scalar
// profile.  This shared profile keeps the C++ examples on the same
// velocity and acceleration limits as Python while remaining self-contained.
class ScalarTrajectory {
 public:
  ScalarTrajectory(double path_length, double velocity_limit,
                   double acceleration_limit)
      : path_length_(path_length),
        velocity_limit_(velocity_limit),
        acceleration_limit_(acceleration_limit) {
    const double distance_to_max_velocity =
        velocity_limit_ * velocity_limit_ / acceleration_limit_;
    acceleration_time_ = velocity_limit_ / acceleration_limit_;
    if (path_length_ >= distance_to_max_velocity) {
      cruise_time_ =
          (path_length_ - distance_to_max_velocity) / velocity_limit_;
      duration_ = 2.0 * acceleration_time_ + cruise_time_;
    } else {
      cruise_time_ = 0.0;
      acceleration_time_ = std::sqrt(path_length_ / acceleration_limit_);
      duration_ = 2.0 * acceleration_time_;
    }
  }

  double duration() const { return duration_; }

  double Position(double timestamp) const {
    const double time = std::clamp(timestamp, 0.0, duration_);
    if (time <= acceleration_time_) {
      return 0.5 * acceleration_limit_ * time * time;
    }
    if (time <= acceleration_time_ + cruise_time_) {
      const double acceleration_distance =
          0.5 * acceleration_limit_ * acceleration_time_ * acceleration_time_;
      return acceleration_distance +
             velocity_limit_ * (time - acceleration_time_);
    }
    const double remaining_time = duration_ - time;
    return path_length_ -
           0.5 * acceleration_limit_ * remaining_time * remaining_time;
  }

 private:
  double path_length_;
  double velocity_limit_;
  double acceleration_limit_;
  double acceleration_time_;
  double cruise_time_;
  double duration_;
};

inline bool MakeTrajectory(double path_length, double velocity_limit,
                    double acceleration_limit, ScalarTrajectory* trajectory) {
  if (!std::isfinite(path_length) || !std::isfinite(velocity_limit) ||
      !std::isfinite(acceleration_limit) || path_length <= 0.0 ||
      velocity_limit <= 0.0 || acceleration_limit <= 0.0) {
    std::cerr << "invalid TOPP-RA trajectory parameters\n";
    return false;
  }
  *trajectory =
      ScalarTrajectory(path_length, velocity_limit, acceleration_limit);
  return std::isfinite(trajectory->duration()) && trajectory->duration() > 0.0;
}


} // namespace x2robot::examples::linear_toppra
