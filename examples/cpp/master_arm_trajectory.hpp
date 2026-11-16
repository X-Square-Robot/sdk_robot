#pragma once

#include "linear_toppra_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace x2robot::examples::master_arm_trajectory {

// Reuse the slave-arm C++ implementation of the two-point TOPPRA path.
// A requested minimum duration only stretches time and reduces derivatives.
class Profile {
public:
    Profile(double distance, double velocity_limit, double acceleration_limit,
            double minimum_seconds)
        : distance_(distance)
    {
        if (!std::isfinite(distance) || distance < 0.0 ||
            !std::isfinite(velocity_limit) || velocity_limit <= 0.0 ||
            !std::isfinite(acceleration_limit) || acceleration_limit <= 0.0 ||
            !std::isfinite(minimum_seconds) || minimum_seconds <= 0.0) return;
        if (distance == 0.0) {
            duration_ = minimum_seconds;
        } else if (linear_toppra::MakeTrajectory(distance, velocity_limit,
                                                acceleration_limit, &base_)) {
            duration_ = std::max(minimum_seconds, base_.duration());
        }
    }

    double duration() const { return duration_; }

    double Progress(double time_fraction) const
    {
        if (!std::isfinite(duration_)) return std::numeric_limits<double>::quiet_NaN();
        if (distance_ == 0.0) return 0.0;
        return std::clamp(base_.Position(std::clamp(time_fraction, 0.0, 1.0) * base_.duration()) /
                              distance_, 0.0, 1.0);
    }

private:
    double distance_;
    double duration_ = std::numeric_limits<double>::quiet_NaN();
    linear_toppra::ScalarTrajectory base_{1.0, 1.0, 1.0};
};

}  // namespace x2robot::examples::master_arm_trajectory
