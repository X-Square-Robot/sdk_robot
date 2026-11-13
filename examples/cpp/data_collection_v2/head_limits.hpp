#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "data_collection/replay.hpp"

namespace sdk_examples::v2 {

// SDK head command limits (radians). Keep aligned with the per-model server
// profiles and the Python v2 replay. Never rewrite the original recording.
inline std::array<double, 4> HeadLimits(const std::string& model)
{
    if (model == "quanta_x1") return {-0.06, 0.9, -1.20, 1.20};
    if (model == "quanta_x2") return {-0.52, 0.87, -1.57, 1.57};
    if (model == "quanta_x3") return {-0.523599, 0.261799, -1.570796, 1.570796};
    throw std::runtime_error("No head command limits for model: " + model);
}

// Validate the complete head stream before SDK mode is entered. Only payloads
// in this in-memory v2 episode are changed; the shared/v1 replayer is untouched.
inline void PrepareHeadCommands(ReplayEpisode& episode, const std::string& model)
{
    uint64_t records = 0, clipped_records = 0, pitch_clipped = 0, yaw_clipped = 0;
    for (auto& record : episode.records)
    {
        if (record.stream != "head_joint_states") continue;
        const auto limits = HeadLimits(model);
        sensor_msgs::JointState state;
        if (!state.ParseFromString(record.payload) || state.position_size() != 2 ||
            !std::isfinite(state.position(0)) || !std::isfinite(state.position(1)))
            throw std::runtime_error("Head replay requires two finite positions: pitch, yaw");
        ++records;
        const double pitch = std::clamp(state.position(0), limits[0], limits[1]);
        const double yaw = std::clamp(state.position(1), limits[2], limits[3]);
        const bool pitch_changed = pitch != state.position(0), yaw_changed = yaw != state.position(1);
        pitch_clipped += pitch_changed;
        yaw_clipped += yaw_changed;
        if (pitch_changed || yaw_changed)
        {
            ++clipped_records;
            state.set_position(0, pitch);
            state.set_position(1, yaw);
            record.payload = state.SerializeAsString();
        }
    }
    std::cout << "Head clamp plan: model=" << model << " records=" << records << " clamped_records=" << clipped_records
              << " pitch_clamped=" << pitch_clipped << " yaw_clamped=" << yaw_clipped;
    if (records)
    {
        const auto limits = HeadLimits(model);
        std::cout << " pitch_limits_rad=[" << limits[0] << ',' << limits[1] << "] yaw_limits_rad=[" << limits[2] << ','
                  << limits[3] << ']';
    }
    std::cout << std::endl;
}
}  // namespace sdk_examples::v2
