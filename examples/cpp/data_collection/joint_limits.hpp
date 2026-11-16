#pragma once

#include "../data_collection_v2/options.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

namespace sdk_examples {

inline bool ReplayJointStream(const std::string& stream, bool joint_mode)
{
    return stream == "head_joint_states" || stream == "lift_joint_states" ||
           (joint_mode && (stream == "left_arm_joint_states" || stream == "right_arm_joint_states" || stream == "waist_joint_states"));
}

class ReplayJointLimits
{
   public:
    explicit ReplayJointLimits(std::string model, std::filesystem::path path = {}) : model_(std::move(model))
    {
        if (path.empty()) path = sdk_recording::Resource("examples/config/replay_joint_limits.conf");
        std::ifstream input(path);
        if (!input) throw std::runtime_error("cannot read joint limits: " + path.string());
        std::map<std::string, std::map<std::string, std::string>> sections;
        std::string section, line;
        while (std::getline(input, line))
        {
            line = sdk_recording::Trim(line);
            if (line.empty() || line.front() == '#' || line.front() == ';') continue;
            if (line.front() == '[' && line.back() == ']')
            {
                section = line.substr(1, line.size() - 2);
                if (section.empty() || !sections.emplace(section, std::map<std::string, std::string>{}).second)
                    throw std::runtime_error("invalid/duplicate joint limits section: " + section);
                continue;
            }
            const auto equal = line.find('=');
            if (section.empty() || equal == std::string::npos) throw std::runtime_error("expected joint limits section and key=value");
            if (!sections.at(section).emplace(sdk_recording::Trim(line.substr(0, equal)), sdk_recording::Trim(line.substr(equal + 1))).second)
                throw std::runtime_error("duplicate joint limits key");
        }
        if (input.bad() || sections.empty()) throw std::runtime_error("empty/unreadable joint limits");
        for (const auto& [name, entry] : sections)
        {
            if (entry.size() != 3 || !entry.count("lower") || !entry.count("upper") || !entry.count("wire") ||
                (entry.at("wire") != "float32" && entry.at("wire") != "float64"))
                throw std::runtime_error("invalid joint limit configuration: " + name);
            const auto lower = Numbers(entry.at("lower")), upper = Numbers(entry.at("upper"));
            if (lower.empty() || lower.size() != upper.size()) throw std::runtime_error("invalid joint bound dimensions: " + name);
            auto& bounds = bounds_[name];
            for (size_t i = 0; i < lower.size(); ++i)
            {
                double lo = lower[i], hi = upper[i];
                if (lo > hi) throw std::runtime_error("reversed joint bounds: " + name);
                if (entry.at("wire") == "float32")
                {
                    lo = FloatInside(lo, true);
                    hi = FloatInside(hi, false);
                }
                if (lo > hi) throw std::runtime_error("joint interval has no representable command: " + name);
                bounds.emplace_back(lo, hi);
            }
        }
    }

    std::vector<double> Clamp(const std::string& stream, const std::vector<double>& values) const
    {
        const auto found = bounds_.find(model_ + '.' + stream);
        if (found == bounds_.end()) throw std::runtime_error("No joint command limits for model=" + model_ + " stream=" + stream);
        if (values.size() != found->second.size())
            throw std::runtime_error(stream + ": expected " + std::to_string(found->second.size()) + " finite joint positions");
        std::vector<double> result;
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (!std::isfinite(values[i])) throw std::runtime_error(stream + ": joint positions must be finite");
            result.push_back(std::clamp(values[i], found->second[i].first, found->second[i].second));
        }
        return result;
    }

    const std::string& Model() const { return model_; }

   private:
    static std::vector<double> Numbers(const std::string& text)
    {
        std::vector<double> values;
        std::istringstream input(text);
        std::string field;
        while (std::getline(input, field, ','))
        {
            field = sdk_recording::Trim(field);
            size_t parsed = 0;
            const auto value = std::stod(field, &parsed);
            if (parsed != field.size() || !std::isfinite(value)) throw std::runtime_error("joint bounds must be finite numbers");
            values.push_back(value);
        }
        if (text.empty() || text.back() == ',') throw std::runtime_error("empty joint bound");
        return values;
    }

    static double FloatInside(double value, bool lower)
    {
        float rounded = static_cast<float>(value);
        if (!std::isfinite(rounded)) throw std::runtime_error("joint bound is not representable as float32");
        if ((lower && rounded < value) || (!lower && rounded > value))
            rounded = std::nextafter(rounded, lower ? std::numeric_limits<float>::infinity() : -std::numeric_limits<float>::infinity());
        return rounded;
    }

    std::string model_;
    std::map<std::string, std::vector<std::pair<double, double>>> bounds_;
};
}  // namespace sdk_examples
