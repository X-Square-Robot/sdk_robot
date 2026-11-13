#pragma once

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace sdk_recording {
namespace fs = std::filesystem;

inline std::string Trim(const std::string& text)
{
    const auto first = text.find_first_not_of(" \t\r\n");
    return first == std::string::npos ? "" : text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}

inline bool ValidId(const std::string& value)
{
    return std::regex_match(value, std::regex("[A-Za-z0-9_-]{1,96}"));
}

inline double Seconds(const std::string& text, bool allow_zero = false)
{
    size_t parsed = 0;
    const double value = std::stod(text, &parsed);
    if (parsed != text.size() || !std::isfinite(value) || value < 0 || (!allow_zero && value == 0) ||
        value > std::numeric_limits<int>::max() / 1000.0)
    {
        throw std::runtime_error("seconds must be finite, " + std::string(allow_zero ? "nonnegative" : "positive") + " and at most 2147483");
    }
    return value;
}

inline void ValidateItems(const std::vector<std::string>& items)
{
    std::set<std::string> unique;
    for (const auto& id : items)
    {
        if (!ValidId(id) || !unique.insert(id).second) throw std::runtime_error("invalid or duplicate item ID: " + id);
    }
    if (unique.empty() || unique.size() > 512) throw std::runtime_error("select between 1 and 512 item IDs");
    if (unique.count("all") && items != std::vector<std::string>{"all"}) throw std::runtime_error("all must be the only item");
}

struct Item
{
    std::string id, description;
    bool enabled;
};
struct Preset
{
    std::string name, model;
    std::vector<Item> items;
};

inline Preset ReadConfig(const fs::path& path)
{
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot read config: " + path.string());
    std::map<std::string, std::map<std::string, std::string>> sections;
    std::vector<std::string> order;
    std::string current, line;
    while (std::getline(input, line))
    {
        line = Trim(line);
        if (line.empty() || line.front() == '#') continue;
        if (line.front() == '[' && line.back() == ']')
        {
            current = line.substr(1, line.size() - 2);
            if (current != "preset" && (current.rfind("item.", 0) != 0 || !ValidId(current.substr(5))))
                throw std::runtime_error("invalid config section: " + current);
            if (!sections.emplace(current, std::map<std::string, std::string>{}).second)
                throw std::runtime_error("duplicate config section: " + current);
            order.push_back(current);
            continue;
        }
        const auto separator = line.find('=');
        if (current.empty() || separator == std::string::npos) throw std::runtime_error("expected a config section and key=value");
        const auto key = Trim(line.substr(0, separator));
        const auto value = Trim(line.substr(separator + 1));
        if (!sections.at(current).emplace(key, value).second) throw std::runtime_error("duplicate config key: " + key);
    }
    if (input.bad() || !sections.count("preset")) throw std::runtime_error("config requires [preset] name/model");
    const auto& header = sections.at("preset");
    if (header.size() != 2 || !header.count("name") || !header.count("model") || !ValidId(header.at("name")) ||
        !std::set<std::string>{"quanta_x1", "quanta_x2", "quanta_x3"}.count(header.at("model")))
        throw std::runtime_error("invalid preset name or model");
    Preset preset{header.at("name"), header.at("model"), {}};
    for (const auto& section : order)
    {
        if (section == "preset") continue;
        const auto& entry = sections.at(section);
        if (entry.size() != 2 || !entry.count("enabled") || !entry.count("description") || entry.at("description").empty() ||
            (entry.at("enabled") != "true" && entry.at("enabled") != "false") || section == "item.all")
            throw std::runtime_error("expected [item.ID] enabled=true/false and description");
        preset.items.push_back({section.substr(5), entry.at("description"), entry.at("enabled") == "true"});
    }
    if (preset.items.empty() || preset.items.size() > 512) throw std::runtime_error("expected between 1 and 512 item definitions");
    return preset;
}

// Prefer resources beside an installed binary, then the source tree used to build it.
// Neither lookup depends on the process working directory.
inline fs::path Resource(const fs::path& relative)
{
    std::error_code error;
    const auto executable = fs::canonical("/proc/self/exe", error);
    const auto source_root = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
    for (const auto& root : {executable.parent_path().parent_path(), source_root})
    {
        const auto candidate = root / relative;
        if (fs::is_regular_file(candidate)) return fs::absolute(candidate);
    }
    throw std::runtime_error("missing bundled resource: " + relative.string());
}

struct Options
{
    std::string address = "127.0.0.1:55051";
    std::string action = "collect";
    std::string session_id;
    fs::path config;
    std::string preset;
    fs::path output_dir = "data_collection_v2";
    std::vector<std::string> items;
    bool explicit_items = false;
    bool raw_only = false;
    bool cleanup_after_download = true;
    bool help = false;
    double duration = 10;
    double finalize_timeout = 600;
    double status_timeout = 30;
    std::string replay_format = "both";
    std::string python = "python3";
};

inline Options ParseOptions(int argc, char** argv)
{
    Options options;
    bool explicit_cleanup = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
            return argv[++i];
        };
        if (arg == "--address")
            options.address = value();
        else if (arg == "--action")
            options.action = value();
        else if (arg == "--session-id")
            options.session_id = value();
        else if (arg == "--config")
            options.config = value();
        else if (arg == "--preset")
            options.preset = value();
        else if (arg == "--output-dir")
            options.output_dir = value();
        else if (arg == "--duration")
            options.duration = Seconds(value(), true);
        else if (arg == "--status-timeout")
            options.status_timeout = Seconds(value());
        else if (arg == "--finalize-timeout")
            options.finalize_timeout = Seconds(value());
        else if (arg == "--replay-format")
            options.replay_format = value();
        else if (arg == "--python")
            options.python = value();
        else if (arg == "--cleanup-after-download" || arg == "--keep-robot-data")
        {
            const bool cleanup = arg == "--cleanup-after-download";
            if (explicit_cleanup && options.cleanup_after_download != cleanup)
                throw std::runtime_error("--cleanup-after-download and --keep-robot-data are mutually exclusive");
            explicit_cleanup = true;
            options.cleanup_after_download = cleanup;
        }
        else if (arg == "--raw-only")
            options.raw_only = true;
        else if (arg == "--help" || arg == "-h")
            options.help = true;
        else if (arg == "--items")
        {
            if (options.explicit_items) throw std::runtime_error("--items specified twice");
            options.explicit_items = true;
            while (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) options.items.push_back(argv[++i]);
            ValidateItems(options.items);
        }
        else
            throw std::runtime_error("unknown option: " + arg);
    }
    if (options.help) return options;
    if (explicit_cleanup && options.action != "collect" && options.action != "download")
        throw std::runtime_error("--cleanup-after-download/--keep-robot-data require collect or download");
    if (!explicit_cleanup) options.cleanup_after_download = options.action == "collect" || options.action == "download";
    if (!std::set<std::string>{"list", "collect", "start", "stop", "status", "download", "delete"}.count(options.action))
    {
        throw std::runtime_error("unknown action: " + options.action);
    }
    if (!std::set<std::string>{"none", "json", "both"}.count(options.replay_format)) throw std::runtime_error("invalid replay format");
    if ((!options.session_id.empty() && !ValidId(options.session_id)) ||
        ((options.action == "stop" || options.action == "download" || options.action == "delete") && options.session_id.empty()))
    {
        throw std::runtime_error("stop/download/delete require a valid --session-id; use the ID printed before start");
    }
    return options;
}
inline fs::path ConfigPath(const Options& options, const std::string& model = "quanta_x1")
{
    if (!options.config.empty()) return options.config;
    const auto preset = options.preset.empty() ? model : options.preset;
    if (!std::set<std::string>{"quanta_x1", "quanta_x1_dual_head_camera", "quanta_x2", "quanta_x3"}.count(preset))
        throw std::runtime_error("unknown preset: " + preset + "; use --config or --items");
    return Resource("examples/cpp/data_collection_v2/config/" + preset + ".conf");
}
}  // namespace sdk_recording
