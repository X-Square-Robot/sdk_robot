#pragma once

// Command line contract for the interactive C++ collection example.
//
// The historical parser only accepted ``--seconds/--once/--binocular`` from
// argv[4] onwards and treated argv[2] as the output directory, so the Orin
// invocation ``data_collection_example <ip> --binocular`` silently recorded
// into a directory literally named ``--binocular`` (and the head camera the
// EX001 Orin does not publish was subscribed instead of the eye streams).
// Options are therefore parsed anywhere on the command line here; the only
// positional arguments are [address] [output-dir] [model].  Keeping the parser
// in a pure-standard-library header lets the offline test gate pin the
// behaviour without a robot or the SDK.

#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

namespace sdk_examples {

struct DataCollectionOptions {
  std::string address{"127.0.0.1:55051"};
  std::string output{"./collected_data"};
  std::string model{"auto"};
  bool binocular{false};
  bool no_master_arm_data{false};
  bool once{false};
  int seconds{0};
  bool help{false};
};

namespace collection_cli {

inline bool IsOption(std::string_view value) {
  return value.size() >= 2 && value[0] == '-' && value[1] == '-';
}

inline bool ParseInt(std::string_view text, int* value) {
  std::string buffer(text);
  char* end = nullptr;
  const long parsed = std::strtol(buffer.c_str(), &end, 10);
  if (end == buffer.c_str() || *end != '\0') return false;
  *value = static_cast<int>(parsed);
  return true;
}

}  // namespace collection_cli

// Returns false and fills ``error`` for an unknown option, a missing option
// value, or more than three positional arguments.
inline bool ParseDataCollectionArgs(int argc, char** argv, DataCollectionOptions* options,
                                    std::string* error) {
  if (options == nullptr) return false;
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) *error = message;
    return false;
  };
  std::vector<std::string> positional;
  bool options_ended = false;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index] == nullptr ? std::string() : std::string(argv[index]);
    if (!options_ended && arg == "--") {
      options_ended = true;
      continue;
    }
    if (!options_ended && (arg == "-h" || arg == "--help")) {
      options->help = true;
      continue;
    }
    if (!options_ended && arg == "--binocular") {
      options->binocular = true;
      continue;
    }
    if (!options_ended && arg == "--no-master-arm-data") {
      options->no_master_arm_data = true;
      continue;
    }
    if (!options_ended && arg == "--once") {
      options->once = true;
      continue;
    }
    if (!options_ended && arg == "--seconds") {
      if (index + 1 >= argc) return fail("--seconds requires a value");
      if (!collection_cli::ParseInt(argv[++index] == nullptr ? "" : argv[index], &options->seconds) ||
          options->seconds <= 0) {
        return fail("--seconds expects a positive integer");
      }
      continue;
    }
    if (!options_ended && arg.rfind("--seconds=", 0) == 0) {
      if (!collection_cli::ParseInt(arg.substr(10), &options->seconds) || options->seconds <= 0) {
        return fail("--seconds expects a positive integer");
      }
      continue;
    }
    if (!options_ended && collection_cli::IsOption(arg)) return fail("unknown option: " + arg);
    positional.push_back(arg);
  }
  if (positional.size() > 3) {
    return fail("too many positional arguments (expected [server] [output-dir] [auto|x1|x2|desktop])");
  }
  if (!positional.empty()) options->address = positional[0];
  if (positional.size() > 1) options->output = positional[1];
  if (positional.size() > 2) options->model = positional[2];
  return true;
}

}  // namespace sdk_examples
