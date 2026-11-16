// Audio counterpart of examples/python/v1/audio.py.
//
// The Python facade accepts either a local file or an URL and hides the wire
// message.  The C++ API intentionally exposes the generated request, so this
// example performs that small conversion here and keeps each action explicit.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"

#include <chrono>
#include <cstdint>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using AudioDataStamped = ::xr::sdk::v2::AudioDataStamped;

namespace {
void signal_handler(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }
}

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  const auto& error = result.ErrorValue();
  std::cerr << operation << " failed [" << error.error_name << "]: "
            << error.error_message << " (request_id=" << result.request_id << ")\n";
  return false;
}

std::string Extension(std::string path) {
  const auto slash = path.find_last_of("/\\");
  if (slash != std::string::npos) path.erase(0, slash + 1);
  const auto dot = path.find_last_of('.');
  return dot == std::string::npos ? "" : path.substr(dot + 1);
}

void PrintPlay(const ::xr::sdk::v2::PlayAudioResponse& response) {
  std::cout << (response.success() ? "[OK] " : "[FAIL] ")
            << "message=" << response.message() << '\n';
  if (response.play_id() != 0) std::cout << "       play_id=" << response.play_id() << '\n';
  if (!response.resource_id().empty()) {
    std::cout << "       resource_id=" << response.resource_id() << '\n';
  }
}

void PrintStop(const ::xr::sdk::v2::StopAudioResponse& response) {
  std::cout << (response.success() ? "[OK] " : "[FAIL] ")
            << "message=" << response.message() << '\n';
}

bool WriteWav(const std::string& path, const AudioDataStamped& first,
              const std::vector<std::uint8_t>& pcm) {
  if (!first.has_audio_info() || pcm.empty()) return false;
  const auto& info = first.audio_info();
  const std::uint16_t channels = static_cast<std::uint16_t>(info.channels());
  const std::uint32_t rate = info.sample_rate();
  const std::uint16_t bits = static_cast<std::uint16_t>(info.bit_depth());
  if (channels == 0 || rate == 0 || bits == 0) return false;
  const std::uint16_t block = static_cast<std::uint16_t>(channels * bits / 8);
  const std::uint32_t byte_rate = rate * block;
  const std::uint32_t data_size = static_cast<std::uint32_t>(pcm.size());
  const std::uint32_t riff_size = 36 + data_size;
  std::ofstream out(path, std::ios::binary);
  if (!out) return false;
  out.write("RIFF", 4);
  out.write(reinterpret_cast<const char*>(&riff_size), 4);
  out.write("WAVEfmt ", 8);
  const std::uint32_t fmt_size = 16;
  const std::uint16_t pcm_format = 1;
  out.write(reinterpret_cast<const char*>(&fmt_size), 4);
  out.write(reinterpret_cast<const char*>(&pcm_format), 2);
  out.write(reinterpret_cast<const char*>(&channels), 2);
  out.write(reinterpret_cast<const char*>(&rate), 4);
  out.write(reinterpret_cast<const char*>(&byte_rate), 4);
  out.write(reinterpret_cast<const char*>(&block), 2);
  out.write(reinterpret_cast<const char*>(&bits), 2);
  out.write("data", 4);
  out.write(reinterpret_cast<const char*>(&data_size), 4);
  out.write(reinterpret_cast<const char*>(pcm.data()), static_cast<std::streamsize>(pcm.size()));
  return static_cast<bool>(out);
}

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string action = argc > 2 ? argv[2] : "stream";
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  auto audio = robot->GetAudio();
  bool ok = true;

  if (action == "play" || action == "replay") {
    const std::string source = argc > 3 ? argv[3] : "";
    ::xr::sdk::v2::PlayAudioRequest request;
    request.set_volume(argc > 4 ? std::stoi(argv[4]) : 80);
    request.set_priority(::xr::sdk::v2::PRIORITY_NORMAL);
    request.set_block(false);
    request.set_cache(true);
    request.set_timeout_s(600.0);
    if (action == "replay") {
      request.set_resource_id(source);
    } else {
      request.set_file_name(source);
      request.set_format(Extension(source));
      if (source.rfind("http://", 0) == 0 || source.rfind("https://", 0) == 0) {
        // The server performs URL retrieval when file_name is a URL.
        request.set_file_name(source);
      } else {
        std::ifstream in(source, std::ios::binary);
        if (!in) {
          std::cerr << "cannot open audio file: " << source << '\n';
          ok = false;
        } else {
          std::vector<char> bytes((std::istreambuf_iterator<char>(in)), {});
          request.set_audio_bytes(bytes.data(), bytes.size());
        }
      }
    }
    if (ok) {
      std::cout << "Connected. " << (action == "replay" ? "Replaying: " : "Playing: ")
                << source << '\n';
      auto result = audio.PlayAudio(request, std::chrono::minutes{10});
      ok = check(result, "Audio.PlayAudio");
      if (ok) PrintPlay(result.Value());
    }
  } else if (action == "stop") {
    ::xr::sdk::v2::StopAudioRequest request;
    request.set_play_id(argc > 3 ? std::stoull(argv[3]) : 0);
    auto result = audio.StopAudio(request, std::chrono::seconds{3});
    ok = check(result, "Audio.StopAudio");
    if (ok) PrintStop(result.Value());
  } else if (action == "stream") {
    const std::string output = argc > 3 ? argv[3] : "audio_output.wav";
    ::xr::sdk::v2::GetAudioStreamRequest request;
    auto opened = audio.GetAudioStream(request, std::chrono::seconds{5});
    ok = check(opened, "Audio.GetAudioStream");
    std::vector<std::uint8_t> pcm;
    AudioDataStamped first;
    std::size_t frames = 0;
    if (ok) {
      auto stream = std::move(opened.Value());
      std::cout << "Connected. Starting audio stream... Press Ctrl+C to stop.\n";
      while (!x2robot::examples::ExitRequestedBySignal()) {
        auto item = stream->Read(std::chrono::seconds{2});
        if (!item.Ok()) break;
        if (frames == 0) {
          first = item.Value();
          std::cout << "Audio format: channels=" << first.audio_info().channels()
                    << ", sample_rate=" << first.audio_info().sample_rate()
                    << "Hz, bit_depth=" << first.audio_info().bit_depth() << '\n';
        }
        const auto& bytes = item.Value().audio_data().data();
        pcm.insert(pcm.end(), bytes.begin(), bytes.end());
        ++frames;
        std::cout << "\rFrames: " << frames << ", Size: " << pcm.size() << " bytes"
                  << std::flush;
      }
      (void)stream->Cancel();
      std::cout << '\n';
      if (frames != 0) {
        ok = WriteWav(output, first, pcm);
        if (ok) std::cout << "Saved to: " << output << '\n';
      }
    }
  } else {
    std::cerr << "usage: " << argv[0]
              << " [address] [play|replay|stop|stream] [source/id/output] [volume]\n";
    ok = false;
  }
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
