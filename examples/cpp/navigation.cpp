// Counterpart of python/v1/navigation.py.  Usage:
//   navigation [address] load <map-name> <file> | export <map-id> <output> | list
#include "x2robot/sdk/robot.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using namespace std::chrono_literals;

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  const auto& e = result.ErrorValue();
  std::cerr << operation << " failed [" << e.error_name << "]: " << e.error_message
            << " (request_id=" << result.request_id << ")\n";
  return false;
}

std::vector<::xr::sdk::v2::UploadRequest> MakeChunks(const std::string& map_name,
                                                     const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) return {};
  input.seekg(0, std::ios::end); const auto size = static_cast<std::uint64_t>(input.tellg());
  input.seekg(0, std::ios::beg);
  constexpr std::size_t kChunk = 1024 * 1024;
  std::vector<::xr::sdk::v2::UploadRequest> chunks;
  std::uint64_t sent = 0;
  while (sent < size || chunks.empty()) {
    const auto count = static_cast<std::size_t>(std::min<std::uint64_t>(kChunk, size - sent));
    std::string bytes(count, '\0');
    input.read(bytes.data(), static_cast<std::streamsize>(count));
    ::xr::sdk::v2::UploadRequest request;
    if (sent == 0) {
      auto* meta = request.mutable_meta();
      meta->set_identifier(map_name);
      meta->set_total_size(size);
    } else request.set_chunk(bytes);
    if (sent == 0 && count != 0) {
      // metadata and first payload are separate oneof messages by contract.
      chunks.push_back(std::move(request));
      request.Clear(); request.set_chunk(bytes);
    }
    chunks.push_back(std::move(request)); sent += count;
    if (size == 0) break;
  }
  return chunks;
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string action = argc > 2 ? argv[2] : "list";
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  auto navigation = robot->GetNavigation();
  bool ok = true;
  if (action == "load") {
    const std::string map_name = argc > 3 ? argv[3] : "";
    const std::string path = argc > 4 ? argv[4] : "";
    auto chunks = MakeChunks(map_name, path);
    if (chunks.empty() && !path.empty()) std::cerr << "cannot read map: " << path << '\n';
    auto result = navigation.LoadMap(chunks, 120s);
    ok = check(result, "Navigation.LoadMap");
    if (ok) std::cout << "Map loaded successfully: " << map_name << " from " << path << '\n';
  } else if (action == "export") {
    const std::string id = argc > 3 ? argv[3] : "";
    const std::string output = argc > 4 ? argv[4] : id + ".map";
    ::xr::sdk::v2::DownloadRequest request; request.set_identifier(id);
    auto opened = navigation.ExportMap(request, 10s);
    ok = check(opened, "Navigation.ExportMap");
    if (ok) {
      auto stream = std::move(opened.Value()); std::ofstream out(output, std::ios::binary);
      while (true) {
        auto chunk = stream->Read(10s);
        if (!chunk.Ok()) { if (chunk.ErrorValue().error_code != x2robot::sdk::errc::kCancelled) ok = check(chunk, "Navigation.ExportMap.Read"); break; }
        out.write(chunk.Value().chunk().data(), static_cast<std::streamsize>(chunk.Value().chunk().size()));
      }
      (void)stream->Cancel(); ok = static_cast<bool>(out) && ok;
      if (ok) std::cout << "Map exported to " << output << '\n';
    }
  } else if (action == "list") {
    auto result = navigation.GetMapList(5s);
    ok = check(result, "Navigation.GetMapList");
    if (ok) {
      std::cout << "Available maps:\n";
      for (const auto& map : result.Value().map_list()) std::cout << "  " << map << '\n';
    }
  } else {
    std::cerr << "Valid actions: load <map-name> <file>, export <map-id> <output>, list\n"; ok = false;
  }
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
