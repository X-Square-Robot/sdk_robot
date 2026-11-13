// Loopback fixture: exercises real public Client decoding and error handling.
#include "x2robot/sdk/robot.hpp"
#include <grpcpp/grpcpp.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <unistd.h>
using namespace std::chrono_literals;
namespace v = xr::sdk::v2;
std::string scenario;
bool quanta_x3 = false;
void Fill(sensor_msgs::Imu* p) { p->mutable_orientation()->set_w(1); }
void Fill(sensor_msgs::LaserScan* p) { p->add_ranges(1); }
void Fill(sensor_msgs::Range* p) { p->set_range(0.25); }
void Fill(v::AudioDataStamped* p) {
  auto* info = p->mutable_audio_info();
  info->set_channels(1);
  info->set_sample_rate(16000);
  info->set_bit_depth(16);
  p->mutable_audio_data()->set_data("\x01\x00\xff\x7f", 4);
}
void Fill(sensor_msgs::PointCloud2* p) {
  const bool large = scenario == "large";
  p->set_width(large ? 640 : 1); p->set_height(large ? 480 : 2);
  p->set_point_step(12); p->set_row_step(large ? 640 * 12 : 16);
  p->set_is_bigendian(scenario == "big_endian");
  p->mutable_header()->set_frame_id("depth");
  int offset = 0;
  for (const char* name : {"x", "y", "z"}) {
    auto* field = p->add_fields(); field->set_name(name);
    field->set_datatype(7); field->set_count(1); field->set_offset(offset); offset += 4;
  }
  p->mutable_data()->Reserve(p->row_step() * p->height());
  for (unsigned row = 0; row < p->height(); ++row) {
    for (unsigned column = 0; column < p->width(); ++column) {
      float xyz[] = {-1.25f - row, -2.5f, -3.75f};
      unsigned char raw[12]; std::memcpy(raw, xyz, 12);
      for (unsigned i = 0; i < 12; ++i)
        p->add_data(raw[p->is_bigendian() ? (i / 4) * 4 + 3 - i % 4 : i]);
    }
    if (!large) for (int i = 0; i < 4; ++i) p->add_data(255);
  }
  if (scenario == "packed_stale_stride" || scenario == "packed_zero_stride" ||
      scenario == "packed_truncated") {
    p->set_width(2); p->set_height(1); p->set_point_step(16);
    p->set_row_step(scenario == "packed_zero_stride" ? 0 : 5120);
    if (scenario == "packed_truncated") p->mutable_data()->RemoveLast();
  }
  if (scenario == "missing_xyz") p->mutable_fields()->RemoveLast();
  if (scenario == "truncated") p->mutable_data()->RemoveLast();
  if (scenario == "bad_stride") p->set_row_step(4);
  if (scenario == "non_byte") p->set_data(0, 256);
  if (scenario == "bad_field") p->mutable_fields(0)->set_offset(11);
}
template <class Result>
grpc::Status Send(grpc::ServerContext* context, grpc::ServerWriter<Result>* writer) {
  if (scenario == "open_error") return {grpc::StatusCode::UNAVAILABLE, "fixture open failure"};
  if (scenario == "empty_stream") return grpc::Status::OK;
  Result out; out.mutable_header()->set_is_success(true); Fill(out.mutable_data());
  if (scenario == "x3_withdrawn" || scenario == "withdrawn" || scenario == "x3_unavailable")
  {
      out.Clear();
      out.mutable_header()->set_is_success(false);
      auto* error = out.mutable_header()->mutable_error();
      const bool unavailable = scenario == "x3_unavailable";
      error->set_error_code(unavailable ? 100503 : 100501);
      error->set_error_name(unavailable ? "UNAVAILABLE" : "UNIMPLEMENTED");
      error->set_error_message("fixture capability response");
      writer->Write(out);
      return grpc::Status::OK;
  }
  if (!writer->Write(out)) return grpc::Status::OK;
  if (scenario == "disconnect") return {grpc::StatusCode::UNAVAILABLE, "fixture disconnect"};
  if (scenario == "sdk_error") {
    out.Clear(); out.mutable_header()->set_is_success(false);
    auto* e = out.mutable_header()->mutable_error(); e->set_error_code(100500);
    e->set_error_name("FIXTURE_ERROR"); e->set_error_message("fixture SDK error");
    writer->Write(out); return grpc::Status::OK;
  }
  if (scenario == "continuous") {
    while (!context->IsCancelled()) {
      std::this_thread::sleep_for(20ms);
      if (!writer->Write(out)) break;
    }
    return grpc::Status::OK;
  }
  while (!context->IsCancelled()) std::this_thread::sleep_for(10ms);
  return grpc::Status::OK;
}
#define RPC(Name, Result) grpc::Status Name(grpc::ServerContext* c, const google::protobuf::Empty*, grpc::ServerWriter<v::Result>* w) override { return Send(c, w); }
class Depth final : public v::DepthPoints::Service {
  grpc::Status GetChassisDepthPointsStream(grpc::ServerContext* c, const google::protobuf::Empty*,
                                         grpc::ServerWriter<v::PointCloud2Result>* w) override {
    std::cerr << "CHASSIS_READ\n";
    return Send(c, w);
  }
  grpc::Status GetLidarPointsStream(grpc::ServerContext* c, const google::protobuf::Empty*,
                                  grpc::ServerWriter<v::PointCloud2Result>* w) override {
    std::cerr << "LIDAR_READ\n";
    return Send(c, w);
  }
};
class Imu final : public v::Imu::Service { RPC(GetChassisImuStream, ImuResult) };
class Radar final : public v::RadarService::Service { RPC(GetLaserScanStream, LaserScanResult) };
class Tof final : public v::Tof::Service {
  RPC(GetChassisTof1Stream, RangeResult) RPC(GetChassisTof2Stream, RangeResult)
};
class Ultrasonic final : public v::Ultrasonic::Service {
  RPC(GetChassisUltrasonic1Stream, RangeResult) RPC(GetChassisUltrasonic2Stream, RangeResult)
  RPC(GetChassisUltrasonic3Stream, RangeResult) RPC(GetChassisUltrasonic4Stream, RangeResult)
};
#undef RPC
class Audio final : public v::Audio::Service {
  grpc::Status GetAudioStream(grpc::ServerContext* context,
                             const v::GetAudioStreamRequest*,
                             grpc::ServerWriter<v::AudioDataStampedResult>* writer) override {
    return Send(context, writer);
  }
};
class System final : public v::System::Service {
  grpc::Status GetModelType(grpc::ServerContext*, const google::protobuf::Empty*, v::ModelTypeResult* out) override {
    out->mutable_header()->set_is_success(true);
    out->mutable_data()->set_model_type(quanta_x3 || scenario.rfind("x3_", 0) == 0 ? v::QUANTA_X3 : v::QUANTA_X1_PRO);
    return grpc::Status::OK;
  }
  grpc::Status SetWorkMode(grpc::ServerContext*, const v::RobotModeParam*, v::EmptyResult*) override {
    std::cerr << "UNEXPECTED_WRITE\n"; return {grpc::StatusCode::UNIMPLEMENTED, "read-only test"};
  }
};
int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) return 2;
  quanta_x3 = argc == 3 && std::string(argv[2]) == "quanta_x3";
  scenario = argv[1]; System system; Depth depth; Imu imu; Radar radar; Tof tof; Ultrasonic ultrasonic; Audio audio;
  grpc::ServerBuilder builder;
  const auto address = "unix:/tmp/sdk_sensor_fixture_" + std::to_string(::getpid()) + ".sock";
  builder.AddListeningPort(address, grpc::InsecureServerCredentials());
  builder.SetMaxSendMessageSize(16 * 1024 * 1024);
  for (grpc::Service* service : std::initializer_list<grpc::Service*>{&system, &depth, &imu, &radar, &tof, &ultrasonic, &audio}) builder.RegisterService(service);
  auto server = builder.BuildAndStart(); if (!server) return 1;
  std::cout << address << std::endl; server->Wait();
}
