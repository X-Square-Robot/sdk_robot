// Loopback-only fixture for the standalone TF example. No robot is involved.
#include "v2/core/tf_bridge.grpc.pb.h"
#include "v2/core/system.grpc.pb.h"
#include <grpcpp/grpcpp.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
using namespace std::chrono_literals;
class TfServer final : public xr::sdk::v2::TfBridge::Service {
 public:
  explicit TfServer(std::string scenario) : scenario_(std::move(scenario)) {}
  grpc::Status SubscribeTfStatic(grpc::ServerContext* context,
      const google::protobuf::Empty*, grpc::ServerWriter<xr::sdk::v2::TfMessageResult>* writer) override {
    return Send(context, writer, true);
  }
  grpc::Status SubscribeTf(grpc::ServerContext* context,
      const google::protobuf::Empty*, grpc::ServerWriter<xr::sdk::v2::TfMessageResult>* writer) override {
    return Send(context, writer, false);
  }
 private:
  grpc::Status Send(grpc::ServerContext* context,
      grpc::ServerWriter<xr::sdk::v2::TfMessageResult>* writer, bool is_static) {
    if (scenario_ == "rpc_error" || (scenario_ == "static_error" && is_static) ||
        (scenario_ == "dynamic_error" && !is_static))
      return {grpc::StatusCode::UNAVAILABLE, "fixture RPC failure"};
    if (scenario_ == "eof") return grpc::Status::OK;
    xr::sdk::v2::TfMessageResult result;
    result.mutable_header()->set_is_success(scenario_ != "sdk_error");
    if (scenario_ == "sdk_error") {
      result.mutable_header()->mutable_error()->set_error_name("FIXTURE_ERROR");
      result.mutable_header()->mutable_error()->set_error_message("fixture SDK failure");
      result.mutable_header()->mutable_error()->set_error_code(1);
    } else if (scenario_ != "empty") {
      auto* edge = result.mutable_data()->add_transforms();
      edge->mutable_header()->set_frame_id(is_static ? "world" : "base");
      edge->set_child_frame_id(is_static ? "base" : "tool");
      edge->mutable_transform()->mutable_rotation()->set_w(1);
    }
    if (scenario_ != "no_data") writer->Write(result);
    if (scenario_ == "disconnect") return {grpc::StatusCode::UNAVAILABLE, "fixture disconnect"};
    // Static publishers need never send a second sample. Dynamic publishers
    // keep advancing unless this case deliberately simulates a stalled stream.
    while (!context->IsCancelled()) {
      std::this_thread::sleep_for(50ms);
      if (!is_static && (scenario_ == "success" || scenario_ == "static_error") &&
          !writer->Write(result)) break;
    }
    return grpc::Status::OK;
  }
  std::string scenario_;
};
class SystemFixture final : public xr::sdk::v2::System::Service {
  grpc::Status GetModelType(grpc::ServerContext*, const google::protobuf::Empty*,
      xr::sdk::v2::ModelTypeResult* out) override {
    out->mutable_header()->set_is_success(true);
    out->mutable_data()->set_model_type(xr::sdk::v2::QUANTA_X1_PRO);
    return grpc::Status::OK;
  }
};
int main(int argc, char** argv) {
  if (argc != 2) return 2;
  TfServer service(argv[1]);
  SystemFixture system;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  builder.RegisterService(&system);
  auto server = builder.BuildAndStart();
  if (!server || port == 0) return 1;
  std::cout << "127.0.0.1:" << port << std::endl;
  server->Wait();
}
