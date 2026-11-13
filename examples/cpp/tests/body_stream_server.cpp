// Loopback-only fixture for body stream examples; no robot is involved.
#include "v2/controller/arm_controller.grpc.pb.h"
#include "v2/controller/head_controller.grpc.pb.h"
#include "v2/controller/gripper_controller.grpc.pb.h"
#include "v2/controller/lift_controller.grpc.pb.h"
#include "v2/controller/waist_controller.grpc.pb.h"
#include "v2/controller/robot_control.grpc.pb.h"
#include "v2/core/system.grpc.pb.h"
#include <grpcpp/grpcpp.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
using namespace std::chrono_literals;
namespace v = xr::sdk::v2;

void Fill(sensor_msgs::JointState* p) {
  p->mutable_header()->mutable_stamp()->set_sec(1);
  p->add_name("fixture_joint");
  p->add_position(0.25);
}
void Fill(geometry_msgs::PoseStamped* p) {
  p->mutable_header()->set_frame_id("fixture_frame");
  p->mutable_header()->mutable_stamp()->set_sec(1);
  p->mutable_pose()->mutable_orientation()->set_w(1);
}
void Fill(v::GripperPosition* p) { p->set_position(0.25); }

template <class Result>
grpc::Status Send(const std::string& scenario, grpc::ServerContext* context,
                  grpc::ServerWriter<Result>* writer) {
  if (scenario == "open_error")
    return {grpc::StatusCode::UNAVAILABLE, "fixture open failure"};
  Result response;
  response.mutable_header()->set_is_success(true);
  Fill(response.mutable_data());
  if (!writer->Write(response)) return grpc::Status::OK;
  if (scenario == "collection") {
    while (!context->IsCancelled()) {
      std::this_thread::sleep_for(20ms);
      if (!writer->Write(response)) break;
    }
    return grpc::Status::OK;
  }
  if (scenario == "disconnect")
    return {grpc::StatusCode::UNAVAILABLE, "fixture disconnect after data"};
  if (scenario == "sdk_error") {
    response.Clear();
    response.mutable_header()->set_is_success(false);
    auto* error = response.mutable_header()->mutable_error();
    error->set_error_code(100500);
    error->set_error_name("FIXTURE_ERROR");
    error->set_error_message("fixture SDK error after data");
    writer->Write(response);
    return grpc::Status::OK;
  }
  // A stall must fail the example's existing two-second Read deadline.
  while (!context->IsCancelled()) std::this_thread::sleep_for(20ms);
  return grpc::Status::OK;
}

template <class Service> class JointFixture : public Service::Service {
 public:
  explicit JointFixture(std::string scenario) : scenario_(std::move(scenario)) {}
  grpc::Status GetJointStatesStream(grpc::ServerContext* c,
      const google::protobuf::Empty*, grpc::ServerWriter<v::JointStateResult>* w) override {
    return Send(scenario_, c, w);
  }
 protected:
  std::string scenario_;
};
template <class Service> class PoseFixture : public JointFixture<Service> {
 public:
  using JointFixture<Service>::JointFixture;
  grpc::Status GetEndPoseStream(grpc::ServerContext* c,
      const google::protobuf::Empty*, grpc::ServerWriter<v::PoseStampedResult>* w) override {
    return Send(this->scenario_, c, w);
  }
};
template <class Service> class GripperFixture : public JointFixture<Service> {
 public:
  using JointFixture<Service>::JointFixture;
  grpc::Status GetPositionStream(grpc::ServerContext* c,
      const google::protobuf::Empty*, grpc::ServerWriter<v::GripperPositionResult>* w) override {
    return Send(this->scenario_, c, w);
  }
};
class SystemGuard final : public v::System::Service {
  grpc::Status GetModelType(grpc::ServerContext*, const google::protobuf::Empty*,
      v::ModelTypeResult* out) override {
    out->mutable_header()->set_is_success(true);
    out->mutable_data()->set_model_type(v::QUANTA_X1_PRO);
    return grpc::Status::OK;
  }
  grpc::Status SetWorkMode(grpc::ServerContext*, const v::RobotModeParam*, v::EmptyResult*) override {
    std::cerr << "UNEXPECTED_WRITE SetWorkMode\n";
    return {grpc::StatusCode::UNIMPLEMENTED, "read example attempted a mode write"};
  }
};
class ControlGuard final : public v::RobotControl::Service {
  grpc::Status SetManipulatorControlMode(grpc::ServerContext*,
      const v::ManipulatorControlModeParam*, v::EmptyResult*) override {
    std::cerr << "UNEXPECTED_WRITE SetManipulatorControlMode\n";
    return {grpc::StatusCode::UNIMPLEMENTED, "read example attempted a mode write"};
  }
};
int main(int argc, char** argv) {
  if (argc != 2) return 2;
  PoseFixture<v::LeftArmController> left(argv[1]);
  PoseFixture<v::RightArmController> right(argv[1]);
  GripperFixture<v::LeftGripperController> lg(argv[1]);
  GripperFixture<v::RightGripperController> rg(argv[1]);
  JointFixture<v::LiftController> lift(argv[1]);
  PoseFixture<v::WaistController> waist(argv[1]);
  JointFixture<v::HeadController> head(argv[1]);
  SystemGuard system;
  ControlGuard control;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  grpc::Service* services[] = {&left, &right, &lg, &rg, &lift, &waist, &head, &system, &control};
  for (grpc::Service* service : services)
    builder.RegisterService(service);
  auto server = builder.BuildAndStart();
  if (!server || !port) return 1;
  std::cout << "127.0.0.1:" << port << std::endl;
  server->Wait();
}
