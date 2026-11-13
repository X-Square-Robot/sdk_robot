// 仅监听本机，模拟超过旧版 3 秒预算的回零，不连接机器人。
#include <chrono>
#include <iostream>
#include <thread>

#include <grpcpp/grpcpp.h>

#include "v2/controller/robot_control.grpc.pb.h"
#include "v2/core/system.grpc.pb.h"

using namespace std::chrono_literals;
namespace v = xr::sdk::v2;

class SystemFixture final : public v::System::Service
{
    grpc::Status GetModelType(grpc::ServerContext*, const google::protobuf::Empty*, v::ModelTypeResult* out) override
    {
        out->mutable_header()->set_is_success(true);
        out->mutable_data()->set_model_type(v::QUANTA_X3);
        return grpc::Status::OK;
    }
    grpc::Status SetWorkMode(grpc::ServerContext*, const v::RobotModeParam* mode, v::EmptyResult* out) override
    {
        std::cerr << "WORK_MODE " << mode->mode() << std::endl;
        out->mutable_header()->set_is_success(true);
        out->mutable_data();
        return grpc::Status::OK;
    }
};

class HomingFixture final : public v::RobotControl::Service
{
    grpc::Status Homing(grpc::ServerContext* context, const google::protobuf::Empty*, v::EmptyResult* out) override
    {
        const auto budget = context->deadline() - std::chrono::system_clock::now();
        std::cerr << "HOMING_BUDGET_MS " << std::chrono::duration_cast<std::chrono::milliseconds>(budget).count()
                  << std::endl;
        const auto finish = std::chrono::steady_clock::now() + 4s;
        while (std::chrono::steady_clock::now() < finish)
        {
            if (context->IsCancelled()) return {grpc::StatusCode::CANCELLED, "homing wait cancelled"};
            std::this_thread::sleep_for(20ms);
        }
        out->mutable_header()->set_is_success(true);
        out->mutable_data();
        std::cerr << "HOMING_FINISHED" << std::endl;
        return grpc::Status::OK;
    }
};

int main()
{
    SystemFixture system;
    HomingFixture homing;
    grpc::ServerBuilder builder;
    int port = 0;
    builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
    builder.RegisterService(&system);
    builder.RegisterService(&homing);
    auto server = builder.BuildAndStart();
    if (!server || !port) return 1;
    std::cout << "127.0.0.1:" << port << std::endl;
    server->Wait();
}
