#pragma once

#include <vector>

#include "v2/controller/navigation.grpc.pb.h"
#include "x2robot/sdk/service/grpc_service.hpp"

namespace x2robot::sdk {

class Navigation final : public client::detail::GrpcServiceClient<::xr::sdk::v2::Navigation>
{
    using Service = ::xr::sdk::v2::Navigation;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit Navigation(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

#define X2ROBOT_NAV_EMPTY(Method)                                                       \
    SdkResult<void> Method(Deadline deadline = Deadline{200}) const                     \
    {                                                                                   \
        ::google::protobuf::Empty request;                                              \
        return UnaryVoid<::xr::sdk::v2::EmptyResult>(request, &Stub::Method, deadline, #Method); \
    }

    X2ROBOT_NAV_EMPTY(StartMapping)
    SdkResult<void> StopMapping(const ::xr::sdk::v2::SaveMapParam& request, Deadline deadline = Deadline{1000}) const
    {
        return UnaryVoid<::xr::sdk::v2::EmptyResult>(request, &Stub::StopMapping, deadline, "StopMapping");
    }
    SdkResult<void> SetNavigationMode(const ::xr::sdk::v2::NavigationModeParam& request, Deadline deadline = Deadline{200}) const
    {
        return UnaryVoid<::xr::sdk::v2::EmptyResult>(request, &Stub::SetNavigationMode, deadline, "SetNavigationMode");
    }
    SdkResult<void> StartLocalization(const ::xr::sdk::v2::StartLocalizationParam& request, Deadline deadline = Deadline{1000}) const
    {
        return UnaryVoid<::xr::sdk::v2::EmptyResult>(request, &Stub::StartLocalization, deadline, "StartLocalization");
    }
    X2ROBOT_NAV_EMPTY(StopLocalization)
    // 允许 ROS 服务发现及取消操作在返回前完成。
    SdkResult<void> CancelMapping(Deadline deadline = Deadline{15000}) const
    {
        ::google::protobuf::Empty request;
        return UnaryVoid<::xr::sdk::v2::EmptyResult>(request, &Stub::CancelMapping, deadline, "CancelMapping");
    }
    X2ROBOT_NAV_EMPTY(CancelNavigation)

    SdkResult<void> LoadMap(const std::vector<::xr::sdk::v2::UploadRequest>& chunks, Deadline deadline = Deadline{120000}) const
    {
        return ClientStreamVoid<::xr::sdk::v2::UploadRequest, ::xr::sdk::v2::EmptyResult>(chunks, &Stub::LoadMap, deadline, "LoadMap");
    }

    SdkResult<ResultStreamHandle<::xr::sdk::v2::DownloadResponse>> ExportMap(const ::xr::sdk::v2::DownloadRequest& request,
                                                                         Deadline first_deadline = Deadline{3000}) const
    {
        return ServerStream<::xr::sdk::v2::DownloadResponse, ::xr::sdk::v2::DownloadResponseResult>(request, &Stub::ExportMap, first_deadline, "ExportMap");
    }

    SdkResult<::xr::sdk::v2::MapListData> GetMapList(Deadline deadline = Deadline{1000}) const
    {
        ::google::protobuf::Empty request;
        return Unary<::xr::sdk::v2::MapListData, ::xr::sdk::v2::MapListResult>(request, &Stub::GetMapList, deadline, "GetMapList");
    }

    SdkResult<::xr::sdk::v2::JsonData> GetHumanTrackingInfo(Deadline deadline = Deadline{1000}) const
    {
        ::google::protobuf::Empty request;
        return Unary<::xr::sdk::v2::JsonData, ::xr::sdk::v2::JsonResult>(request, &Stub::GetHumanTrackingInfo, deadline, "GetHumanTrackingInfo");
    }

    SdkResult<::xr::sdk::v2::JsonData> GetNavigationStatus(Deadline deadline = Deadline{1000}) const
    {
        ::google::protobuf::Empty request;
        return Unary<::xr::sdk::v2::JsonData, ::xr::sdk::v2::JsonResult>(request, &Stub::GetNavigationStatus, deadline, "GetNavigationStatus");
    }

#undef X2ROBOT_NAV_EMPTY
};

}  // namespace x2robot::sdk
