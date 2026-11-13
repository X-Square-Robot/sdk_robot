#pragma once

#include "v2/sensors/audio.grpc.pb.h"
#include "x2robot/sdk/service/grpc_service.hpp"

namespace x2robot::sdk {

class Audio final : public client::detail::GrpcServiceClient<::xr::sdk::v2::Audio>
{
    using Service = ::xr::sdk::v2::Audio;
    using Base = client::detail::GrpcServiceClient<Service>;
    using Stub = Service::StubInterface;

   public:
    explicit Audio(std::weak_ptr<client::ServiceContext> context) : Base(std::move(context))
    {
    }

    SdkResult<::xr::sdk::v2::PlayAudioResponse> PlayAudio(const ::xr::sdk::v2::PlayAudioRequest& request, Deadline deadline = Deadline{600000}) const
    {
        return Unary<::xr::sdk::v2::PlayAudioResponse, ::xr::sdk::v2::PlayAudioResponseResult>(request, &Stub::PlayAudio, deadline, "PlayAudio");
    }

    SdkResult<::xr::sdk::v2::StopAudioResponse> StopAudio(const ::xr::sdk::v2::StopAudioRequest& request, Deadline deadline = Deadline{1000}) const
    {
        return Unary<::xr::sdk::v2::StopAudioResponse, ::xr::sdk::v2::StopAudioResponseResult>(request, &Stub::StopAudio, deadline, "StopAudio");
    }

    SdkResult<ResultStreamHandle<::xr::sdk::v2::AudioDataStamped>> GetAudioStream(const ::xr::sdk::v2::GetAudioStreamRequest& request,
                                                                              Deadline first_deadline = Deadline{3000}) const
    {
        return ServerStream<::xr::sdk::v2::AudioDataStamped, ::xr::sdk::v2::AudioDataStampedResult>(request, &Stub::GetAudioStream, first_deadline, "GetAudioStream");
    }

    SdkResult<::xr::sdk::v2::JsonData> GetAudioPlaybackStatus(Deadline deadline = Deadline{1000}) const
    {
        ::google::protobuf::Empty request;
        return Unary<::xr::sdk::v2::JsonData, ::xr::sdk::v2::JsonResult>(request, &Stub::GetAudioPlaybackStatus, deadline, "GetAudioPlaybackStatus");
    }

    SdkResult<::xr::sdk::v2::JsonData> GetSpeechHistory(const ::xr::sdk::v2::GetSpeechHistoryRequest& request, Deadline deadline = Deadline{1000}) const
    {
        return Unary<::xr::sdk::v2::JsonData, ::xr::sdk::v2::JsonResult>(request, &Stub::GetSpeechHistory, deadline, "GetSpeechHistory");
    }
};

}  // namespace x2robot::sdk
