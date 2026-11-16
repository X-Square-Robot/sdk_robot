#pragma once

#include <functional>

#include "v2/common/file_transfer.pb.h"
#include "x2robot/sdk/core/service_context.hpp"

namespace x2robot::sdk {

// Whole-file convenience API with bounded buffering. Each call returns only
// after both transport completion and size/MD5 validation. Existing targets
// are never overwritten. Keep request_id to retry an uncertain upload result.
class Files
{
   public:
    explicit Files(std::weak_ptr<client::ServiceContext> context) : context_(std::move(context))
    {
    }
    SdkResult<::xr::sdk::v2::FileReceipt> UploadFile(const std::string& local_path,
                                                     const std::string& root_id,
                                                     const std::string& relative_path,
                                                     std::string request_id = {},
                                                     Deadline deadline = Deadline{600000}) const;
    SdkResult<std::string> DownloadFile(const std::string& file_id, const std::string& local_path, Deadline deadline = Deadline{600000}) const;

    // received/total are bytes. verified is true only after MD5 and local publication.
    // Callback exceptions disable notifications without failing the transfer.
    using ProgressCallback = std::function<void(uint64_t received, uint64_t total, bool verified)>;
    SdkResult<std::string> DownloadFile(const std::string& file_id,
                                        const std::string& local_path,
                                        Deadline deadline,
                                        ProgressCallback progress) const;

   private:
    std::weak_ptr<client::ServiceContext> context_;
};
}  // namespace x2robot::sdk
