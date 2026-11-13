#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>

namespace x2robot::sdk::files {
inline constexpr std::size_t kChunkSize = 1U << 20;

inline bool ValidMd5(std::string_view value)
{
    if (value.size() != 32) return false;
    for (const char c : value)
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    return true;
}

// Incremental file integrity only. Implementation dependencies stay private.
class Md5
{
   public:
    Md5();
    ~Md5();
    void Update(std::string_view bytes);
    std::string Finish();
    static std::string Hex(const unsigned char* bytes, std::size_t count);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
std::string NewTransferId();
}  // namespace x2robot::sdk::files
