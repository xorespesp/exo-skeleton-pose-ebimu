#include "source_address.hh"

#include <charconv>
#include <format>
#include <utility>

namespace app
{
    source_address source_address::device(const uint32_t index) noexcept
    {
        source_address out;
        out._value = index;
        return out;
    }

    source_address source_address::recording(std::filesystem::path file)
    {
        source_address out;
        out._value = std::move(file);
        return out;
    }

    std::optional<source_address> source_address::try_parse(const std::string_view text)
    {
        if (text.empty()) { return std::nullopt; }

        // Only an all-digit text is an index; "0.mcap" is a path.
        uint32_t index{};
        const auto* const last = text.data() + text.size();
        const auto [ptr, ec] = std::from_chars(text.data(), last, index);
        if (ec == std::errc{} && ptr == last) { return source_address::device(index); }

        return source_address::recording(std::filesystem::path{ text });
    }

    std::string source_address::to_string() const
    {
        return this->is_device() ? std::to_string(this->device_index()) : this->recording_path().string();
    }

    std::string source_address::display_name() const
    {
        return this->is_device()
            ? std::format("device #{}", this->device_index())
            : this->recording_path().filename().string();
    }

} // namespace app
