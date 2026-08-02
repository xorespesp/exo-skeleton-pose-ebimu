#include "source_address.hh"

#include <charconv>
#include <format>
#include <utility>

namespace app
{
    namespace
    {
        // What names a backend in the text form. A drive letter cannot be mistaken for one of
        // these, so a Windows path still reads as a path.
        constexpr std::string_view kK4aPrefix{ "k4a:" };
        constexpr std::string_view kVzPrefix{ "vz:" };

        template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

        // Whole unsigned integers only, so "0.mcap" and "1 " are not indices.
        std::optional<uint32_t> try_parse_index(const std::string_view text)
        {
            uint32_t index{};
            const auto* const last = text.data() + text.size();
            const auto [ptr, ec] = std::from_chars(text.data(), last, index);
            if (ec != std::errc{} || ptr != last) { return std::nullopt; }
            return index;
        }
    } // namespace

    source_address source_address::k4a_device(const uint32_t index)
    {
        source_address out;
        out._value = k4a_device_t{ index };
        return out;
    }

    source_address source_address::vz_device(const uint32_t index)
    {
        source_address out;
        out._value = vz_device_t{ index };
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

        if (text.starts_with(kK4aPrefix)) {
            const std::optional<uint32_t> index = try_parse_index(text.substr(kK4aPrefix.size()));
            if (!index.has_value()) { return std::nullopt; }
            return source_address::k4a_device(*index);
        }
        if (text.starts_with(kVzPrefix)) {
            const std::optional<uint32_t> index = try_parse_index(text.substr(kVzPrefix.size()));
            if (!index.has_value()) { return std::nullopt; }
            return source_address::vz_device(*index);
        }

        // Refused rather than taken as a path, so the failure names the missing prefix.
        if (try_parse_index(text).has_value()) { return std::nullopt; }

        return source_address::recording(std::filesystem::path{ text });
    }

    bool source_address::is_k4a_device() const noexcept
    {
        return std::holds_alternative<k4a_device_t>(_value);
    }

    bool source_address::is_vz_device() const noexcept
    {
        return std::holds_alternative<vz_device_t>(_value);
    }

    bool source_address::is_recording() const noexcept
    {
        return std::holds_alternative<std::filesystem::path>(_value);
    }

    uint32_t source_address::k4a_device_index() const
    {
        return std::get<k4a_device_t>(_value).index;
    }

    uint32_t source_address::vz_device_index() const
    {
        return std::get<vz_device_t>(_value).index;
    }

    const std::filesystem::path& source_address::recording_path() const
    {
        return std::get<std::filesystem::path>(_value);
    }

    std::string source_address::to_string() const
    {
        return std::visit(overloaded{
            [](const k4a_device_t& d) { return std::format("{}{}", kK4aPrefix, d.index); },
            [](const vz_device_t& d)  { return std::format("{}{}", kVzPrefix, d.index); },
            [](const std::filesystem::path& p) { return p.string(); },
        }, _value);
    }

    std::string source_address::display_name() const
    {
        return std::visit(overloaded{
            [](const k4a_device_t& d) { return std::format("k4a device #{}", d.index); },
            [](const vz_device_t& d)  { return std::format("vz device #{}", d.index); },
            [](const std::filesystem::path& p) { return p.filename().string(); },
        }, _value);
    }

} // namespace app
