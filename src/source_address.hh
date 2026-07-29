#pragma once
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

namespace app
{
    // Where frames come from: a live camera index or a recording file, never both or neither.
    class source_address
    {
    public:
        source_address() = default; // device #0

        static source_address device(uint32_t index) noexcept;
        static source_address recording(std::filesystem::path file);

        // Text form shared by the CLI and the wire protocol: a whole unsigned integer is a
        // device index, anything else a recording path. nullopt if the text names neither.
        [[nodiscard]] static std::optional<source_address> try_parse(std::string_view text);

        bool is_device() const noexcept { return std::holds_alternative<uint32_t>(_value); }
        bool is_recording() const noexcept { return !this->is_device(); }

        // Preconditions: the matching is_*() holds.
        uint32_t device_index() const { return std::get<uint32_t>(_value); }
        const std::filesystem::path& recording_path() const { return std::get<std::filesystem::path>(_value); }

        std::string to_string() const;    // round-trips through try_parse()
        std::string display_name() const; // "device #0" / "frontal.mcap"

    private:
        std::variant<uint32_t, std::filesystem::path> _value{ uint32_t{ 0 } };
    };

} // namespace app
