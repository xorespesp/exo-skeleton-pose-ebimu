#pragma once
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

namespace app
{
    // Where frames come from: a camera or a recording file, never both or neither.
    class source_address
    {
    public:
        // A camera, by index within its backend's enumeration.
        struct k4a_device_t { uint32_t index{ 0 }; };
        struct vz_device_t { uint32_t index{ 0 }; };

        source_address() = default; // K4A device #0

        static source_address k4a_device(uint32_t index);
        static source_address vz_device(uint32_t index);
        static source_address recording(std::filesystem::path file);

        // Text form shared by the CLI and the wire protocol:
        //
        //   "k4a:<index>"   a K4A camera, by device index
        //   "vz:<index>"    a VZ camera, by device index
        //   anything else   a recording file path
        //
        // A camera always names its backend, so a bare number is nullopt rather than a guess.
        [[nodiscard]] static std::optional<source_address> try_parse(std::string_view text);

        bool is_k4a_device() const noexcept;
        bool is_vz_device() const noexcept;
        bool is_recording() const noexcept;
        bool is_device() const noexcept { return !this->is_recording(); } // either camera

        // Preconditions: the matching is_*() holds.
        uint32_t k4a_device_index() const;
        uint32_t vz_device_index() const;
        const std::filesystem::path& recording_path() const;

        std::string to_string() const;    // round-trips through try_parse()
        std::string display_name() const; // "k4a device #0" / "vz device #0" / "walk.mcap"

    private:
        std::variant<k4a_device_t, vz_device_t, std::filesystem::path> _value{ k4a_device_t{} };
    };

} // namespace app
