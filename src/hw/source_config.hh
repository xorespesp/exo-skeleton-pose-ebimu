#pragma once
#include "calibration.hh"
#include "frame_format.hh"
#include "roi.hh"
#include "source_backend.hh"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>

namespace hw
{
    // What to open and how to stream it.
    // Every config carries `roi`, which the provider reads generically.

    // Orbbec K4A Wrapper camera.
    struct k4a_device_config_t
    {
        uint32_t device_index{ 0 };
        std::optional<int32_t> exposure_us; // nullopt: auto exposure
        std::optional<int32_t> gain;        // nullopt: auto gain

        frame_format_t frame_format{ frame_format_t::bgr8 };
        std::optional<roi_t> roi;     // nullopt: the whole frame
    };

    // Vieworks VZ-5MU-C79H00 camera.
    struct vz_device_config_t
    {
        uint32_t device_index{ 0 };        // position in the enumeration
        std::optional<double> exposure_us; // nullopt: leave the camera's current setting
        std::optional<double> gain;        // nullopt: leave the camera's current setting

        frame_format_t frame_format{ frame_format_t::gray8 };
        std::optional<roi_t> roi;    // nullopt: the whole frame

        // The VZ camera reports no intrinsics of its own, so a calibration measured off-line is supplied here. 
        // NOTE: Left empty, tag poses cannot be solved; the estimators that work off 2D tag centers do not need them.
        std::optional<intrinsic_t>  intrinsic;
        std::optional<distortion_t> distortion; // empty alongside intrinsics: no distortion
    };

    // One of our own recordings, played back in place of a camera.
    struct recording_config_t
    {
        std::filesystem::path file;
        std::optional<roi_t> roi; // nullopt: the whole frame
    };

    using source_config_t = std::variant<k4a_device_config_t, vz_device_config_t, recording_config_t>;

    // Which backend this config selects.
    source_backend_t get_source_backend(const source_config_t& config);

    // Short human-facing label,
    // e.g. "k4a device #0", "recording 'walk.mcap'".
    std::string describe(const source_config_t& config);

} // namespace hw
