#pragma once
#include "source_address.hh"

#include "hw/roi.hh"
#include "pose/frontal_pose_estimator.hh"
#include "pose/sagittal_pose_estimator.hh"
#include "pose/tag_detector.hh"
#include "pose/view_plane.hh"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace app
{
    // An installation's own settings, persisted as JSON. Fixed for one physical setup, hence off
    // the wire; what an operator adjusts during a run (the rest pose) is session state and stays out.

    // The sensor: which one to open, and what gets programmed into it.
    struct camera_config_t
    {
        std::optional<source_address> source; // nullopt: nothing to auto-open

        std::optional<int32_t> exposure_us; // nullopt: auto
        std::optional<int32_t> gain;        // nullopt: auto
        std::optional<hw::roi_t> roi;       // nullopt: whole frames

        // Calibration measured off-line, for a camera reporting none of its own.
        // Empty leaves tag poses unsolvable, which the 2D estimators do not mind. Ignored for a K4A.
        std::string intrinsics_file;
    };

    // Turning frames into joint angles. Both planes are kept, so switching loses neither.
    struct pose_config_t
    {
        // Where the camera stands relative to the exo, which picks the estimator that runs.
        pose::view_plane_t view_plane{ pose::view_plane_t::frontal };

        double tag_size_m{ 0.05 }; // printed black-square edge length [m]

        pose::tag_detector::options_t detector;
        pose::frontal_pose_estimator::options_t frontal;
        pose::sagittal_pose_estimator::options_t sagittal;
    };

    struct server_config_t
    {
        uint16_t port{ 9002 };
    };

    struct app_config_t
    {
        server_config_t server;
        camera_config_t camera;
        pose_config_t pose;
    };

    // Where the app keeps its own output (`recordings`, `dumps`, `configs`):
    // the nearest such folder at or above the executable, else the executable's own directory.
    // Not config-relative, since these hold what the tool produces, not what an installation names.
    [[nodiscard]] std::filesystem::path project_dir(std::string_view name);

    // Every key the schema names must be present, `config_version` included; a default standing in
    // for an omitted one is the failure this guards against. Keys it does not name are ignored, so
    // a file may carry notes of its own. On failure `err` names the key and `out` is untouched.
    [[nodiscard]] bool load_config(const std::filesystem::path& path, app_config_t& out, std::string& err);

    [[nodiscard]] bool save_config(const app_config_t& config, const std::filesystem::path& path, std::string& err);

    // The JSON text `save_config()` would write. (--dump-config)
    [[nodiscard]] std::string dump_config(const app_config_t& config);

    // What `--config` named: a path if it carries a separator or a `.json` suffix, else a profile
    // sought as `configs/<name>.json` and then as `<name>.json` beside the executable, matching
    // where `project_dir("configs")` writes. An empty name seeks the `default` profile, and
    // nullopt means there is none, so a bare run uses the built-in defaults.
    [[nodiscard]] std::optional<std::filesystem::path> find_config_file(std::string_view name);

} // namespace app
