#pragma once
#include "source_address.hh"

#include "hw/frame_format.hh"
#include "hw/roi.hh"
#include "pose/color_marker_detector.hh"
#include "pose/frontal_pose_estimator.hh"
#include "pose/sagittal_pose_estimator.hh"
#include "pose/tag_detector.hh"
#include "pose/view_plane.hh"
#include "utils/serializable.hh"

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

        DECLARE_SERIALIZABLE_FIELDS(
            v("source",          o.source);
            v("exposure_us",     o.exposure_us);
            v("gain",            o.gain);
            v("roi",             o.roi);
            v("intrinsics_file", o.intrinsics_file);
        )
    };

    // What is printed on the exo, which decides how a frame becomes per-joint measurements.
    enum class marker_kind_t
    {
        apriltag,     // tags carry an id, so a detection names its own joint
        color_marker, // plain coloured discs, placed by their order along the limb
    };

    constexpr std::string_view marker_kind_name(marker_kind_t k) {
        return (k == marker_kind_t::color_marker) ? "color_marker" : "apriltag";
    }

    // nullopt if `name` is neither kind. The names round-trip through `marker_kind_name()`,
    // which is what the config file spells.
    constexpr std::optional<marker_kind_t> marker_kind_from_name(std::string_view name) {
        if (name == marker_kind_name(marker_kind_t::apriltag))     { return marker_kind_t::apriltag; }
        if (name == marker_kind_name(marker_kind_t::color_marker)) { return marker_kind_t::color_marker; }
        return std::nullopt;
    }

    // What a source has to deliver for `k` to be detectable, which is what an open programs into
    // the camera. Colour classification needs the colour; reading a printed pattern takes luminance
    // alone, and asking for one channel there keeps the link bandwidth and the demosaic cost off.
    constexpr hw::frame_format_t marker_frame_format(marker_kind_t k) {
        return (k == marker_kind_t::color_marker) ? hw::frame_format_t::bgr8 : hw::frame_format_t::gray8;
    }

    // What this installation's markers photograph as, measured on site by the debugger's colour
    // panel. Written back into the profile it came from, so what a run detects with and what the
    // file says are the same thing.
    //
    // The blob filters ride with the colour because they are decided in the same sitting and by the
    // same physical facts: how many pixels a marker covers at this distance and frame size, how the
    // light glances off it, how far a swing smears it. `min_score` goes further and is stated
    // against the colour model itself, so it means something different the moment that changes.
    struct color_marker_calibration_t
    {
        pose::color_marker_detector::options_t detector; // blob filters, and the colour model inside

        // What the camera delivered while this was measured, which the `camera` block above does
        // not state: a ROI, a binned mode or another sensor all change how many pixels a marker
        // covers, and the blob gates are counted in pixels. Compared at open.
        Eigen::Vector2i frame_resolution{ 0, 0 };

        DECLARE_SERIALIZABLE_FIELDS(
            v("detector",         o.detector);
            v("frame_resolution", o.frame_resolution);
        )
    };

    // The colour-marker path.
    struct color_marker_config_t
    {
        // Empty until someone has measured this installation, which is the state a profile is
        // authored in. Nothing is detected without it, and the colour panel is where it comes from.
        std::optional<color_marker_calibration_t> calibration;

        pose::color_marker_assigner::options_t assigner; // which blob is which joint

        DECLARE_SERIALIZABLE_FIELDS(
            v("calibration", o.calibration);
            v("assigner",    o.assigner);
        )
    };

    // Both marker kinds are kept, so switching loses neither one's tuning.
    struct detector_config_t
    {
        marker_kind_t kind{ marker_kind_t::apriltag };

        pose::tag_detector::options_t apriltag;
        color_marker_config_t color_marker;

        DECLARE_SERIALIZABLE_FIELDS(
            v("kind",         o.kind);
            v("apriltag",     o.apriltag);
            v("color_marker", o.color_marker);
        )
    };

    // Turning frames into joint angles. Both planes are kept, so switching loses neither.
    struct pose_config_t
    {
        // Where the camera stands relative to the exo, which picks the estimator that runs.
        pose::view_plane_t view_plane{ pose::view_plane_t::frontal };

        double tag_size_m{ 0.05 }; // printed black-square edge length [m]

        detector_config_t detector;
        pose::frontal_pose_estimator::options_t frontal;
        pose::sagittal_pose_estimator::options_t sagittal;

        DECLARE_SERIALIZABLE_FIELDS(
            v("view_plane", o.view_plane);
            v("tag_size_m", o.tag_size_m);
            v("detector",   o.detector);
            v("frontal",    o.frontal);
            v("sagittal",   o.sagittal);
        )
    };

    struct server_config_t
    {
        uint16_t port{ 9002 };

        DECLARE_SERIALIZABLE_FIELDS(
            v("port", o.port);
        )
    };

    struct app_config_t
    {
        // The current config schema version. (YYMMDDRR) 
        // It has to be updated whenever any field list nested below this one changes.
        static constexpr int config_version = 26081000;

        server_config_t server;
        camera_config_t camera;
        pose_config_t pose;

        DECLARE_SERIALIZABLE_FIELDS(
            // NOTE: `config_version` MUST be named first, so that a file written for another
            //       schema says so before any key of that schema's shape is read.
            v("config_version", o.config_version);
            v("server",         o.server);
            v("camera",         o.camera);
            v("pose",           o.pose);
        )
    };

    // Where the app keeps its own output (`recordings`, `dumps`, `configs`):
    // the nearest such folder at or above the executable, else the executable's own directory.
    // Not config-relative, since these hold what the tool produces, not what an installation names.
    [[nodiscard]] std::filesystem::path project_dir(std::string_view name);

    // Every key the schema names must be present; a default standing in for an omitted one is the
    // failure this guards against. Keys it does not name are ignored.
    // On failure `err` names the key and `out` is untouched.
    [[nodiscard]] bool load_config(const std::filesystem::path& path, app_config_t& out, std::string& err);

    // Written from the schema, so the file that comes out holds the keys it names and nothing else.
    [[nodiscard]] bool save_config(const app_config_t& config, const std::filesystem::path& path, std::string& err);

    // The JSON text `save_config()` would write. (--dump-config)
    [[nodiscard]] std::string dump_config(const app_config_t& config);

    // Where `--config <name>` points: a bare filename comes from `project_dir("configs")`, the
    // folder a save writes to; a name carrying a folder is that path. Empty when nothing is there.
    [[nodiscard]] std::filesystem::path find_config_file(std::string_view name);

} // namespace app
