#include "vz_frame_source.hh"

#include <spdlog/spdlog.h>

#include <chrono>
#include <format>
#include <initializer_list>
#include <optional>
#include <string>

namespace hw
{
    namespace
    {
        // How long one capture may take before the provider is handed nothing and loops. Long
        // enough for a slow trigger, short enough that `close()` is never stuck behind a grab.
        constexpr uint32_t kGrabTimeoutMs = 500;

        // How long an interface scan may take before the cameras on it are counted as absent.
        constexpr uint32_t kEnumerateTimeoutMs = 500;

        // Which of the camera's own illuminant presets the channel ratios come from.
        constexpr const char* kLightSourcePreset = "Daylight6500K";

        vz::frame_format_t to_vz_format(const frame_format_t format)
        {
            return (format == frame_format_t::gray8) ? vz::frame_format_t::gray
                                                     : vz::frame_format_t::bgr;
        }

        // Whether a GenICam pixel format name carries colour.
        bool pixel_format_carries_color(const std::string& name)
        {
            // Check monochrome 
            return !name.starts_with("Mono");
        }

        // GenICam nodes vary by model, so one this camera does not carry is a knob it does not have
        // and nothing that went wrong.
        void pin_enum_node_if_present(vz::device& device, const char* name, const char* value)
        {
            if (!device.read_enum(name).has_value()) { return; }
            if (!device.write_enum(name, value)) {
                spdlog::warn("vz: could not set {} to {}: {}", name, value, device.last_err_msg());
            }
        }

        void pin_bool_node_if_present(vz::device& device, const char* name, const bool value)
        {
            if (!device.read_bool(name).has_value()) { return; }
            if (!device.write_bool(name, value)) {
                spdlog::warn("vz: could not set {} to {}: {}", name, value, device.last_err_msg());
            }
        }

        // Settles the camera's colour processing, so every open starts from the same response.
        //
        // These nodes survive power cycles, so a value an earlier session left behind reaches this
        // one. Each goes to a state the camera itself names: the chroma from the illuminant preset,
        // everything else from its "do nothing" setting. Pinned at any frame format, since they also
        // reach the luminance a gray conversion produces.
        //
        // TODO: fixed here because no installation has asked for a state of its own. Should one,
        // these belong beside exposure and gain in the profile.
        void pin_persistent_color_response(vz::device& device)
        {
            // Auto rewrites the channel ratios frame by frame, which a fitted model cannot follow.
            // Off leaves them where they were last driven, so the preset states them right after.
            pin_enum_node_if_present(device, "BalanceWhiteAuto", "Off");
            pin_enum_node_if_present(device, "LightSourcePreset", kLightSourcePreset);

            // Gamma bends each channel by an amount that depends on how bright it already is, 
            // which moves a* and b* without moving the marker.
            pin_bool_node_if_present(device, "GammaEnable", false);

            // Saturation scales chroma outright; 
            // the luminance lookup and the digital shift reach it by another route.
            pin_enum_node_if_present(device, "SaturationMode", "Off");
            pin_bool_node_if_present(device, "LUTEnable", false);
            if (device.read_int("DigitalShift").has_value() && !device.write_int("DigitalShift", 0)) {
                spdlog::warn("vz: could not clear the digital shift: {}", device.last_err_msg());
            }

            // Not colour, but the same leftover: a mirrored readout turns the leg upside down in the
            // frame, and the colour markers are named by their order down it.
            pin_bool_node_if_present(device, "ReverseX", false);
            pin_bool_node_if_present(device, "ReverseY", false);

            // What is left unpinned, said out loud. `BlackLevel` is an offset added before
            // digitisation, so raising it compresses chroma toward neutral, and the camera names no
            // neutral value for it to be pinned to.
            std::string ratios;
            if (const std::optional<vz::enum_feature_t> selector = device.read_enum("BalanceRatioSelector"))
            {
                for (const std::string& channel : selector->entries)
                {
                    if (!device.write_enum("BalanceRatioSelector", channel)) { continue; }
                    const std::optional<vz::float_feature_t> r = device.read_float("BalanceRatio");
                    if (!r.has_value()) { continue; }
                    if (!ratios.empty()) { ratios += " "; }
                    ratios += std::format("{}={:.3f}", channel, r->value);
                }
            }
            const std::optional<vz::float_feature_t> black = device.read_float("BlackLevel");
            spdlog::info("vz: color response settled (preset {}, {}, black level {})"
                , kLightSourcePreset
                , ratios.empty() ? std::string{ "no balance ratios" } : ratios
                , black.has_value() ? std::format("{:.1f}", black->value) : std::string{ "n/a" });
        }

        // Cameras disagree on which of the interchangeable GenICam names they carry, 
        // so each is tried in turn.
        std::optional<int64_t> read_first_readable_int_node(
            const vz::device& device,
            std::initializer_list<const char*> names)
        {
            for (const char* name : names) {
                if (const std::optional<vz::int_feature_t> f = device.read_int(name)) {
                    return f->value;
                }
            }
            return std::nullopt;
        }
    } // namespace

    vz_frame_source::~vz_frame_source()
    {
        this->close();
    }

    bool vz_frame_source::open(const vz_device_config_t& config) noexcept try
    {
        std::scoped_lock lk{ _mtx };

        // An index names a position in the enumeration, so the cameras have to be listed
        // before it means anything. The order is the SDK's and moves as cameras come and go.
        std::string err_msg;
        const std::vector<vz::device_info_t> found = vz::device::enumerate(kEnumerateTimeoutMs, &err_msg);
        if (found.empty()) {
            spdlog::error("vz: no camera found{}{}", err_msg.empty() ? "" : ": ", err_msg);
            return false;
        }
        if (config.device_index >= found.size()) {
            spdlog::error("vz: camera #{} was asked for, but {} camera(s) are attached"
                , config.device_index
                , found.size()
            );
            return false;
        }

        const std::string& serial = found[config.device_index].serial;
        if (!_device.open(serial)) { return false; }

        _frame_format = config.frame_format;
        _device.set_frame_format(to_vz_format(_frame_format));

        // Colour classification needs the sensor to deliver colour. A monochrome format debayers to
        // R=G=B, which reads as a valid three-channel frame the whole way to the detector and
        // measures nothing, so the stream is refused here where the cause is still legible.
        if (_frame_format != frame_format_t::gray8)
        {
            if (const std::optional<vz::enum_feature_t> pf = _device.read_enum("PixelFormat");
                pf.has_value() && !pixel_format_carries_color(pf->value))
            {
                spdlog::error("vz: the camera streams '{}', which carries no color; set a color "
                              "pixel format on it before opening a color-marker profile", pf->value);
                _device.close();
                return false;
            }
        }

        pin_persistent_color_response(_device);

        // The calibration describes the sensor's whole readout, so the sensor extents are what
        // is wanted. `Width`/`Height` report the programmed window instead and come last, for
        // a camera that names no maximum.
        const std::optional<int64_t> sensor_w = read_first_readable_int_node(_device, { "WidthMax", "SensorWidth", "Width" });
        const std::optional<int64_t> sensor_h = read_first_readable_int_node(_device, { "HeightMax", "SensorHeight", "Height" });
        if (!sensor_w.has_value() || !sensor_h.has_value() || *sensor_w <= 0 || *sensor_h <= 0) {
            spdlog::error("vz: the camera reports no frame size");
            _device.close();
            return false;
        }
        const int width = static_cast<int>(*sensor_w);
        const int height = static_cast<int>(*sensor_h);

        // A readout window survives across opens, so one left from an earlier run would
        // silently narrow this one. Start whole and let `try_set_roi()` narrow it.
        if (!_device.set_roi(vz::roi_t{ .width = *sensor_w, .height = *sensor_h })) {
            spdlog::warn("vz: could not reset the readout window: {}", _device.last_err_msg());
        }

        // Left unset, the camera keeps whatever it was last given. A manual value only holds
        // with the matching auto mode off, which is a separate node: with it left on, the
        // camera overwrites what was just written on the next frame.
        if (config.exposure_us.has_value()) {
            if (!_device.write_enum("ExposureAuto", "Off")
                || !_device.write_float("ExposureTime", *config.exposure_us))
            {
                spdlog::warn("vz: could not set the exposure: {}", _device.last_err_msg());
            }
        }
        if (config.gain.has_value()) {
            if (!_device.write_enum("GainAuto", "Off")
                || !_device.write_float("Gain", *config.gain))
            {
                spdlog::warn("vz: could not set the gain: {}", _device.last_err_msg());
            }
        }

        _calib = calibration_t{};
        _calib.frame_resolution = Eigen::Vector2i{ width, height };

        // The camera carries no intrinsics of its own, so they arrive measured off-line. A set
        // measured on a different frame size would project plausibly and wrongly, which is the
        // hardest failure to notice, so it is refused rather than rescaled.
        if (config.intrinsic.has_value()) {
            const intrinsic_t& intr = *config.intrinsic;
            if (intr.calib_resolution != _calib.frame_resolution) {
                spdlog::error("vz: the supplied intrinsics were measured on {}x{} but the sensor "
                              "is {}x{}; running without them"
                    , intr.calib_resolution.x()
                    , intr.calib_resolution.y()
                    , width
                    , height
                );
            }
            else {
                _calib.intrinsic = intr;
                if (config.distortion.has_value()) { _calib.distortion = *config.distortion; }
            }
        }

        spdlog::info("vz: camera '{}' ready ({}x{}, {}, intrinsics {})"
            , serial
            , width
            , height
            , frame_format_to_str(_frame_format)
            , _calib.intrinsic.fx > 0.0f ? "supplied" : "absent"
        );
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("vz_frame_source::open failed: {}", e.what());
        return false;
    }

    bool vz_frame_source::is_valid() const
    {
        std::scoped_lock lk{ _mtx };
        return _device.is_open();
    }

    void vz_frame_source::close()
    {
        std::scoped_lock lk{ _mtx };
        _device.close();
    }

    std::optional<roi_t> vz_frame_source::try_set_roi(const roi_t& roi)
    {
        std::scoped_lock lk{ _mtx };
        if (!_device.is_open()) { return std::nullopt; }

        const roi_t fitted = clamp_roi(roi, _calib.frame_resolution.x(), _calib.frame_resolution.y());
        if (fitted.is_empty()) {
            spdlog::warn("vz: the requested ROI leaves nothing of the frame");
            return std::nullopt;
        }

        // The readout window is part of the acquisition setup and is locked while frames are
        // in flight, so a live stream steps aside for the write and picks up again after.
        const bool was_streaming = _device.is_streaming();
        if (was_streaming) { _device.stop_stream(); }

        const bool written = _device.set_roi(vz::roi_t{
            .width = fitted.width,
            .height = fitted.height,
            .offset_x = fitted.x,
            .offset_y = fitted.y,
        });
        if (!written) {
            spdlog::warn("vz: the camera refused the readout window: {}", _device.last_err_msg());
        }

        const std::optional<vz::roi_t> granted = _device.read_roi();
        if (was_streaming && !_device.start_stream()) {
            spdlog::error("vz: acquisition did not resume after the readout window was written");
        }
        if (!written || !granted.has_value()) { return std::nullopt; }

        return roi_t{
            .x = static_cast<int>(granted->offset_x),
            .y = static_cast<int>(granted->offset_y),
            .width = static_cast<int>(granted->width),
            .height = static_cast<int>(granted->height),
        };
    }

    std::optional<sensor_frameset> vz_frame_source::fetch_next_sensor_frameset()
    {
        std::scoped_lock lk{ _mtx };
        if (!_device.is_open()) { return std::nullopt; }

        // Acquisition is deferred to here, which leaves the readout window writable for as
        // long as nobody has asked for a frame.
        if (!_device.is_streaming() && !_device.start_stream()) {
            return std::nullopt;
        }

        std::optional<vz::captured_frame_t> captured = _device.grab_frame(kGrabTimeoutMs);
        if (!captured.has_value() || captured->image.empty()) { return std::nullopt; }

        // The camera's clock is smooth but has an epoch of its own; the anchor lifts it onto
        // Unix time without touching the spacing between frames. Where the camera offers no
        // clock, arrival time is all there is and the host's scheduling shows up in it.
        const timestamp_t timestamp = captured->device_timestamp.has_value()
            ? _clock_anchor.to_unix(*captured->device_timestamp)
            : std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());

        return sensor_frameset{ std::make_shared<sensor_frame>(
            std::move(captured->image),
            _frame_format,
            timestamp
        ) };
    }

} // namespace hw
