#include "vz_frame_source.hh"

#include <spdlog/spdlog.h>

#include <chrono>
#include <initializer_list>

namespace hw
{
    namespace
    {
        // How long one capture may take before the provider is handed nothing and loops. Long
        // enough for a slow trigger, short enough that `close()` is never stuck behind a grab.
        constexpr uint32_t kGrabTimeoutMs = 500;

        // How long an interface scan may take before the cameras on it are counted as absent.
        constexpr uint32_t kEnumerateTimeoutMs = 500;

        vz::frame_format_t to_vz_format(const frame_format_t format)
        {
            return (format == frame_format_t::gray8) ? vz::frame_format_t::gray
                                                     : vz::frame_format_t::bgr;
        }

        // The first of `names` the camera implements and can read. Cameras disagree on which
        // of the interchangeable GenICam names they carry, so each is tried in turn.
        std::optional<int64_t> read_first_int(
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

    bool vz_frame_source::open(const vz_device_config& config) noexcept try
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

        _color_format = config.color_format;
        _device.set_frame_format(to_vz_format(_color_format));

        // The calibration describes the sensor's whole readout, so the sensor extents are what
        // is wanted. `Width`/`Height` report the programmed window instead and come last, for
        // a camera that names no maximum.
        const std::optional<int64_t> sensor_w = read_first_int(_device, { "WidthMax", "SensorWidth", "Width" });
        const std::optional<int64_t> sensor_h = read_first_int(_device, { "HeightMax", "SensorHeight", "Height" });
        if (!sensor_w.has_value() || !sensor_h.has_value() || *sensor_w <= 0 || *sensor_h <= 0) {
            spdlog::error("vz: the camera reports no frame size");
            _device.close();
            return false;
        }
        const int width = static_cast<int>(*sensor_w);
        const int height = static_cast<int>(*sensor_h);

        // A readout window survives across opens, so one left from an earlier run would
        // silently narrow this one. Start whole and let `try_set_color_roi()` narrow it.
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
        _calib.color_resolution = Eigen::Vector2i{ width, height };

        // The camera carries no intrinsics of its own, so they arrive measured off-line. A set
        // measured on a different frame size would project plausibly and wrongly, which is the
        // hardest failure to notice, so it is refused rather than rescaled.
        if (config.color_intr.has_value()) {
            const intrinsic_t& intr = *config.color_intr;
            if (intr.width != width || intr.height != height) {
                spdlog::error("vz: the supplied intrinsics were measured on {}x{} but the sensor "
                              "is {}x{}; running without them"
                    , intr.width
                    , intr.height
                    , width
                    , height
                );
            }
            else {
                _calib.color_intr = intr;
                if (config.color_dist.has_value()) { _calib.color_dist = *config.color_dist; }
                _calib.color_fov = _calib.color_intr.get_fov();
            }
        }

        spdlog::info("vz: camera '{}' ready ({}x{}, {}, intrinsics {})"
            , serial
            , width
            , height
            , frame_format_to_str(_color_format)
            , _calib.color_intr.fx > 0.0f ? "supplied" : "absent"
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

    std::optional<roi_t> vz_frame_source::try_set_color_roi(const roi_t& roi)
    {
        std::scoped_lock lk{ _mtx };
        if (!_device.is_open()) { return std::nullopt; }

        const roi_t fitted = clamp_roi(roi, _calib.color_resolution.x(), _calib.color_resolution.y());
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
            _color_format,
            timestamp
        ) };
    }

} // namespace hw
