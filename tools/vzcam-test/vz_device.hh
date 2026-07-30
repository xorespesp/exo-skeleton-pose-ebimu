#pragma once
#include <opencv2/core.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace vz
{
    // Vieworks VZ camera control built on the Galaxy C++ API (`GxIAPICPPEx`).
    //
    // Implemented against (Win64):
    //   VZSolutionSDK package  2.1.1           `VZSolutionSDK.dll`
    //   Galaxy C++ API         2.0.2512.8261   `GxIAPICPPEx.dll`, headers `GalaxyIncludes.h` 1.2.2310.9241
    //   GenICam runtime        V3.0 (VC120)    bundled with the package
    //
    // The VZSolutionSDK ships a Vieworks wrapper (`VZCAM_API`) over that same Galaxy library,
    // but its headers do not compile under C++20 and it exposes acquisition as a push
    // callback only. The Galaxy layer underneath compiles clean and offers a blocking pull
    // (`IGXStream::GetImage`), which is what `hw::sensor_frame_source` is shaped around.
    //
    // The SDK headers are confined to the .cc through a pimpl: the Galaxy API reports
    // failures by throwing, so every entry point here converts that into a bool plus
    // `last_err_msg()`, matching the noexcept/bool style the sensor backends use.
    //
    // Acquisition is pull-based (`IGXStream::GetImage`) on a worker thread that publishes
    // the newest converted frame; the UI reads whatever is current without blocking.

    struct device_info_t
    {
        std::string vendor;
        std::string model;
        std::string serial;
        std::string display_name;
    };

    // Availability of a GenICam node, queried per access.
    struct feature_flags_t
    {
        bool implemented{ false };
        bool readable{ false };
        bool writable{ false };
    };

    struct float_feature_t
    {
        feature_flags_t flags;
        double value{ 0.0 };
        double min{ 0.0 };
        double max{ 0.0 };
        std::string unit;
    };

    struct int_feature_t
    {
        feature_flags_t flags;
        int64_t value{ 0 };
        int64_t min{ 0 };
        int64_t max{ 0 };
        int64_t inc{ 1 }; // permitted step; ROI fields are rarely settable to arbitrary values
    };

    struct enum_feature_t
    {
        feature_flags_t flags;
        std::string value;
        std::vector<std::string> entries;
    };

    // Region of interest in sensor pixels, as reported by the camera.
    struct roi_t
    {
        int64_t width{ 0 }, height{ 0 };
        int64_t offset_x{ 0 }, offset_y{ 0 };
    };

    // Output format of each capture.
    enum class frame_format_t { bgr, gray };

    struct stream_stats_t
    {
        uint64_t frames{ 0 };      // frames delivered with `GX_FRAME_STATUS_SUCCESS`
        uint64_t incomplete{ 0 };  // frames the transport reported as partial
        uint64_t timeouts{ 0 };    // `GetImage()` calls that expired
        double   fps{ 0.0 };       // EMA-smoothed delivery rate
        uint64_t payload_bytes{ 0 };
        uint32_t width{ 0 }, height{ 0 };
        std::string pixel_format;  // GenICam `PixelFormat` of the last frame
        double convert_ms{ 0.0 };  // EMA-smoothed demosaic cost per frame, in the active format
    };

    class device final
    {
    public:
        device();
        ~device();

        device(const device&) = delete;
        device& operator=(const device&) = delete;

        // Discovers cameras on every interface. Safe to call with none attached.
        [[nodiscard]] static std::vector<device_info_t> enumerate(
            uint32_t timeout_ms = 500,
            std::string* err_msg = nullptr/* out-opt */
        );

        [[nodiscard]] bool open(const std::string& serial);
        void close();
        bool is_open() const;

        [[nodiscard]] bool start_stream();
        void stop_stream();
        bool is_streaming() const;

        // Selects what `latest_frame()` delivers.
        // (safe to change while streaming; the worker picks it up on the next capture)
        void set_frame_format(frame_format_t mode);
        frame_format_t frame_format() const;

        // Newest capture in the configured format (8-bit BGR or 8-bit single channel);
        // empty until the first one arrives.
        cv::Mat latest_frame() const;

        stream_stats_t stats() const;

        // GenICam node access. Returns `nullopt` when the node is absent or unreadable.
        std::optional<float_feature_t> read_float(const char* name) const;
        std::optional<int_feature_t>   read_int(const char* name) const;
        std::optional<enum_feature_t>  read_enum(const char* name) const;
        std::optional<std::string>     read_string(const char* name) const;

        [[nodiscard]] bool write_float(const char* name, double value);
        [[nodiscard]] bool write_int(const char* name, int64_t value);
        [[nodiscard]] bool write_enum(const char* name, const std::string& value);
        [[nodiscard]] bool execute(const char* name);

        // `Width`/`Height` and `OffsetX`/`OffsetY` constrain each other: an extent's maximum is the
        // sensor size minus the current offset. Each axis is therefore written offset-first
        // (zeroed), then extent, then offset, and values are snapped to each node's `inc`.
        [[nodiscard]] bool set_roi(const roi_t& roi);
        std::optional<roi_t> read_roi() const;

        // Full node list of the remote device, for the feature explorer.
        std::vector<std::string> feature_names() const;

        // One line per node: "name  [type]  value". Nodes that fail to read are reported as such.
        std::vector<std::string> dump_features() const;

        std::string last_err_msg() const;

    private:
        struct impl_t;
        std::unique_ptr<impl_t> _impl;
    };

} // namespace vz
