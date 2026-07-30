#include "vz_device.hh"

#include <GalaxyIncludes.h>

#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <thread>

namespace vz
{
    namespace
    {
        // The Galaxy runtime is process-wide: `Init()` on the first `ensure_init()`, `Uninit()` at exit.
        class sdk_runtime_t {
        public:
            static void ensure_init() { static sdk_runtime_t rt; }

        private:
            sdk_runtime_t() { IGXFactory::GetInstance().Init(); }
            ~sdk_runtime_t() {
                try { IGXFactory::GetInstance().Uninit(); } catch (...) {}
            }
        };

        std::string make_exception_text(const CGalaxyException& e)
        {
            // `GetErrorCode()` is not const-qualified in the SDK. 
            // const-casting is safe here: the thrown object is not itself const, and the call just reads a member.
            const int code = const_cast<CGalaxyException&>(e).GetErrorCode();
            return std::string{ e.what() } + " (code " + std::to_string(code) + ")";
        }

        // `cvtColor` codes for one Bayer layout: to colour, and straight to grayscale.
        struct bayer_conversion_codes_t {
            cv::ColorConversionCodes to_bgr;
            cv::ColorConversionCodes to_gray;
        };

        // OpenCV names a Bayer pattern after the 2nd/3rd columns of the SECOND row, which is
        // the opposite corner from the GenICam name. Spelling these out avoids the classic
        // red/blue swap: GenICam `BayerRG` maps to `cv::COLOR_BayerBG2BGR`, and so on.
        // Empty for a format that is not Bayer at all.
        std::optional<bayer_conversion_codes_t> try_get_bayer_conversion_codes(GX_PIXEL_FORMAT_ENTRY fmt)
        {
            switch (fmt) {
            case GX_PIXEL_FORMAT_BAYER_RG8: case GX_PIXEL_FORMAT_BAYER_RG10:
            case GX_PIXEL_FORMAT_BAYER_RG12: case GX_PIXEL_FORMAT_BAYER_RG16:
                return bayer_conversion_codes_t{ cv::COLOR_BayerBG2BGR, cv::COLOR_BayerBG2GRAY };
            case GX_PIXEL_FORMAT_BAYER_GR8: case GX_PIXEL_FORMAT_BAYER_GR10:
            case GX_PIXEL_FORMAT_BAYER_GR12: case GX_PIXEL_FORMAT_BAYER_GR16:
                return bayer_conversion_codes_t{ cv::COLOR_BayerGB2BGR, cv::COLOR_BayerGB2GRAY };
            case GX_PIXEL_FORMAT_BAYER_GB8: case GX_PIXEL_FORMAT_BAYER_GB10:
            case GX_PIXEL_FORMAT_BAYER_GB12: case GX_PIXEL_FORMAT_BAYER_GB16:
                return bayer_conversion_codes_t{ cv::COLOR_BayerGR2BGR, cv::COLOR_BayerGR2GRAY };
            case GX_PIXEL_FORMAT_BAYER_BG8: case GX_PIXEL_FORMAT_BAYER_BG10:
            case GX_PIXEL_FORMAT_BAYER_BG12: case GX_PIXEL_FORMAT_BAYER_BG16:
                return bayer_conversion_codes_t{ cv::COLOR_BayerRG2BGR, cv::COLOR_BayerRG2GRAY };
            default:
                return std::nullopt;
            }
        }

        // `GX_PIXEL_FORMAT_ENTRY` packs the bit depth into bits 16..23 (`GX_PIXEL_8BIT` is
        // 0x00080000, `GX_PIXEL_10BIT` is 0x000A0000, and so on), so the count reads out directly.
        int pixel_format_bit_depth(GX_PIXEL_FORMAT_ENTRY fmt)
        {
            return (static_cast<int>(fmt) >> 16) & 0xFF;
        }

        // Bring a requested value into the node's range and down onto its increment grid.
        int64_t clamp_and_snap_to_increment(int64_t value, int64_t min, int64_t max, int64_t inc)
        {
            if (inc <= 0) { inc = 1; }
            value = std::clamp(value, min, max);
            return min + ((value - min) / inc) * inc;
        }

    } // namespace

    // ---------------------------------------------------------------------------------------

    struct device::impl_t
    {
        CGXDevicePointer         dev;
        CGXStreamPointer         stream;
        CGXFeatureControlPointer fc;

        std::thread       worker;
        std::atomic<bool> running{ false };

        mutable std::mutex frame_mtx;
        cv::Mat            frame; // newest capture, in `format`
        std::atomic<frame_format_t> format{ frame_format_t::bgr };

        mutable std::mutex stats_mtx;
        stream_stats_t     stats;

        mutable std::mutex err_msg_mtx;
        std::string        err_msg;

        void set_err_msg(const std::string& text)
        {
            std::scoped_lock lk{ err_msg_mtx };
            err_msg = text;
        }
    };

    device::device() : _impl{ std::make_unique<impl_t>() } {}

    device::~device()
    {
        this->close();
    }

    std::vector<device_info_t> device::enumerate(uint32_t timeout_ms, std::string* err_msg)
    {
        std::vector<device_info_t> out;
        if (err_msg) { err_msg->clear(); }
        try {
            sdk_runtime_t::ensure_init();
            GxIAPICPP::gxdeviceinfo_vector infos;
            IGXFactory::GetInstance().UpdateAllDeviceList(timeout_ms, infos);
            out.reserve(infos.size());
            for (size_t i = 0; i < infos.size(); ++i) {
                out.push_back(device_info_t{
                    infos[i].GetVendorName().c_str(),
                    infos[i].GetModelName().c_str(),
                    infos[i].GetSN().c_str(),
                    infos[i].GetDisplayName().c_str(),
                });
            }
        }
        catch (const CGalaxyException& e) { if (err_msg) { *err_msg = make_exception_text(e); } }
        catch (const std::exception& e) { if (err_msg) { *err_msg = e.what(); } }
        return out;
    }

    bool device::open(const std::string& serial)
    {
        if (this->is_open()) { this->close(); }
        try {
            sdk_runtime_t::ensure_init();
            _impl->dev = IGXFactory::GetInstance().OpenDeviceBySN(
                serial.c_str(), GX_ACCESS_EXCLUSIVE);
            _impl->fc = _impl->dev->GetRemoteFeatureControl();
            spdlog::info("vz: opened device sn={}", serial);
            return true;
        }
        catch (const CGalaxyException& e) {
            _impl->set_err_msg(make_exception_text(e));
            spdlog::error("vz: open failed: {}", make_exception_text(e));
            _impl->dev = CGXDevicePointer{};
            return false;
        }
    }

    void device::close()
    {
        this->stop_stream();
        try {
            if (!_impl->dev.IsNull()) { _impl->dev->Close(); }
        }
        catch (const CGalaxyException& e) { _impl->set_err_msg(make_exception_text(e)); }
        _impl->fc = CGXFeatureControlPointer{};
        _impl->dev = CGXDevicePointer{};

        std::scoped_lock lk{ _impl->frame_mtx };
        _impl->frame.release();
    }

    bool device::is_open() const { return !_impl->dev.IsNull(); }
    bool device::is_streaming() const { return _impl->running.load(); }

    bool device::start_stream()
    {
        if (!this->is_open() || this->is_streaming()) { return false; }
        try {
            _impl->stream = _impl->dev->OpenStream(0);

            {
                std::scoped_lock lk{ _impl->stats_mtx };
                _impl->stats = stream_stats_t{};
                _impl->stats.payload_bytes = _impl->stream->GetPayloadSize();
            }

            _impl->stream->StartGrab();
            _impl->fc->GetCommandFeature("AcquisitionStart")->Execute();
            _impl->running.store(true);

            _impl->worker = std::thread([this] {
                using clock = std::chrono::steady_clock;
                auto last = clock::now();

                while (_impl->running.load()) {
                    CImageDataPointer img;
                    try {
                        img = _impl->stream->GetImage(500);
                    }
                    catch (const CGalaxyException&) {
                        std::scoped_lock lk{ _impl->stats_mtx };
                        ++_impl->stats.timeouts;
                        continue;
                    }
                    if (img.IsNull()) { continue; }

                    if (img->GetStatus() != GX_FRAME_STATUS_SUCCESS) {
                        std::scoped_lock lk{ _impl->stats_mtx };
                        ++_impl->stats.incomplete;
                        continue;
                    }

                    const auto t0 = clock::now();
                    const int w = static_cast<int>(img->GetWidth());
                    const int h = static_cast<int>(img->GetHeight());
                    const GX_PIXEL_FORMAT_ENTRY fmt = img->GetPixelFormat();
                    const int depth = pixel_format_bit_depth(fmt);

                    // Wraps the SDK buffer without copying; the conversion below owns its output.
                    cv::Mat raw(h, w, depth > 8 ? CV_16UC1 : CV_8UC1, img->GetBuffer());
                    cv::Mat raw8;
                    if (depth > 8) { raw.convertTo(raw8, CV_8U, 1.0 / (1 << (depth - 8))); }
                    else { raw8 = raw; }

                    const std::optional<bayer_conversion_codes_t> bayer_codes = try_get_bayer_conversion_codes(fmt);
                    const bool want_gray = _impl->format.load() == frame_format_t::gray;

                    cv::Mat out;
                    if (bayer_codes.has_value()) {
                        cv::cvtColor(raw8, out, want_gray ? bayer_codes->to_gray : bayer_codes->to_bgr);
                    } else if (raw8.channels() == 1) {
                        if (want_gray) { out = raw8.clone(); }
                        else { cv::cvtColor(raw8, out, cv::COLOR_GRAY2BGR); }
                    } else {
                        if (want_gray) { cv::cvtColor(raw8, out, cv::COLOR_BGR2GRAY); }
                        else { out = raw8.clone(); }
                    }

                    const double convert_ms =
                        std::chrono::duration<double, std::milli>(clock::now() - t0).count();

                    {
                        std::scoped_lock lk{ _impl->frame_mtx };
                        _impl->frame = std::move(out);
                    }
                    {
                        const auto now = clock::now();
                        const double dt = std::chrono::duration<double>(now - last).count();
                        last = now;

                        std::scoped_lock lk{ _impl->stats_mtx };
                        ++_impl->stats.frames;
                        _impl->stats.width = static_cast<uint32_t>(w);
                        _impl->stats.height = static_cast<uint32_t>(h);
                        _impl->stats.convert_ms = _impl->stats.convert_ms * 0.9 + convert_ms * 0.1;
                        if (dt > 0.0 && dt < 5.0) {
                            const double inst = 1.0 / dt;
                            _impl->stats.fps = (_impl->stats.fps <= 0.0)
                                ? inst : (_impl->stats.fps * 0.9 + inst * 0.1);
                        }
                    }
                }
            });

            spdlog::info("vz: acquisition started (payload {} bytes)", this->stats().payload_bytes);
            return true;
        }
        catch (const CGalaxyException& e) {
            _impl->set_err_msg(make_exception_text(e));
            spdlog::error("vz: start_stream failed: {}", make_exception_text(e));
            _impl->running.store(false);
            _impl->stream = CGXStreamPointer{};
            return false;
        }
    }

    void device::stop_stream()
    {
        if (!_impl->running.exchange(false)) {
            _impl->stream = CGXStreamPointer{};
            return;
        }
        if (_impl->worker.joinable()) { _impl->worker.join(); }

        try {
            if (!_impl->fc.IsNull()) { _impl->fc->GetCommandFeature("AcquisitionStop")->Execute(); }
            if (!_impl->stream.IsNull()) { _impl->stream->StopGrab(); _impl->stream->Close(); }
            spdlog::info("vz: acquisition stopped");
        }
        catch (const CGalaxyException& e) { _impl->set_err_msg(make_exception_text(e)); }
        _impl->stream = CGXStreamPointer{};
    }

    void device::set_frame_format(frame_format_t mode) { _impl->format.store(mode); }
    frame_format_t device::frame_format() const { return _impl->format.load(); }

    cv::Mat device::latest_frame() const
    {
        std::scoped_lock lk{ _impl->frame_mtx };
        return _impl->frame;
    }

    stream_stats_t device::stats() const
    {
        std::scoped_lock lk{ _impl->stats_mtx };
        stream_stats_t s = _impl->stats;
        if (!_impl->fc.IsNull()) {
            try {
                if (_impl->fc->IsImplemented("PixelFormat") && _impl->fc->IsReadable("PixelFormat")) {
                    s.pixel_format = _impl->fc->GetEnumFeature("PixelFormat")->GetValue().c_str();
                }
            }
            catch (const CGalaxyException&) {}
        }
        return s;
    }

    std::string device::last_err_msg() const
    {
        std::scoped_lock lk{ _impl->err_msg_mtx };
        return _impl->err_msg;
    }

    // ---------------------------------------------------------------------------------------
    // GenICam node access
    // ---------------------------------------------------------------------------------------

    namespace
    {
        // Queries only; non-const because the SDK's `Is*()` are not const-qualified.
        feature_flags_t probe(CGXFeatureControlPointer& fc_ptr, const char* name)
        {
            feature_flags_t f{};
            f.implemented = fc_ptr->IsImplemented(name);
            if (f.implemented) {
                f.readable = fc_ptr->IsReadable(name);
                f.writable = fc_ptr->IsWritable(name);
            }
            return f;
        }
    } // namespace

    std::optional<float_feature_t> device::read_float(const char* name) const
    {
        if (_impl->fc.IsNull()) { return std::nullopt; }
        try {
            const feature_flags_t f = probe(_impl->fc, name);
            if (!f.implemented || !f.readable) { return std::nullopt; }
            CFloatFeaturePointer p = _impl->fc->GetFloatFeature(name);
            return float_feature_t{ f, p->GetValue(), p->GetMin(), p->GetMax(), p->GetUnit().c_str() };
        }
        catch (const CGalaxyException& e) { _impl->set_err_msg(make_exception_text(e)); return std::nullopt; }
    }

    std::optional<int_feature_t> device::read_int(const char* name) const
    {
        if (_impl->fc.IsNull()) { return std::nullopt; }
        try {
            const feature_flags_t f = probe(_impl->fc, name);
            if (!f.implemented || !f.readable) { return std::nullopt; }
            CIntFeaturePointer p = _impl->fc->GetIntFeature(name);
            return int_feature_t{ f, p->GetValue(), p->GetMin(), p->GetMax(), p->GetInc() };
        }
        catch (const CGalaxyException& e) { _impl->set_err_msg(make_exception_text(e)); return std::nullopt; }
    }

    std::optional<enum_feature_t> device::read_enum(const char* name) const
    {
        if (_impl->fc.IsNull()) { return std::nullopt; }
        try {
            const feature_flags_t f = probe(_impl->fc, name);
            if (!f.implemented || !f.readable) { return std::nullopt; }
            CEnumFeaturePointer p = _impl->fc->GetEnumFeature(name);

            enum_feature_t out{};
            out.flags = f;
            out.value = p->GetValue().c_str();
            GxIAPICPP::gxstring_vector entries = p->GetEnumEntryList();
            out.entries.reserve(entries.size());
            for (size_t i = 0; i < entries.size(); ++i) { out.entries.emplace_back(entries[i].c_str()); }
            return out;
        }
        catch (const CGalaxyException& e) { _impl->set_err_msg(make_exception_text(e)); return std::nullopt; }
    }

    std::optional<std::string> device::read_string(const char* name) const
    {
        if (_impl->fc.IsNull()) { return std::nullopt; }
        try {
            const feature_flags_t f = probe(_impl->fc, name);
            if (!f.implemented || !f.readable) { return std::nullopt; }
            return std::string{ _impl->fc->GetStringFeature(name)->GetValue().c_str() };
        }
        catch (const CGalaxyException& e) { _impl->set_err_msg(make_exception_text(e)); return std::nullopt; }
    }

    bool device::write_float(const char* name, double value)
    {
        if (_impl->fc.IsNull()) { return false; }
        try { _impl->fc->GetFloatFeature(name)->SetValue(value); return true; }
        catch (const CGalaxyException& e) {
            _impl->set_err_msg(make_exception_text(e));
            spdlog::warn("vz: set {}={} failed: {}", name, value, e.what());
            return false;
        }
    }

    bool device::write_int(const char* name, int64_t value)
    {
        if (_impl->fc.IsNull()) { return false; }
        try { _impl->fc->GetIntFeature(name)->SetValue(value); return true; }
        catch (const CGalaxyException& e) {
            _impl->set_err_msg(make_exception_text(e));
            spdlog::warn("vz: set {}={} failed: {}", name, value, e.what());
            return false;
        }
    }

    bool device::write_enum(const char* name, const std::string& value)
    {
        if (_impl->fc.IsNull()) { return false; }
        try { _impl->fc->GetEnumFeature(name)->SetValue(value.c_str()); return true; }
        catch (const CGalaxyException& e) {
            _impl->set_err_msg(make_exception_text(e));
            spdlog::warn("vz: set {}={} failed: {}", name, value, e.what());
            return false;
        }
    }

    bool device::execute(const char* name)
    {
        if (_impl->fc.IsNull()) { return false; }
        try { _impl->fc->GetCommandFeature(name)->Execute(); return true; }
        catch (const CGalaxyException& e) {
            _impl->set_err_msg(make_exception_text(e));
            spdlog::warn("vz: execute {} failed: {}", name, e.what());
            return false;
        }
    }

    std::optional<roi_t> device::read_roi() const
    {
        const auto w = this->read_int("Width");
        const auto h = this->read_int("Height");
        if (!w || !h) { return std::nullopt; }
        const auto ox = this->read_int("OffsetX");
        const auto oy = this->read_int("OffsetY");
        return roi_t{ w->value, h->value, ox ? ox->value : 0, oy ? oy->value : 0 };
    }

    bool device::set_roi(const roi_t& roi)
    {
        if (_impl->fc.IsNull()) { return false; }

        // `Width`'s maximum is the sensor width minus `OffsetX`, and `Height`'s likewise, so an
        // extent only opens up to the full sensor once its offset sits at zero. Writing each
        // axis as offset-then-extent-then-offset therefore satisfies both a shrinking and a
        // growing request with a single order, and reads each range when it is at its widest.
        bool ok = true;
        const auto apply_axis = [&](const char* extent_name, const char* offset_name,
                                    int64_t want_extent, int64_t want_offset)
        {
            if (const auto off = this->read_int(offset_name)) {
                ok = this->write_int(offset_name, off->min) && ok;
            }
            if (const auto ext = this->read_int(extent_name)) {
                ok = this->write_int(extent_name,
                    clamp_and_snap_to_increment(want_extent, ext->min, ext->max, ext->inc)) && ok;
            }
            if (const auto off = this->read_int(offset_name)) {
                ok = this->write_int(offset_name,
                    clamp_and_snap_to_increment(want_offset, off->min, off->max, off->inc)) && ok;
            }
        };

        apply_axis("Width", "OffsetX", roi.width, roi.offset_x);
        apply_axis("Height", "OffsetY", roi.height, roi.offset_y);
        return ok;
    }

    std::vector<std::string> device::feature_names() const
    {
        std::vector<std::string> out;
        if (_impl->fc.IsNull()) { return out; }
        try {
            GxIAPICPP::gxstring_vector names;
            _impl->fc->GetFeatureNameList(names);
            out.reserve(names.size());
            for (size_t i = 0; i < names.size(); ++i) { out.emplace_back(names[i].c_str()); }
        }
        catch (const CGalaxyException& e) { _impl->set_err_msg(make_exception_text(e)); }
        return out;
    }

    std::vector<std::string> device::dump_features() const
    {
        std::vector<std::string> out;
        if (_impl->fc.IsNull()) { return out; }

        for (const std::string& name : this->feature_names()) {
            try {
                if (!_impl->fc->IsImplemented(name.c_str())) {
                    out.push_back(name + "  [not implemented]");
                    continue;
                }
                if (!_impl->fc->IsReadable(name.c_str())) {
                    out.push_back(name + "  [not readable]");
                    continue;
                }
                const char* w = _impl->fc->IsWritable(name.c_str()) ? "rw" : "ro";
                switch (_impl->fc->GetFeatureType(name.c_str())) {
                case GX_FEATURE_INT: {
                    CIntFeaturePointer p = _impl->fc->GetIntFeature(name.c_str());
                    out.push_back(name + "  [int " + w + "]  " + std::to_string(p->GetValue())
                        + "  min=" + std::to_string(p->GetMin())
                        + " max=" + std::to_string(p->GetMax())
                        + " inc=" + std::to_string(p->GetInc()));
                    break;
                }
                case GX_FEATURE_FLOAT: {
                    CFloatFeaturePointer p = _impl->fc->GetFloatFeature(name.c_str());
                    out.push_back(name + "  [float " + w + "]  " + std::to_string(p->GetValue())
                        + "  min=" + std::to_string(p->GetMin())
                        + " max=" + std::to_string(p->GetMax())
                        + " unit=" + p->GetUnit().c_str());
                    break;
                }
                case GX_FEATURE_ENUM: {
                    CEnumFeaturePointer p = _impl->fc->GetEnumFeature(name.c_str());
                    std::string line = name + "  [enum " + w + "]  " + p->GetValue().c_str() + "  {";
                    GxIAPICPP::gxstring_vector es = p->GetEnumEntryList();
                    for (size_t i = 0; i < es.size(); ++i) {
                        line += (i ? " " : "");
                        line += es[i].c_str();
                    }
                    out.push_back(line + "}");
                    break;
                }
                case GX_FEATURE_BOOL: {
                    CBoolFeaturePointer p = _impl->fc->GetBoolFeature(name.c_str());
                    out.push_back(name + "  [bool " + w + "]  " + (p->GetValue() ? "true" : "false"));
                    break;
                }
                case GX_FEATURE_STRING: {
                    CStringFeaturePointer p = _impl->fc->GetStringFeature(name.c_str());
                    out.push_back(name + "  [string " + w + "]  " + p->GetValue().c_str());
                    break;
                }
                case GX_FEATURE_COMMAND:
                    out.push_back(name + "  [command " + w + "]");
                    break;
                default:
                    out.push_back(name + "  [other]");
                    break;
                }
            }
            catch (const CGalaxyException& e) {
                out.push_back(name + "  [error] " + e.what());
            }
        }
        return out;
    }

} // namespace vz
