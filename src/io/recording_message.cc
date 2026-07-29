#include "recording_message.hh"

#include <CameraCalibration_generated.h>
#include <CompressedImage_generated.h>
#include <RawImage_generated.h>

#include <CameraCalibration_bfbs.h>
#include <CompressedImage_bfbs.h>
#include <RawImage_bfbs.h>

#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>

namespace io
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        // Foxglove's RawImage encoding name for CV_8UC3 in OpenCV's channel order.
        constexpr std::string_view kRawEncoding{ "bgr8" };

        // Foxglove's CompressedImage format name for what cv::imencode(".jpg") produces.
        constexpr std::string_view kJpegFormat{ "jpeg" };

        // OpenCV's rational Brown-Conrady model, which is what hw::distortion_t holds.
        constexpr std::string_view kDistortionModel{ "rational_polynomial" };

        foxglove::Time to_foxglove_time(std::chrono::nanoseconds timestamp) noexcept
        {
            const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
            const auto nsec = timestamp - sec;
            return foxglove::Time{
                static_cast<uint32_t>(sec.count()),
                static_cast<uint32_t>(nsec.count())
            };
        }

        std::vector<std::byte> to_bytes(const flatbuffers::FlatBufferBuilder& builder)
        {
            const auto* data = reinterpret_cast<const std::byte*>(builder.GetBufferPointer());
            return std::vector<std::byte>{ data, data + builder.GetSize() };
        }

        // Rejects payloads that do not verify as the given root type, so a malformed or
        // foreign recording produces an empty frame instead of a read past the buffer.
        template <typename VerifyFn>
        bool verify_payload(std::span<const std::byte> payload, VerifyFn verify) noexcept
        {
            flatbuffers::Verifier verifier{
                reinterpret_cast<const uint8_t*>(payload.data()), payload.size()
            };
            return verify(verifier);
        }

        std::vector<std::byte> encode_jpeg_frame(
            const cv::Mat& bgr,
            std::chrono::nanoseconds timestamp,
            std::string_view frame_id,
            const encode_options& options)
        {
            std::vector<uint8_t> jpeg;
            const std::vector<int> params{ cv::IMWRITE_JPEG_QUALITY, options.jpeg_quality };
            if (!cv::imencode(".jpg", bgr, jpeg, params)) {
                throw std::runtime_error{ "encode_frame: jpeg encoding failed" };
            }

            flatbuffers::FlatBufferBuilder builder;
            const auto data = builder.CreateVector(jpeg);
            const auto frame = builder.CreateString(frame_id);
            const auto format = builder.CreateString(kJpegFormat);
            const foxglove::Time time = to_foxglove_time(timestamp);

            builder.Finish(foxglove::CreateCompressedImage(builder, &time, frame, data, format));
            return to_bytes(builder);
        }

        std::vector<std::byte> encode_raw_frame(
            const cv::Mat& bgr,
            std::chrono::nanoseconds timestamp,
            std::string_view frame_id)
        {
            // CreateVector needs one contiguous run; a view into a larger Mat is not.
            const cv::Mat packed = bgr.isContinuous() ? bgr : bgr.clone();
            const auto step = static_cast<uint32_t>(packed.cols * packed.elemSize());

            flatbuffers::FlatBufferBuilder builder;
            const auto data = builder.CreateVector(
                packed.ptr<uint8_t>(), static_cast<size_t>(step) * packed.rows);
            const auto frame = builder.CreateString(frame_id);
            const auto encoding = builder.CreateString(kRawEncoding);
            const foxglove::Time time = to_foxglove_time(timestamp);

            builder.Finish(foxglove::CreateRawImage(
                builder, &time, frame,
                static_cast<uint32_t>(packed.cols), static_cast<uint32_t>(packed.rows),
                encoding, step, data));
            return to_bytes(builder);
        }

        cv::Mat decode_jpeg_frame(std::span<const std::byte> payload)
        {
            if (!verify_payload(payload, [](flatbuffers::Verifier& v) {
                return foxglove::VerifyCompressedImageBuffer(v);
            })) {
                throw std::runtime_error{ "not a valid CompressedImage message" };
            }

            const auto* image = foxglove::GetCompressedImage(payload.data());
            const auto* data = image->data();
            if (!data) { throw std::runtime_error{ "CompressedImage has no data" }; }

            const cv::Mat encoded{
                1, static_cast<int>(data->size()), CV_8UC1,
                const_cast<uint8_t*>(data->data())
            };
            cv::Mat bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
            if (bgr.empty()) { throw std::runtime_error{ "jpeg decoding failed" }; }
            return bgr;
        }

        cv::Mat decode_raw_frame(std::span<const std::byte> payload)
        {
            if (!verify_payload(payload, [](flatbuffers::Verifier& v) {
                return foxglove::VerifyRawImageBuffer(v);
            })) {
                throw std::runtime_error{ "not a valid RawImage message" };
            }

            const auto* image = foxglove::GetRawImage(payload.data());
            const auto* data = image->data();
            if (!data) { throw std::runtime_error{ "RawImage has no data" }; }

            const auto encoding = image->encoding() ? image->encoding()->string_view() : std::string_view{};
            if (encoding != kRawEncoding) {
                throw std::runtime_error{ std::format("unsupported RawImage encoding '{}'", encoding) };
            }

            const auto width = static_cast<int>(image->width());
            const auto height = static_cast<int>(image->height());
            const auto step = static_cast<size_t>(image->step());
            if (width <= 0 || height <= 0 || step < static_cast<size_t>(width) * 3
                || data->size() < step * static_cast<size_t>(height))
            {
                throw std::runtime_error{ "RawImage dimensions do not match its data" };
            }

            // The view aliases the message buffer, which dies with the payload: clone it out.
            const cv::Mat view{ height, width, CV_8UC3, const_cast<uint8_t*>(data->data()), step };
            return view.clone();
        }

    } // namespace

    const image_codec_desc* find_image_codec(const image_codec codec) noexcept
    {
        const auto it = std::ranges::find(kImageCodecs, codec, &image_codec_desc::codec);
        return (it != kImageCodecs.end()) ? &*it : nullptr;
    }

    const image_codec_desc* find_image_codec(const std::string_view id) noexcept
    {
        const auto it = std::ranges::find(kImageCodecs, id, &image_codec_desc::id);
        return (it != kImageCodecs.end()) ? &*it : nullptr;
    }

    std::span<const uint8_t> image_schema_bytes(const image_codec codec) noexcept
    {
        switch (codec) {
        case image_codec::jpeg:     return kCompressedImageBfbs;
        case image_codec::raw_bgr8: return kRawImageBfbs;
        }
        return {};
    }

    std::span<const uint8_t> calibration_schema_bytes() noexcept
    {
        return kCameraCalibrationBfbs;
    }

    std::vector<std::byte> encode_frame(
        const image_codec codec,
        const cv::Mat& bgr,
        const std::chrono::nanoseconds timestamp,
        const std::string_view frame_id,
        const encode_options& options)
    {
        if (bgr.empty() || bgr.type() != CV_8UC3) {
            throw std::invalid_argument{ "encode_frame: expected an 8-bit BGR image" };
        }

        switch (codec) {
        case image_codec::jpeg:     return encode_jpeg_frame(bgr, timestamp, frame_id, options);
        case image_codec::raw_bgr8: return encode_raw_frame(bgr, timestamp, frame_id);
        }
        throw std::invalid_argument{ "encode_frame: unknown image codec" };
    }

    cv::Mat decode_frame(const image_codec codec, const std::span<const std::byte> payload) noexcept try
    {
        switch (codec) {
        case image_codec::jpeg:     return decode_jpeg_frame(payload);
        case image_codec::raw_bgr8: return decode_raw_frame(payload);
        }
        throw std::invalid_argument{ "unknown image codec" };
    }
    catch (const std::exception& e)
    {
        spdlog::error("decode_frame failed: {}", e.what());
        return {};
    }

    std::vector<std::byte> encode_calibration(
        const hw::calibration_t& calibration,
        const std::string_view frame_id)
    {
        const hw::intrinsic_t& intr = calibration.color_intr;
        const hw::distortion_t& dist = calibration.color_dist;

        // OpenCV's coefficient order, which is what "rational_polynomial" means.
        const std::vector<double> d{
            dist.k1, dist.k2, dist.p1, dist.p2, dist.k3, dist.k4, dist.k5, dist.k6
        };
        const std::vector<double> k{
            intr.fx, 0.0,     intr.cx,
            0.0,     intr.fy, intr.cy,
            0.0,     0.0,     1.0
        };
        const std::vector<double> r{ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
        const std::vector<double> p{
            intr.fx, 0.0,     intr.cx, 0.0,
            0.0,     intr.fy, intr.cy, 0.0,
            0.0,     0.0,     1.0,     0.0
        };

        flatbuffers::FlatBufferBuilder builder;
        const auto frame = builder.CreateString(frame_id);
        const auto model = builder.CreateString(kDistortionModel);
        const auto d_vec = builder.CreateVector(d);
        const auto k_vec = builder.CreateVector(k);
        const auto r_vec = builder.CreateVector(r);
        const auto p_vec = builder.CreateVector(p);
        const foxglove::Time time = to_foxglove_time(std::chrono::nanoseconds{ 0 });

        builder.Finish(foxglove::CreateCameraCalibration(
            builder, &time, frame,
            static_cast<uint32_t>(calibration.color_resolution.x()),
            static_cast<uint32_t>(calibration.color_resolution.y()),
            model, d_vec, k_vec, r_vec, p_vec));
        return to_bytes(builder);
    }

    hw::calibration_t decode_calibration(const std::span<const std::byte> payload) noexcept try
    {
        if (!verify_payload(payload, [](flatbuffers::Verifier& v) {
            return foxglove::VerifyCameraCalibrationBuffer(v);
        })) {
            throw std::runtime_error{ "not a valid CameraCalibration message" };
        }

        const auto* calib = foxglove::GetCameraCalibration(payload.data());
        const auto* k = calib->k();
        if (!k || k->size() != 9) {
            throw std::runtime_error{ "CameraCalibration has no intrinsic matrix" };
        }

        const auto width = static_cast<int>(calib->width());
        const auto height = static_cast<int>(calib->height());

        hw::calibration_t out{};
        out.color_intr = hw::intrinsic_t{
            .fx = static_cast<float>(k->Get(0)),
            .fy = static_cast<float>(k->Get(4)),
            .cx = static_cast<float>(k->Get(2)),
            .cy = static_cast<float>(k->Get(5)),
            .width = width,
            .height = height,
        };

        // Absent or shorter coefficient vectors leave the remaining terms at zero,
        // which is what "this model does not use them" means.
        if (const auto* d = calib->d()) {
            const auto at = [d](const uint32_t i) {
                return (i < d->size()) ? static_cast<float>(d->Get(i)) : 0.0f;
            };
            out.color_dist = hw::distortion_t{
                .k1 = at(0), .k2 = at(1), .k3 = at(4),
                .k4 = at(5), .k5 = at(6), .k6 = at(7),
                .p1 = at(2), .p2 = at(3),
            };
        }

        out.color_resolution = Eigen::Vector2i{ width, height };

        const float fx = out.color_intr.fx;
        const float fy = out.color_intr.fy;
        const float h_fov = (fx > 0.0f)
            ? static_cast<float>(2.0 * std::atan(width / (2.0 * fx)) * 180.0 / kPi)
            : 0.0f;
        const float v_fov = (fy > 0.0f)
            ? static_cast<float>(2.0 * std::atan(height / (2.0 * fy)) * 180.0 / kPi)
            : 0.0f;
        out.color_fov = Eigen::Vector2f{ h_fov, v_fov };

        return out;
    }
    catch (const std::exception& e)
    {
        spdlog::error("decode_calibration failed: {}", e.what());
        return {};
    }

} // namespace io
