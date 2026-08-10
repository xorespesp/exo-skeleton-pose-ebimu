#pragma once
#include "calibration.hh"

#include "utils/serializable.hh"

#include <algorithm>

namespace hw
{
    // Region of interest (ROI) in a frame.
    struct roi_t
    {
        int x{ 0 }, y{ 0 }; // (x,y) : top-left corner
        int width{ 0 }, height{ 0 }; // (w,h) : dimensions

        bool is_empty() const { return width <= 0 || height <= 0; }

        DECLARE_SERIALIZABLE_FIELDS(
            v("x",      o.x);
            v("y",      o.y);
            v("width",  o.width);
            v("height", o.height);
        )
    };

    // `roi` clipped to a width x height frame. Comes back empty when the two do not overlap.
    inline roi_t clamp_roi(const roi_t& roi, int frame_width, int frame_height)
    {
        const int x0 = std::clamp(roi.x, 0, frame_width);
        const int y0 = std::clamp(roi.y, 0, frame_height);
        const int x1 = std::clamp(roi.x + roi.width, x0, frame_width);
        const int y1 = std::clamp(roi.y + roi.height, y0, frame_height);
        return roi_t{ x0, y0, x1 - x0, y1 - y0 };
    }

    // Retarget a calibration measured on the full frame onto `roi`.
    //
    // Narrowing the frame moves the optical center within the delivered image but leaves the
    // lens alone: the focal lengths hold and the principal point shifts by the offset. The
    // distortion coefficients are expressed relative to that principal point, so they carry
    // over untouched.
    inline void apply_roi(calibration_t& calib, const roi_t& roi)
    {
        calib.intrinsic.cx -= static_cast<float>(roi.x);
        calib.intrinsic.cy -= static_cast<float>(roi.y);
        calib.intrinsic.calib_resolution = Eigen::Vector2i{ roi.width, roi.height };

        calib.frame_resolution = Eigen::Vector2i{ roi.width, roi.height };
    }

} // namespace hw
