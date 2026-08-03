#pragma once
#include "hw/calibration.hh"

#include <filesystem>
#include <string>

namespace io
{
    // Reads a calibration measured off-line, for a camera that reports none of its own.
    //
    // The layout is OpenCV FileStorage (.yml / .xml), so what a chessboard calibration writes
    // out is taken as is: `image_width`, `image_height`, a 3x3 `camera_matrix`, and
    // `distortion_coefficients` of 4, 5, 8, 12 or 14. The frame size lands in `intr` because a
    // set measured at one size projects plausibly and wrongly at another.
    //
    // On failure `err` says what was missing or malformed and the outputs are left untouched.
    [[nodiscard]] bool load_camera_calibration(
        const std::filesystem::path& file,
        hw::intrinsic_t& intr /*out*/,
        hw::distortion_t& dist /*out*/,
        std::string& err /*out*/
    );

} // namespace io
