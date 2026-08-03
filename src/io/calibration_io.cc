#include "calibration_io.hh"

#include <opencv2/core.hpp>

#include <format>

namespace io
{
    namespace
    {
        // Distortion coefficients come as a row or a column, and the count varies with the model
        // OpenCV was asked for. Reading them positionally out of a flattened view covers every
        // shape, and a set shorter than ours simply leaves the rest at zero.
        void assign_distortion(const cv::Mat& coeffs, hw::distortion_t& dist)
        {
            const cv::Mat flat = coeffs.reshape(1, 1);
            const auto at = [&flat](const int i) {
                return (i < flat.cols) ? static_cast<float>(flat.at<double>(0, i)) : 0.0f;
            };

            // OpenCV's order: k1 k2 p1 p2 [k3 [k4 k5 k6 [s1 s2 s3 s4 [taux tauy]]]]
            dist.k1 = at(0);
            dist.k2 = at(1);
            dist.p1 = at(2);
            dist.p2 = at(3);
            dist.k3 = at(4);
            dist.k4 = at(5);
            dist.k5 = at(6);
            dist.k6 = at(7);
        }

    } // namespace

    bool load_camera_calibration(
        const std::filesystem::path& file,
        hw::intrinsic_t& intr,
        hw::distortion_t& dist,
        std::string& err)
    {
        cv::FileStorage fs;
        try {
            if (!fs.open(file.string(), cv::FileStorage::READ)) {
                err = std::format("cannot open '{}'", file.string());
                return false;
            }
        }
        catch (const cv::Exception& e) {
            err = e.what();
            return false;
        }

        int width = 0, height = 0;
        cv::Mat camera_matrix, coeffs;
        try {
            fs["image_width"] >> width;
            fs["image_height"] >> height;
            fs["camera_matrix"] >> camera_matrix;
            fs["distortion_coefficients"] >> coeffs;
        }
        catch (const cv::Exception& e) {
            err = e.what();
            return false;
        }

        if (width <= 0 || height <= 0) {
            err = "'image_width' / 'image_height' are missing or not positive";
            return false;
        }
        if (camera_matrix.rows != 3 || camera_matrix.cols != 3) {
            err = "'camera_matrix' must be 3x3";
            return false;
        }

        // FileStorage writes floating point as double whatever the in-memory type was.
        camera_matrix.convertTo(camera_matrix, CV_64F);

        hw::intrinsic_t parsed{};
        parsed.fx = static_cast<float>(camera_matrix.at<double>(0, 0));
        parsed.fy = static_cast<float>(camera_matrix.at<double>(1, 1));
        parsed.cx = static_cast<float>(camera_matrix.at<double>(0, 2));
        parsed.cy = static_cast<float>(camera_matrix.at<double>(1, 2));
        parsed.width = width;
        parsed.height = height;

        // A zero focal length would reach the pose solver as a valid-looking set and produce
        // garbage, so it is caught where the file is read rather than where it is used.
        if (parsed.fx <= 0.0f || parsed.fy <= 0.0f) {
            err = "'camera_matrix' holds no positive focal length";
            return false;
        }

        hw::distortion_t parsed_dist{};
        if (!coeffs.empty()) {
            coeffs.convertTo(coeffs, CV_64F);
            assign_distortion(coeffs, parsed_dist);
        }

        intr = parsed;
        dist = parsed_dist;
        return true;
    }

} // namespace io
