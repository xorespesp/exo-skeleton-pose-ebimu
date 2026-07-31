#pragma once
#include <string_view>

namespace pose
{
    // Plane the camera observes the exo in.
    enum class view_plane_t
    {
        frontal,  // camera faces the exo; both legs visible, rig recovered in 3D
        sagittal, // camera at the side; the near leg's flexion lies in the image plane
    };

    constexpr std::string_view view_plane_name(view_plane_t p) {
        return (p == view_plane_t::sagittal) ? "sagittal" : "frontal";
    }

} // namespace pose
