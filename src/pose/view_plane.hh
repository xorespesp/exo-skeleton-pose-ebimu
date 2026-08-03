#pragma once
#include <optional>
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

    // nullopt if `name` is neither plane. The names round-trip through `view_plane_name()`,
    // which is what the CLI and the config file both spell.
    constexpr std::optional<view_plane_t> view_plane_from_name(std::string_view name) {
        if (name == view_plane_name(view_plane_t::frontal))  { return view_plane_t::frontal; }
        if (name == view_plane_name(view_plane_t::sagittal)) { return view_plane_t::sagittal; }
        return std::nullopt;
    }

} // namespace pose
