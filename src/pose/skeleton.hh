#pragma once
#include <Eigen/Geometry>

#include <array>
#include <optional>
#include <string_view>
#include <cstddef>

// Exo lower-limb rig: the joint set, how each joint binds to a tag and to its parent, and the
// per-joint state one estimation step produces. Every pose estimator consumes this rig and fills
// `joint_state_t`, so the rest of the app (GUI plots, trace, protocol) reads one shape regardless
// of which estimator produced it.
//
// Rig space is the frame everything here is expressed in. It is the frame of a camera placed
// squarely in front of the exo, with the exo facing back at it: X to that camera's right, Y down,
// Z away from the camera. Facing each other reverses two of the exo's own senses: its left falls on
// the camera's right (positive X), and its front points back at the camera (negative Z).
//
//        front view (what that camera sees)              seen from above
//
//                    -Y  up                            +Z  behind the exo
//                     |                                 ^
//         r_knee      |      l_knee                     |
//            o--------+--------o  --> +X        r_knee  o--+--o  l_knee
//                     |          camera's right,           |
//                     v          the exo's left            |  --> +X
//                    +Y  down                          [camera]
//                                                    the exo's front
//
// A frontal estimator can therefore pass camera-space measurements straight through; one working
// from any other viewpoint carries them into this frame itself.
namespace pose
{
    // ---------------------------------------------------------------------------
    // Skeleton definition
    // ---------------------------------------------------------------------------

    enum class joint_id_t
    {
        pelvis = 0, r_knee, l_knee, r_ankle, l_ankle, r_foot, l_foot,
        count
    };

    inline constexpr size_t kNumJoints{ static_cast<size_t>(joint_id_t::count) };

    // Which half of the exo's body a joint belongs to. The exo faces the camera in rig space, so its
    // `right` joints sit at negative X and its `left` ones at positive X.
    enum class joint_side_t { midline, right, left };

    // Static per-joint definition: binds a joint to its tag, its parent, and its left/right twin.
    struct joint_info_t
    {
        joint_id_t id;
        std::string_view name;
        int tag_id;        // tag id bound to this joint
        joint_id_t parent; // parent joint (== id itself for a root)
        joint_id_t mirror; // left/right counterpart (== id itself when the joint is on the midline)
        joint_side_t side; // body half this joint belongs to
    };

    // Single source of truth for the rig. Add a joint => add an enum value + one row.
    inline constexpr std::array<joint_info_t, kNumJoints> kJointsInfo{ {
        { joint_id_t::pelvis,  "pelvis",  0, joint_id_t::pelvis,  joint_id_t::pelvis,  joint_side_t::midline }, // root (self-parent)
        { joint_id_t::r_knee,  "r_knee",  1, joint_id_t::pelvis,  joint_id_t::l_knee,  joint_side_t::right   },
        { joint_id_t::l_knee,  "l_knee",  2, joint_id_t::pelvis,  joint_id_t::r_knee,  joint_side_t::left    },
        { joint_id_t::r_ankle, "r_ankle", 3, joint_id_t::r_knee,  joint_id_t::l_ankle, joint_side_t::right   },
        { joint_id_t::l_ankle, "l_ankle", 4, joint_id_t::l_knee,  joint_id_t::r_ankle, joint_side_t::left    },
        { joint_id_t::r_foot,  "r_foot",  5, joint_id_t::r_ankle, joint_id_t::l_foot,  joint_side_t::right   },
        { joint_id_t::l_foot,  "l_foot",  6, joint_id_t::l_ankle, joint_id_t::r_foot,  joint_side_t::left    },
    } };

    // Table lookups over kJointsInfo.
    constexpr const joint_info_t& joint_info(joint_id_t j) {
        return kJointsInfo[static_cast<size_t>(j)];
    }

    // Reverse lookup: tag id -> joint (linear over kJointsInfo).
    constexpr std::optional<joint_id_t> tag_to_joint(int tag_id) {
        for (const auto& info : kJointsInfo) {
            if (info.tag_id == tag_id) { return info.id; }
        }
        return std::nullopt;
    }

    // NOTE: a root joint is its own parent (parent == id).
    constexpr bool is_root_joint(joint_id_t j) {
        return joint_info(j).parent == j;
    }

    // NOTE: a midline joint is its own mirror (mirror == id).
    constexpr bool has_mirror_joint(joint_id_t j) {
        return joint_info(j).mirror != j;
    }

    constexpr joint_side_t joint_side(joint_id_t j) {
        return joint_info(j).side;
    }

    // NOTE: The estimator walks kJointsInfo in a single forward pass,
    // so every parent must precede its child. (parent index <= own)
    static_assert([] {
        for (const auto& j : kJointsInfo) {
            if (static_cast<size_t>(j.parent) > static_cast<size_t>(j.id)) { return false; }
        }
        return true;
    }(), "kJointsInfo must be parent-before-child ordered");

    // NOTE: mirroring is an involution, so a joint must be the mirror of its own mirror, and the
    // two halves of a pair must sit on opposite sides (a midline joint pairs with itself).
    static_assert([] {
        for (const auto& j : kJointsInfo) {
            if (joint_info(j.mirror).mirror != j.id) { return false; }
            const bool midline = (j.mirror == j.id);
            if (midline != (j.side == joint_side_t::midline)) { return false; }
            if (!midline && joint_info(j.mirror).side == j.side) { return false; }
        }
        return true;
    }(), "kJointsInfo mirror pairs must be symmetric and sit on opposite sides");

    // ---------------------------------------------------------------------------
    // Per-joint result of one estimation step
    // ---------------------------------------------------------------------------
    struct joint_state_t
    {
        std::optional<Eigen::Vector3d> raw_position;      // raw rig-space position this frame (fresh detection)
        std::optional<Eigen::Vector3d> position;          // smoothed + held rig-space position (drives the skeleton)
        std::optional<Eigen::Quaterniond> local_anim_rot; // parent-relative rotation vs the captured rest (drives the rig)
    };

} // namespace pose
