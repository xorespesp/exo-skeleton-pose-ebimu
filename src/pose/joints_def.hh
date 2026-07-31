#pragma once
#include <array>
#include <optional>
#include <span>
#include <string_view>
#include <cstddef>

// Exo lower-limb rig: the joint set, and how each joint binds to a tag, to its parent and to its
// left/right twin. One table defines all of it, so adding a joint is an enum value plus a row.
//
// Rig space is the frame the rig and everything measured for it are expressed in. It is the frame
// of a camera placed squarely in front of the exo, with the exo facing back at it: X to that
// camera's right, Y down, Z away from the camera. Facing each other reverses two of the exo's own
// senses: its left falls on the camera's right (positive X), and its front points back at the
// camera (negative Z).
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
    enum class joint_id_t
    {
        pelvis = 0, r_knee, l_knee, r_ankle, l_ankle, r_foot, l_foot,
        count
    };

    inline constexpr size_t kNumJoints{ static_cast<size_t>(joint_id_t::count) };

    // Which half of the exo's body a joint belongs to.
    enum class joint_side_t { midline, right, left };

    // Static per-joint definition: tag binding, parent, and left/right twin.
    struct joint_definition_t
    {
        joint_id_t joint_id;
        std::string_view name;
        int tag_id;        // tag printed for this joint
        joint_id_t parent; // == joint_id for a root
        joint_id_t mirror; // == joint_id on the midline
        joint_side_t side;
    };

    namespace detail
    {
        // Single source of truth for the rig. Add a joint => add an enum value + one row.
        // Behind detail so call sites use the accessors below: a guarded row lookup, or a range.
        inline constexpr std::array<joint_definition_t, kNumJoints> kJointDefTable{ {
            { joint_id_t::pelvis,  "pelvis",  0, joint_id_t::pelvis,  joint_id_t::pelvis,  joint_side_t::midline }, // root (self-parent)
            { joint_id_t::r_knee,  "r_knee",  1, joint_id_t::pelvis,  joint_id_t::l_knee,  joint_side_t::right   },
            { joint_id_t::l_knee,  "l_knee",  2, joint_id_t::pelvis,  joint_id_t::r_knee,  joint_side_t::left    },
            { joint_id_t::r_ankle, "r_ankle", 3, joint_id_t::r_knee,  joint_id_t::l_ankle, joint_side_t::right   },
            { joint_id_t::l_ankle, "l_ankle", 4, joint_id_t::l_knee,  joint_id_t::r_ankle, joint_side_t::left    },
            { joint_id_t::r_foot,  "r_foot",  5, joint_id_t::r_ankle, joint_id_t::l_foot,  joint_side_t::right   },
            { joint_id_t::l_foot,  "l_foot",  6, joint_id_t::l_ankle, joint_id_t::r_foot,  joint_side_t::left    },
        } };

        // --- table invariants: a row edited into an inconsistent state fails the build ---

        // NOTE: get_joint_def() indexes by enum value, so row i must define joint i.
        static_assert([] {
            for (size_t i = 0; i < kNumJoints; ++i) {
                if (kJointDefTable[i].joint_id != static_cast<joint_id_t>(i)) { return false; }
            }
            return true;
        }(), "kJointDefTable rows must be ordered by joint_id_t");

        // NOTE: a tag binds to one joint, so tag_id_to_joint_id() has a single answer.
        // A duplicate id would silently resolve to the earlier row.
        static_assert([] {
            for (size_t i = 0; i < kNumJoints; ++i) {
                for (size_t k = 0; k < i; ++k) {
                    if (kJointDefTable[k].tag_id == kJointDefTable[i].tag_id) { return false; }
                }
            }
            return true;
        }(), "kJointDefTable must bind distinct tag ids");

        // NOTE: estimators walk the table in one forward pass, so a parent must precede its child.
        static_assert([] {
            for (const auto& j : kJointDefTable) {
                if (static_cast<size_t>(j.parent) > static_cast<size_t>(j.joint_id)) { return false; }
            }
            return true;
        }(), "kJointDefTable must be parent-before-child ordered");

        // NOTE: mirroring is an involution, and a pair's halves sit on opposite sides
        // (a midline joint pairs with itself).
        static_assert([] {
            for (const auto& j : kJointDefTable) {
                const auto& twin = kJointDefTable[static_cast<size_t>(j.mirror)];
                if (twin.joint_id != j.mirror || twin.mirror != j.joint_id) { return false; }
                const bool midline = (j.mirror == j.joint_id);
                if (midline != (j.side == joint_side_t::midline)) { return false; }
                if (!midline && twin.side == j.side) { return false; }
            }
            return true;
        }(), "kJointDefTable mirror pairs must be symmetric and sit on opposite sides");

        // NOTE: get_root_joint() reads the root off row 0 instead of searching for it.
        static_assert(kJointDefTable[0].parent == kJointDefTable[0].joint_id,
            "kJointDefTable row 0 must be the rig root");
    } // namespace detail

    // All joint definitions, parent before child.
    constexpr std::span<const joint_definition_t> get_joint_defs() {
        return detail::kJointDefTable;
    }

    // Definition of `j`; empty for an id outside the enum, which only a cast can produce.
    constexpr std::optional<joint_definition_t> get_joint_def(joint_id_t j) {
        const size_t i = static_cast<size_t>(j);
        if (i >= kNumJoints) [[unlikely]] { return std::nullopt; }
        return detail::kJointDefTable[i];
    }

    // The joint every chain hangs from: the one that is its own parent.
    constexpr joint_id_t get_root_joint() {
        return detail::kJointDefTable[0].joint_id;
    }

    // Reverse lookup: tag id -> joint.
    constexpr std::optional<joint_id_t> tag_id_to_joint_id(int tag_id) {
        for (const auto& def : get_joint_defs()) {
            if (def.tag_id == tag_id) { return def.joint_id; }
        }
        return std::nullopt;
    }

    // Display name of `j` for logs, labels and dumps; "?" for an id outside the enum.
    constexpr std::string_view get_joint_name(joint_id_t j) {
        const auto def = get_joint_def(j);
        return def.has_value() ? def->name : "?";
    }

    // Body half `j` belongs to.
    constexpr std::optional<joint_side_t> get_joint_side(joint_id_t j) {
        const auto def = get_joint_def(j);
        return def.has_value() ? std::optional{ def->side } : std::nullopt;
    }

    // A root joint is its own parent; an id outside the enum is not one.
    constexpr bool is_root_joint(joint_id_t j) {
        const auto def = get_joint_def(j);
        return def.has_value() && def->parent == j;
    }

    // True for joints with a left/right twin; midline joints have none.
    constexpr bool has_mirror_joint(joint_id_t j) {
        const auto def = get_joint_def(j);
        return def.has_value() && def->mirror != j;
    }

} // namespace pose
