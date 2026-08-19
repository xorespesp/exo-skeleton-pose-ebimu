#pragma once
#include <array>
#include <numbers>
#include <optional>
#include <span>
#include <string_view>
#include <cstddef>

// Exo lower-limb rig: the joint set, and how each joint binds to a tag, to its parent and to its
// left/right twin. One table defines all of it, so adding a joint is an enum value plus a row.
//
// A joint is an articulation point: its state (rotation, angles) describes the bend AT it, which
// turns the bone leaving it toward its child. Several joints may share one tag when they sit at
// the same physical point: the pelvis marker carries the pelvis root and both hip articulations,
// so a detection of that tag measures all three. The feet are marker end sites, joints that close
// off the chain with a position and no articulation of their own.
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
        pelvis = 0, r_hip, l_hip, r_knee, l_knee, r_ankle, l_ankle, r_foot, l_foot,
        count
    };

    inline constexpr size_t kNumJoints{ static_cast<size_t>(joint_id_t::count) };

    // Which half of the exo's body a joint belongs to.
    enum class joint_side_t { midline, right, left };

    constexpr std::string_view joint_side_name(joint_side_t s) {
        switch (s) {
        case joint_side_t::right: return "right";
        case joint_side_t::left:  return "left";
        case joint_side_t::midline: break;
        }
        return "midline";
    }

    // nullopt if `name` is none of them. The names round-trip through `joint_side_name()`,
    // which is what a config file spells.
    constexpr std::optional<joint_side_t> joint_side_from_name(std::string_view name) {
        if (name == joint_side_name(joint_side_t::right))   { return joint_side_t::right; }
        if (name == joint_side_name(joint_side_t::left))    { return joint_side_t::left; }
        if (name == joint_side_name(joint_side_t::midline)) { return joint_side_t::midline; }
        return std::nullopt;
    }

    // Static per-joint definition
    struct joint_definition_t
    {
        joint_id_t joint_id;
        std::string_view name;
        int tag_id;        // tag this joint is measured by; co-sited joints share one
        joint_id_t parent; // == joint_id for a root
        joint_side_t side;

        // Clinical Joint Angle conversion (docs/joint_angle_convention.md):
        //
        //   sagittal_clinical_angle = sagittal_clinical_sign * rig_bend - sagittal_clinical_neutral_offset_rad
        //
        // `rig_bend` is the bend at this joint in the rig's hinge sign (right-hand rule about
        // the rig's +X, so a backward swing reads positive; 0 when the two bones are collinear).
        // `sagittal_clinical_sign` turns that into the convention's flexion/dorsiflexion-positive
        // reading, and `sagittal_clinical_neutral_offset_rad` moves the zero onto the convention's
        // neutral stance; only the ankle has one, its neutral holding the foot perpendicular to
        // the shank. A sign of 0 marks a joint with no clinical angle: the root, whose bone is
        // the reference axis itself, and the marker end sites, which no bone leaves.
        int sagittal_clinical_sign;
        double sagittal_clinical_neutral_offset_rad;
    };

    namespace detail
    {
        inline constexpr double kDeg2Rad = std::numbers::pi / 180.0;
        inline constexpr double kAnkleNeutralOffsetRad = 90.0 * kDeg2Rad;

        // Joint definition table. Add a joint => add an enum value + one row.
        // NOTE: Tag 0 is the pelvis marker, which the pelvis root and both hips are co-sited on.
        inline constexpr std::array<joint_definition_t, kNumJoints> kJointDefTable{ {
            { joint_id_t::pelvis,  "pelvis",  0, joint_id_t::pelvis,  joint_side_t::midline,  0, 0.0 }, // root (self-parent)
            { joint_id_t::r_hip,   "r_hip",   0, joint_id_t::pelvis,  joint_side_t::right,   -1, 0.0 },
            { joint_id_t::l_hip,   "l_hip",   0, joint_id_t::pelvis,  joint_side_t::left,    -1, 0.0 },
            { joint_id_t::r_knee,  "r_knee",  1, joint_id_t::r_hip,   joint_side_t::right,   +1, 0.0 },
            { joint_id_t::l_knee,  "l_knee",  2, joint_id_t::l_hip,   joint_side_t::left,    +1, 0.0 },
            { joint_id_t::r_ankle, "r_ankle", 3, joint_id_t::r_knee,  joint_side_t::right,   -1, kAnkleNeutralOffsetRad },
            { joint_id_t::l_ankle, "l_ankle", 4, joint_id_t::l_knee,  joint_side_t::left,    -1, kAnkleNeutralOffsetRad },
            { joint_id_t::r_foot,  "r_foot",  5, joint_id_t::r_ankle, joint_side_t::right,    0, 0.0 }, // marker end site
            { joint_id_t::l_foot,  "l_foot",  6, joint_id_t::l_ankle, joint_side_t::left,     0, 0.0 }, // marker end site
        } };

        // --- table invariants: a row edited into an inconsistent state fails the build ---

        // NOTE: get_joint_def() indexes by enum value, so row i must define joint i.
        static_assert([] {
            for (size_t i = 0; i < kNumJoints; ++i) {
                if (kJointDefTable[i].joint_id != static_cast<joint_id_t>(i)) { return false; }
            }
            return true;
        }(), "kJointDefTable rows must be ordered by joint_id_t");

        // NOTE: joints sharing a tag are co-sited: they name one physical point, so each of them
        // must hang directly off the tag's primary joint (its first row), which is what makes the
        // shared measurement a valid position for all of them. tag_id_to_joint_id() resolves a
        // tag to that primary row.
        static_assert([] {
            for (size_t i = 0; i < kNumJoints; ++i) {
                for (size_t k = 0; k < i; ++k) {
                    if (kJointDefTable[k].tag_id != kJointDefTable[i].tag_id) { continue; }
                    if (kJointDefTable[i].parent != kJointDefTable[k].joint_id) { return false; }
                    break; // k is the primary: the first row bound to this tag
                }
            }
            return true;
        }(), "kJointDefTable joints co-sited on one tag must hang directly off the tag's primary joint");

        // NOTE: estimators walk the table in one forward pass, so a parent must precede its child.
        static_assert([] {
            for (const auto& j : kJointDefTable) {
                if (static_cast<size_t>(j.parent) > static_cast<size_t>(j.joint_id)) { return false; }
            }
            return true;
        }(), "kJointDefTable must be parent-before-child ordered");

        // NOTE: get_root_joint() reads the root off row 0 instead of searching for it.
        static_assert(kJointDefTable[0].parent == kJointDefTable[0].joint_id,
            "kJointDefTable row 0 must be the rig root");

        // NOTE: Twins are one articulation on two legs, found by parent-chain depth + opposite side.
        //       Three things hold, one per check below:
        //         - no clinical angle => no offset
        //         - exactly one partner per leg joint, which is what makes that lookup exact
        //         - partners share sign and offset, so a twin's angle reads as the joint's own
        static_assert([] {
            const auto depth = [](size_t i) {
                size_t d = 0;
                while (i < kNumJoints && kJointDefTable[i].parent != kJointDefTable[i].joint_id && d < kNumJoints) {
                    i = static_cast<size_t>(kJointDefTable[i].parent);
                    ++d;
                }
                return d;
            };

            for (size_t i = 0; i < kNumJoints; ++i) {
                const auto& j = kJointDefTable[i];
                if (j.sagittal_clinical_sign == 0 && j.sagittal_clinical_neutral_offset_rad != 0.0) { return false; }
                if (j.side == joint_side_t::midline) { continue; }

                size_t twins = 0;
                for (size_t k = 0; k < kNumJoints; ++k) {
                    const auto& t = kJointDefTable[k];
                    if (t.side == j.side || t.side == joint_side_t::midline) { continue; }
                    if (depth(k) != depth(i)) { continue; }
                    ++twins;
                    if (t.sagittal_clinical_sign != j.sagittal_clinical_sign) { return false; }
                    if (t.sagittal_clinical_neutral_offset_rad != j.sagittal_clinical_neutral_offset_rad) { return false; }
                }
                if (twins != 1) { return false; }
            }
            return true;
        }(), "kJointDefTable legs must pair one to one across the two sides and share one clinical conversion");

        // NOTE: a clinical angle exists exactly at the articulated leg joints: a non-root joint
        // that a bone leaves (one with a child). The root's bone is the reference axis and an
        // end site has no bone of its own, so neither carries one.
        static_assert([] {
            for (const auto& j : kJointDefTable) {
                bool has_child = false;
                for (const auto& c : kJointDefTable) {
                    if (c.joint_id != j.joint_id && c.parent == j.joint_id) { has_child = true; }
                }
                const bool articulated = has_child && j.parent != j.joint_id;
                if (articulated != (j.sagittal_clinical_sign != 0)) { return false; }
                if (j.sagittal_clinical_sign < -1 || j.sagittal_clinical_sign > 1) { return false; }
            }
            return true;
        }(), "kJointDefTable clinical conversions must sit exactly on the articulated leg joints");
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

    // Reverse lookup: tag id -> the tag's primary joint (its first table row). Joints co-sited on
    // the tag hang directly off the primary; a caller wanting all of them scans the table for the
    // tag id, which is what the measurement bindings do.
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

    // The child hanging off `j`; empty at the end of a chain. A leg is a single strand, so below
    // its hip the first row that names `j` as its parent is the only one, and walking repeatedly
    // gives the whole limb. The root parents one hip per side, which get_leg_root_joint() picks
    // between; here the root answers with its first child.
    constexpr std::optional<joint_id_t> get_child_joint(joint_id_t j) {
        for (const auto& c : get_joint_defs()) {
            if (!is_root_joint(c.joint_id) && c.parent == j) { return c.joint_id; }
        }
        return std::nullopt;
    }

    // Where one leg's chain starts: the root's child on `side`.
    // Empty for the midline, which owns no limb of its own.
    constexpr std::optional<joint_id_t> get_leg_root_joint(joint_side_t side) {
        const joint_id_t root = get_root_joint();
        for (const auto& c : get_joint_defs()) {
            if (is_root_joint(c.joint_id) || c.parent != root) { continue; }
            if (c.side == side) { return c.joint_id; }
        }
        return std::nullopt;
    }

    // ---------------------------------------------------------------------------
    // Published angle conventions (docs/joint_angle_convention.md)
    // ---------------------------------------------------------------------------
    //
    // The estimators measure geometry in the rig's hinge sign: rotations about the rig's +X, so
    // a swing toward the exo's back reads positive. The functions below carry those readings
    // into the document's biomechanics conventions, and are the only place the two sign systems
    // meet: every per-joint sign and neutral offset lives in `kJointDefTable`, nowhere else.
    // Angles are radians throughout.

    // Segment Angle: a bone's attitude measured from vertically down, positive tilted toward
    // the exo's front (anterior). The rig-sign attitude measures the same tilt from the same
    // axis with the opposite sign. Applies equally to a change of the attitude, since the flip
    // distributes over a difference.
    constexpr double sagittal_segment_angle_from_rig(double rig_sagittal_segment_angle_rad) {
        return -rig_sagittal_segment_angle_rad;
    }

    // Clinical Joint Angle at `j` (Neutral Zero Method): the bend versus the parent bone, 0 at
    // the neutral stance, positive in flexion (hip, knee) and dorsiflexion (ankle). Empty for a
    // joint that has no clinical angle (the root and the marker end sites).
    constexpr std::optional<double> sagittal_clinical_angle_from_rig_bend(joint_id_t j, double rig_bend_rad) {
        const auto def = get_joint_def(j);
        if (!def.has_value() || def->sagittal_clinical_sign == 0) { return std::nullopt; }
        return def->sagittal_clinical_sign * rig_bend_rad - def->sagittal_clinical_neutral_offset_rad;
    }

    // Change of the Clinical Joint Angle at `j` for a change of the rig-sign bend: the neutral
    // offset cancels out of a difference, leaving the sign alone.
    constexpr std::optional<double> sagittal_clinical_angle_delta_from_rig_bend_delta(joint_id_t j, double rig_bend_delta_rad) {
        const auto def = get_joint_def(j);
        if (!def.has_value() || def->sagittal_clinical_sign == 0) { return std::nullopt; }
        return def->sagittal_clinical_sign * rig_bend_delta_rad;
    }

    // Included Angle at `j`: the angle between the two bones meeting at the joint, as a signed
    // continuous quantity: pi when collinear, shrinking with flexion and growing past pi in
    // extension (the ankle's neutral reads pi/2). Derived from the Clinical Joint Angle through
    // this one function alone, so the two always state one bend: the estimators fill
    // `joint_state_t` with it right beside the clinical angle, and every other reader takes the
    // stored pair as is.
    constexpr std::optional<double> sagittal_included_angle_from_clinical(joint_id_t j, double sagittal_clinical_angle_rad) {
        const auto def = get_joint_def(j);
        if (!def.has_value() || def->sagittal_clinical_sign == 0) { return std::nullopt; }
        return (std::numbers::pi - def->sagittal_clinical_neutral_offset_rad) - sagittal_clinical_angle_rad;
    }

} // namespace pose
