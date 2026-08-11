// Maps the protocol's joints onto the Xbot rig and applies incoming rotations.
//
// The server sends, per joint, two forms of one parent-relative motion measured from the captured
// rest pose: `local_anim_rot`, a rotation, and `local_sagittal_angle`, the flexion in the exo's
// sagittal plane. A three.js bone's local quaternion is also parent-relative, so either form drives
// a bone as:
//
//     bone.quaternion = bindQuaternion * animDelta
//
// where animDelta is the rotation itself, or an equal turn about the lateral axis built from the
// angle. Driving from each in turn shows on the rig how closely the two forms agree.
//

import * as THREE from 'three';

// Indexed by exo.proto.JointId (0..6). Sanitized bone names (no colon).
export const BONE_BY_JOINT_ID = [
    'mixamorigHips',       // 0 Pelvis
    'mixamorigRightUpLeg', // 1 RKnee  (thigh)
    'mixamorigLeftUpLeg',  // 2 LKnee
    'mixamorigRightLeg',   // 3 RAnkle (shin)
    'mixamorigLeftLeg',    // 4 LAnkle
    'mixamorigRightFoot',  // 5 RFoot
    'mixamorigLeftFoot',   // 6 LFoot
];

// The exo's lateral axis in the rig frame, which the schema defines and states is fixed, so it
// is not carried in any message. Every leg joint hinges about it and the sagittal angles are
// measured about it.
const HINGE_AXIS = new THREE.Vector3(1, 0, 0);

// Snapshot each driven bone's rest (bind) quaternion so deltas apply on top of it.
export function captureBindPose(bones) {
    const bind = {};
    for (const name of BONE_BY_JOINT_ID) {
        if (bones[name]) { bind[name] = bones[name].quaternion.clone(); }
    }
    return bind;
}

const _delta = new THREE.Quaternion();

// Apply one decoded PoseFrame to the rig. `frame` is the generated flatbuffers object.
// With `driveFromAngle` set, each bone's delta is a turn about the lateral axis by that joint's
// `local_sagittal_angle`; otherwise the bone takes `local_anim_rot` as its delta.
export function applyPoseFrame(bones, bindPose, frame, driveFromAngle = false) {
    for (let i = 0; i < frame.jointsLength(); i++) {
        const jp = frame.joints(i);
        const name = BONE_BY_JOINT_ID[jp.id()];
        const bone = bones[name];
        if (!bone) { continue; }

        if (driveFromAngle) {
            const a = jp.localSagittalAngle();
            if (a === null) { continue; } // lost joint (no angle) keeps its rest pose
            _delta.setFromAxisAngle(HINGE_AXIS, a);
        } else {
            const q = jp.localAnimRot();
            if (!q) { continue; } // lost joint (null rotation) keeps its rest pose
            _delta.set(q.x(), q.y(), q.z(), q.w());
        }

        bone.quaternion.copy(bindPose[name]).multiply(_delta);
    }
}

const _q = new THREE.Quaternion();
const _euler = new THREE.Euler();

// Convert a flatbuffers Quat to euler angles [deg] in the given euler order.
// (for display-only; the rig is driven by the quaternion)
export function quatToEulerDeg(quat, order = 'XYZ') {
    _q.set(quat.x(), quat.y(), quat.z(), quat.w());
    _euler.setFromQuaternion(_q, order);
    const deg = THREE.MathUtils.radToDeg;
    return { x: deg(_euler.x), y: deg(_euler.y), z: deg(_euler.z) };
}

// The flexion recoverable from a rotation alone: the turn `quat` carries about the lateral axis [deg].
// It matches `local_sagittal_angle` when the rotation turns purely about that axis, 
// and shrinks in proportion to how far the rotation's own axis tilts away from it.
export function quatHingeAngleDeg(quat) {
    let [x, y, z, w] = [quat.x(), quat.y(), quat.z(), quat.w()];
    // quat and its negation name the same rotation; pinning w >= 0 keeps the sign below tied to the
    // axis component and not to the hemisphere the quaternion arrived in.
    if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
    const dot = x * HINGE_AXIS.x + y * HINGE_AXIS.y + z * HINGE_AXIS.z;
    return THREE.MathUtils.radToDeg(2 * Math.atan2(dot, w));
}
