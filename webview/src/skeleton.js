// Maps the protocol's joints onto the Xbot rig and applies incoming rotations.
//
// Two drive sources, both ending in the same application:
//
//     bone.quaternion = bindQuaternion * delta
//
// `local_anim_rot` is that delta outright: a parent-relative rotation from the captured rest
// pose. It needs a calibrated rest, and it reproduces whatever pose was captured as the bind.
//
// `sagittal_clinical_angle` gives the same delta as a turn about the rig's +X (the exo's left),
// the axis `local_anim_rot` is expressed about. It is the bend against the parent bone, which is
// what a bone's local quaternion holds, and it reads 0 at the neutral stance, so the per-joint
// sign below carries it onto that axis and the bind pose takes it as it is. It flows with no rest
// pose, so the rig animates as soon as the markers are seen, and it puts the neutral stance on
// the bind pose, so the two sources agree exactly whenever the captured rest is that stance.

import * as THREE from 'three';

// Indexed by exo.proto.JointId (0..8). Sanitized bone names (no colon).
// A joint's rotation turns the bone leaving it toward its child, which is exactly what rotating
// a three.js bone node does, so each joint maps onto the node standing at its articulation
// point. RFoot/LFoot are marker end sites: no bone leaves them, so they drive nothing.
export const BONE_BY_JOINT_ID = [
    'mixamorigHips',       // 0 Pelvis
    'mixamorigRightUpLeg', // 1 RHip   (drives the thigh)
    'mixamorigLeftUpLeg',  // 2 LHip
    'mixamorigRightLeg',   // 3 RKnee  (drives the shin)
    'mixamorigLeftLeg',    // 4 LKnee
    'mixamorigRightFoot',  // 5 RAnkle (drives the foot)
    'mixamorigLeftFoot',   // 6 LAnkle
    null,                  // 7 RFoot  (marker end site)
    null,                  // 8 LFoot
];

// Sign that carries `sagittal_clinical_angle` (flexion positive) onto a turn about the rig's +X.
// Mirrors `sagittal_clinical_sign` in src/pose/joints_def.hh, where 0 marks a joint with no
// clinical angle. The sign is the whole conversion: the clinical angle already states a deviation
// from neutral, which is what a delta on the bind pose is.
const CLINICAL_ANGLE_SIGN_BY_JOINT_ID = [
    0,  // 0 Pelvis (no parent bone)
    -1, // 1 RHip
    -1, // 2 LHip
    +1, // 3 RKnee
    +1, // 4 LKnee
    -1, // 5 RAnkle
    -1, // 6 LAnkle
    0,  // 7 RFoot (marker end site)
    0,  // 8 LFoot
];

// Left/right twin of each joint, indexed by JointId; a midline joint is its own twin. The rig
// table in src/pose/joints_def.hh reaches the same pairing from parent-chain depth and side.
const MIRROR_BY_JOINT_ID = [
    0, // 0 Pelvis (midline)
    2, // 1 RHip   -> LHip
    1, // 2 LHip   -> RHip
    4, // 3 RKnee  -> LKnee
    3, // 4 LKnee  -> RKnee
    6, // 5 RAnkle -> LAnkle
    5, // 6 LAnkle -> RAnkle
    8, // 7 RFoot  -> LFoot
    7, // 8 LFoot  -> RFoot
];

// exo.proto.JointId names, for the debug readout.
export const JOINT_NAME_BY_ID = [
    'Pelvis', 'RHip', 'LHip', 'RKnee', 'LKnee', 'RAnkle', 'LAnkle', 'RFoot', 'LFoot',
];

// Snapshot each driven bone's rest (bind) quaternion so deltas apply on top of it.
export function captureBindPose(bones) {
    const bind = {};
    for (const name of BONE_BY_JOINT_ID) {
        if (name && bones[name]) { bind[name] = bones[name].quaternion.clone(); }
    }
    return bind;
}

// Put every driven bone back on its bind pose. Called when a setting changes which bones get
// driven, so a bone the new set does not reach shows its bind pose.
export function resetToBindPose(bones, bindPose) {
    for (const name of BONE_BY_JOINT_ID) {
        if (name && bones[name] && bindPose[name]) { bones[name].quaternion.copy(bindPose[name]); }
    }
}

// The two wire fields a bone's delta can be built from.
export const DRIVE_SOURCE = {
    localAnimRot: 'local_anim_rot',
    clinicalAngle: 'clinical_angle',
};

// The source a frame is actually driven from. Anything other than one of the two above reads as
// "follow the frame", which goes by its rest-pose flag: that flag is exactly what says whether
// `local_anim_rot` is present, so the rig animates from the angles until a rest is captured and
// from the rotations after. Resolved once per frame, so one frame cannot mix two reference poses.
export function resolveDriveSource(mode, frame) {
    if (mode === DRIVE_SOURCE.localAnimRot || mode === DRIVE_SOURCE.clinicalAngle) { return mode; }
    return frame.hasRestPose() ? DRIVE_SOURCE.localAnimRot : DRIVE_SOURCE.clinicalAngle;
}

const _delta = new THREE.Quaternion();
const _hingeAxis = new THREE.Vector3(1, 0, 0); // rig +X, the exo's left
const _localAnimRotByJointId = new Array(BONE_BY_JOINT_ID.length).fill(null);
const _clinicalAngleRadByJointId = new Array(BONE_BY_JOINT_ID.length).fill(null);

// Apply one decoded PoseFrame to the rig. `frame` is the generated flatbuffers object,
// `driveSource` is a `DRIVE_SOURCE` value (see the note atop this file), and `mirrorUnmeasured`
// lets a joint the frame left empty take its twin's value. A joint with no value to apply keeps
// the pose it already holds.
export function applyPoseFrame(bones, bindPose, frame, driveSource = DRIVE_SOURCE.localAnimRot, mirrorUnmeasured = true) {
    const fromClinicalAngle = (driveSource === DRIVE_SOURCE.clinicalAngle);

    // Collected into slots first because a joint's twin may come later in the frame. Twins are the
    // same articulation, sharing one hinge axis and one clinical conversion, so either value
    // carries across unchanged.
    _localAnimRotByJointId.fill(null);
    _clinicalAngleRadByJointId.fill(null);
    for (let i = 0; i < frame.jointsLength(); i++) {
        const jp = frame.joints(i);
        const id = jp.id();
        _localAnimRotByJointId[id] = jp.localAnimRot();
        _clinicalAngleRadByJointId[id] = jp.sagittalClinicalAngle();
    }

    for (let id = 0; id < BONE_BY_JOINT_ID.length; id++) {
        const name = BONE_BY_JOINT_ID[id];
        const bone = name ? bones[name] : null; // marker end sites drive nothing
        if (!bone) { continue; }
        const twin = mirrorUnmeasured ? MIRROR_BY_JOINT_ID[id] : id;

        if (fromClinicalAngle) {
            const sign = CLINICAL_ANGLE_SIGN_BY_JOINT_ID[id];
            const clinicalAngleRad = _clinicalAngleRadByJointId[id] ?? _clinicalAngleRadByJointId[twin];
            if (sign === 0 || clinicalAngleRad === null) { continue; }
            _delta.setFromAxisAngle(_hingeAxis, sign * clinicalAngleRad);
        } else {
            const q = _localAnimRotByJointId[id] ?? _localAnimRotByJointId[twin];
            if (!q) { continue; } // unmeasured joint (null rotation)
            _delta.set(q.x(), q.y(), q.z(), q.w());
        }

        bone.quaternion.copy(bindPose[name]).multiply(_delta);
    }
}

// Every field of one decoded PoseFrame as a plain object.
// Nulls stay null and numbers keep the value that arrived, in the wire's own units. 
export function dumpPoseFrame(frame) {
    const joints = [];
    for (let i = 0; i < frame.jointsLength(); i++) {
        const jp = frame.joints(i);
        const id = jp.id();
        const q = jp.localAnimRot();
        joints.push({
            id,
            joint: JOINT_NAME_BY_ID[id] ?? '?',
            bone: BONE_BY_JOINT_ID[id],
            local_anim_rot: q ? { x: q.x(), y: q.y(), z: q.z(), w: q.w() } : null,
            sagittal_segment_angle: jp.sagittalSegmentAngle(),
            sagittal_clinical_angle: jp.sagittalClinicalAngle(),
            sagittal_included_angle: jp.sagittalIncludedAngle(),
        });
    }
    return {
        frame_seq: frame.frameSeq(),
        // `timestamp_us` is stringified because the wire's 64-bit value decodes to a BigInt, which JSON.stringify refuses.
        timestamp_us: String(frame.timestampUs()),
        has_rest_pose: frame.hasRestPose(),
        joints,
    };
}

// A `dumpPoseFrame()` result as overlay text: one line per joint, 
// keyed by the JointPose field names so a value on screen names the field it came off. 
// The angles are shown in degrees, which the header states; the wire carries radians.
export function formatPoseFrameDump(dump) {
    const deg = (rad) => (rad === null ? 'null' : (rad * 180 / Math.PI).toFixed(3));
    const quat = (v) => (v === null
        ? 'null'
        : `(${v.x.toFixed(6)}, ${v.y.toFixed(6)}, ${v.z.toFixed(6)}, ${v.w.toFixed(6)})`);

    const lines = [
        `frame_seq=${dump.frame_seq} timestamp_us=${dump.timestamp_us} has_rest_pose=${dump.has_rest_pose}`,
        'JointPose fields per joint. angles [deg], local_anim_rot (x, y, z, w)',
        '',
    ];
    for (const j of dump.joints) {
        lines.push(
            `${j.id} ${j.joint.padEnd(6)}`
            + ` sagittal_segment_angle=${deg(j.sagittal_segment_angle)}`
            + ` sagittal_clinical_angle=${deg(j.sagittal_clinical_angle)}`
            + ` sagittal_included_angle=${deg(j.sagittal_included_angle)}`
            + ` local_anim_rot=${quat(j.local_anim_rot)}`
        );
    }
    return lines.join('\n');
}

