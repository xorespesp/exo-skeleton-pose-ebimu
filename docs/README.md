# Markers Mapping

Each lower-limb joint carries one AprilTag marker.

- Family: `tagStandard41h12`
- Tag ids: `0 .. 6`
- Printable sheet: `tagStandard41h12.pdf`
- On-sheet layout: `Coordinates_tagStandard41h12.json`

> Generated from: https://shiqiliu-67.github.io/apriltag-generator/

Print the sheet, cut the tags out, and stick one on each joint per the table below. 
The black-square edge length is what the estimator needs (default 0.05 m): `marker.tag_size_m` in
the installation config, or `--tag-size` when running without one.
Measure your printed tags and set it to match.

## Tag to joint mapping

| Tag id | Joint   | Side  |
|--------|---------|-------|
| 0      | pelvis  | root  |
| 1      | r_knee  | right |
| 2      | l_knee  | left  |
| 3      | r_ankle | right |
| 4      | l_ankle | left  |
| 5      | r_foot  | right |
| 6      | l_foot  | left  |

`pelvis` (tag 0) is the root. The rest chain off it per leg:
`pelvis -> knee -> ankle -> foot`. 
Keep the ids consistent with the mapping, otherwise the joints will be swapped.

## Tag mounting

Only each tag's 3D position is used: the estimator takes the tag's camera-space position as the joint's position and runs IK over the per-joint positions. (tag's rotation is not used)

What does matter:

- Placement: Stick each tag as close to the joint's rotation center as practical, rigidly so it moves with the segment. A fixed offset from the true joint center is absorbed into the captured rest reference; only large or shifting offsets distort the angles.
- Visibility: Keep the tag facing the camera enough to be detected reliably throughout the motion.

Every leg joint is modeled as a 1-DOF forward/back hinge about a single lateral axis shared by all joints.
That axis is a camera-frame setting (frontal view ≈ camera X, sagittal/side view ≈ camera Z), chosen in the debugger's Leg Hinge control; 
it is a property of the viewing geometry, not of how a tag is attached.

Capture the rest pose (`Calibrate`) with the joints in a clean neutral stance.
The stance can be any neutral pose (e.g. with the foot already 90 degrees forward of the shank); 
that offset is folded into the captured rest reference.
