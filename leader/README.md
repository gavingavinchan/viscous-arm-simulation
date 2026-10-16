# Star Arm 102-HD leader model

CAD-derived replacement for the manufacturer's shared LD/HD URDF, for Gavin's
standard purple 102-HD with its original handle and finger rings. The standard
mechanical configuration was confirmed by Gavin on 15 October 2026.

Open [model/star_arm_102_hd.urdf](model/star_arm_102_hd.urdf). Keep its `meshes/`
directory beside it. All meshes are already in metres; angles are radians.

From the repository root, start the existing viewer:

```sh
.venv/bin/python viewer/server.py --port 8776
```

Then open <http://127.0.0.1:8776/viewer/?model=leader>. The viewer switches between
the leader and Viscous models and retains progressive, piece-by-piece loading.
Every control is offline.

## What was corrected

- J1–J6 are revolute. In particular, J5 now rotates about the analytic shaft axis
  measured from the HD STEP, corroborated by the mating wrist part's cylindrical
  seat. The old shared URDF had a fixed J5 with a zero axis vector.
- Geometry and all six axis lines are reconstructed from the placed HD assembly,
  rather than copying the ROS 2 follower's different gripper and wrist geometry.
- Servo output shafts and horns follow their attached moving link. J5 is mounted
  in reverse: its output shaft belongs to `link4`, while its case, mounting
  bracket and subsequent wrist belong to `link5`.
- One input, `joint7_left`, opens both finger rings; `joint7_right` mimics it with
  multiplier −1. Both rings and the original handle are retained.
- `tool0` is the center of the handle's 16 mm radius grip cylinder, at the midpoint
  of its axial extent. Its +Z points down the handle and +X follows CAD +X, toward
  the opening direction of the left ring. This is a geometric hand reference,
  not an experimentally calibrated TCP.
- Relative mesh paths work without a ROS package resolver. All 219 solids and
  one non-solid terminal-block surface occurrence are preserved exactly once.

The upstream joint names `joint1`–`joint6`, `joint7_left`, and `joint7_right` are
retained. Servo IDs 0–5 correspond to J1–J6; servo 6 drives the ring mechanism.
The manifest records the LeRobot motor aliases. This correspondence does **not**
establish servo signs, offsets, or the conversion to a follower's motor angles.

## Evidence and limits

The pinned manufacturer source is
[`servodevelop/Star-Arm-102` at `b6eae72210082778976643c636655502ebb17417`](https://github.com/servodevelop/Star-Arm-102/tree/b6eae72210082778976643c636655502ebb17417).
`source/hd_assembly.STEP` is the manufacturer's
`hardware/102-hd/step/assembly/star-arm-102-hd-assembly-export-2026-10-12.STEP`.
[source/intake.json](source/intake.json) records original paths, sizes and SHA-256
hashes. Both upstream URDFs are retained for comparison. The manufacturer's
[license scope note](source/LICENSE-scope.md) is preserved without adding a new
license grant for its assets.

[model/model_manifest.json](model/model_manifest.json) records every part owner,
shaft selector, axis, coordinate transform, joint definition and assumption.
[inspection/parts.json](inspection/parts.json) contains analytic CAD measurements.
[inspection/validation_report.json](inspection/validation_report.json) records
checks of the emitted file, not just the builder's in-memory model:

- Valid rooted URDF tree, local assets, unique part accounting, input/output hashes.
- Exported mesh vertices coincide with the STEP tessellation at CAD zero, within
  0.05 micrometres after coordinate conversion and STL rounding. This checks the
  export transform, **not** the physical accuracy of the manufacturer CAD.
- All six arm axes agree with coaxial features on a second CAD part.
- 101 configurations agree between URDF FK, spatial rotations about the raw CAD
  axis lines, and MuJoCo's importer of a temporary copy of the emitted URDF.
- All 100 sampled nonzero arm poses have a rank-six hand-pose Jacobian. A 30° J5
  change moves the geometric hand reference by about 37.51 mm and rotates its
  orientation while the preceding link remains fixed.
- The ring mimic opens the rings in opposing directions and leaves `tool0` fixed
  relative to the handle.

Remaining qualifications:

- CAD zero is the exported assembly pose. Physical encoder signs and offsets
  still need known-pose measurements. No hardware calibration or active mapping
  files were changed.
- Joint limits are preview intervals based on nominal manufacturer travel. Their
  placement about CAD zero, especially J2/J3, is unverified. Effort 1 Nm and
  velocity 1 rad/s are required URDF metadata chosen for preview, not actuator
  ratings or controller settings. They must not replace measured hardware limits.
- The source places the left ring bore 0.15 mm sideways from its motor shaft.
  Geometry is preserved and the driven joint follows the shaft. The right ring's
  axis is independently corroborated by its handle mounting hole. Equal opposing
  ring motion models the gears kinematically; backlash and compliance are absent.
- This is a **kinematic model**. No HD masses/inertias are asserted. Using the
  shared LD/HD file's unvalidated inertias for gravity compensation would require
  additional identification. Validation-only synthetic inertias exist solely in
  memory to let MuJoCo compile the kinematic tree; they are not in the output URDF.
- Selected CAD surfaces are provided as collision meshes. Dynamic engines may
  convexify them; contact dynamics, friction and motor limits are not validated.
- The source omits cables and the HD button assembly. Bearing internal motion is
  simplified: the single thrust-bearing CAD solid is carried by `link1`.

## Rebuild and validate

Use the repository's pinned Python environment (`requirements-lock.txt`). From
the repository root:

```sh
.venv/bin/python leader/inspect_cad.py
.venv/bin/python leader/build_model.py
.venv/bin/python leader/validate_model.py
.venv/bin/python leader/package_model.py
```

Inspection imports the STEP with assembly placements, inventories solids and the
surface component, measures analytic cylinders, and tessellates at 0.05 mm linear
and 0.2 rad angular deflection. Inspection STLs and PNGs are regenerable local
artifacts. The builder rejects a changed input hash until extraction is rerun.
Its selectors and ownership map are specific to this pinned assembly.

The body frames have a common orientation at CAD zero. Coordinates are:

```text
robot XYZ = (-(CAD Z + 7.5), -CAD X, CAD Y) / 1000
```

The base origin is on J1's axis at the mounting floor. Joint origins lie on each
measured axis; their position along that axis is a frame convention. Signs follow
the old reference's geometric convention and require separate encoder checks.

Offline FK is available as `leader.kinematics.fk(q)` and
`leader.kinematics.handle_pose(q)`, with six arm angles followed by ring opening,
all in radians. These functions neither clamp to hardware limits nor convert
encoder readings.

`star_arm_102_hd_model.zip` is a portable runtime package containing the URDF,
meshes, manifest, validation reports, offline joint-mapping drafts and
source/license records. The STEP and rebuild scripts remain in this repository
directory. The included joint-mapping drafts are not physically verified.

The next mapping step is to measure the leader's encoder-to-model signs and
zeros, then compare hand motion with each Viscous model. The waterproofed and
non-waterproofed followers retain their own measured calibration and limits.
See [joint-posture mapping and next-session checks](MAPPING.md) for the selected
one-to-one joint mapper, both offline draft profiles and the waterproof unit's
J1 frame correction. The Cartesian mapper remains a separate offline experiment.
