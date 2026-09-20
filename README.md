# Viscous Arm Simulation

CAD-derived URDF, MuJoCo simulation, and a browser model explorer for the Viscous Arm. Built from Gavin's 18 September 2026 STEP/F3Z exports. These are already the shorter Viscous assembly, despite their Maker Arm filenames.

This is an offline model for inspection, kinematics, collision queries, and initial simulation. It does not contain a hardware controller or a verified encoder-to-URDF mapping.

## How this model was made

See [CAD to URDF: repeatable workflow for another robot arm](docs/CAD_TO_URDF_WORKFLOW.md) for the extraction process, rigid-link mapping, joint-frame choices, mass/inertia calculations, collision modeling, validation, and the exact code locations to adapt. The current scripts are specific to this arm; they are not an automatic arbitrary-CAD converter.

## Open

- Browser explorer: start `python viewer/server.py`, then open http://127.0.0.1:8766/viewer/
- URDF: `model/viscous_arm.urdf`
- MuJoCo: `model/viscous_arm.xml`
- Complete assumptions, per-part ownership and physical properties: `model/model_manifest.json`
- Editable mass/material assumptions: `materials.json`
- Numerical validation: `inspection/validation_report.json`
- Independent reviews: `inspection/baseline_audit.json`, `inspection/model_physics_validation.json`, `inspection/fresh_kinematics_review.json`
- Rendered poses: `inspection/cad_zero.png`, `inspection/inspection_pose.png`, `inspection/extended_pose.png`

The browser's seven independent sliders represent six arm rotations plus symmetric gripper opening. The second finger is a mimic joint. No controls in this project send commands to the physical arm.

**Preserve progressive assembly as a viewer feature.** Gavin specifically wants to keep the effect where the CAD appears piece by piece on refresh. Load meshes concurrently and display each as it becomes ready while rendering continues. Do not replace this with an all-at-once reveal or add artificial loading delays; the assembly order and speed may naturally vary with caching and file size.

## Geometry and frames

The CAD has 72 solids. The two finger loops (44,45) are excluded entirely because Gavin confirmed the real arm does not have them. The remaining 70 solids contribute to mass. Ten internal crank/rod/bearing pieces are omitted from visible/contact geometry in the simplified gripper; their mass and inertia are retained in the wrist at the exported CAD pose. No loops are hidden collision objects.

Motor barrel surfaces determine the rotation-axis lines. The measured J2–J3 and J3–J4 spacings are 160 mm and 150 mm; both are 190 mm in the reference Maker URDF. An independent comparison found a 4 mm J5-axis discrepancy in the old reference, so the new model uses new CAD axis positions for the entire arm.

Robot coordinates are Z-up, metres: XYZ = (-CAD Y, -CAD Z, CAD X) / 1000. The underside of the mounting plate is Z=0. Joint frames are located on the relevant axis and share the robot orientation at CAD zero; their axis vectors specify local rotation directions. These frame choices are valid URDF conventions, not motor calibration.

`tool0` is the area-weighted midpoint of the opposing inner finger-pad faces. Its +Z points toward the fingertips, +X follows the left jaw's opening direction, and +Y completes a right-handed frame. It is a geometric TCP, not a physically measured probe calibration.

CAD zero is close to physical zero but not exact (Gavin). Positive rotation signs follow the reference convention and have been checked for internal geometric consistency, not measured against encoders. Recorded motor limits are preserved separately in the manifest; URDF limits provisionally assume zero offset and include CAD zero. Do not treat them as calibrated physical limits.

## Physical assumptions

Current estimated total mass is approximately 3.51 kg, including base and a 30 g camera placeholder; it is not a weighing result.

- Seven RS00 actuator solids: 0.310 kg each. Their CAD shape is normalized to that mass to estimate COM and inertia. Rotor/transmission inertia is not identified separately. Manufacturer source: https://github.com/RobStride/Product_Information (detailed sources in the physics review).
- Structural CNC aluminium: 2700 kg/m3, likely 6061 per Gavin; alloy not confirmed.
- Bearings, linear guide parts and bolts: provisional steel 7850 kg/m3, subject to weighing.
- Gripper fingers and camera bracket: PLA+ confirmed by Gavin. Nominal solid density 1240 kg/m3 times 30% effective solid fraction = 372 kg/m3. Gavin estimates actual slicer infill at 15%, with 30% used to allow for perimeters. Uniform effective-density inertia is an approximation; it does not reconstruct shells/perimeters.
- Finger loops: absent, no mass/visual/contact geometry.
- Camera: 30 g placeholder; replace in `materials.json` when measured.
- Missing cables, screws, potting/waterproofing additions and payload are not silently invented.

Full CAD volume properties and the parallel-axis theorem produce each link's COM and inertia. Motor weights override uniform aluminium assumptions. All generated inertia tensors pass positive-definiteness and triangle-inequality checks.

## Contact and actuation

Each arm span now has one continuous solid collision envelope: J2–J3 combines both side plates and their joint hardware; J3–J4 combines all four plates, all six cylindrical spacers and their joint hardware. The space between the plates and spacers is deliberately filled. These envelopes follow the assembled outer profile and use 3 and 8 convex simulation pieces respectively, rather than separate colliders for every component. The original CAD appearance, joint definitions, masses and inertias are unchanged. Rail and finger collision envelopes retain their broad-view outlines while filling internal holes/deep grooves (25 pieces for the rail base, 23 per finger). Selected other brackets retain modest convex compounds. [collision_settings.json](collision_settings.json) records each policy and assembly membership. Tiny fasteners/bearings and the omitted crank mechanism have no separate contact geometry. Parent/child contacts at mechanical interfaces are excluded; nonadjacent collisions remain enabled. Enable **Collision** in the browser to inspect the result.

The offline builder uses [Shapely](https://shapely.readthedocs.io/en/stable/) for filled-outline partitioning and [CoACD](https://github.com/SarahWeiii/CoACD) for other selected brackets; running the generated model requires neither library. CoACD intermediate results are cached in `inspection/.collision_cache/` by source geometry, settings and library version. Delete that directory to force decomposition again. [Collision validation](inspection/collision_validation.json) checks CAD surface coverage within 2 micrometres of STL export tolerance, the rail/finger silhouettes within the same tolerance, and solid interior coverage across both complete arm spans. Assemblies apply a 25 micrometre profile cleanup with a 25 micrometre outward margin to avoid tessellation notches; their conservative profile overfill is checked against a 0.1 mm bound. All assembly pieces remain inside the complete 3D convex envelope, subject to export rounding.

[Collision validation](inspection/collision_validation.json) records current piece/triangle counts and machine-specific query timings against the original single-hull policy. These query timings are not full simulation throughput. Generate a CAD/collision-envelope comparison with `.venv/bin/python scripts/render_collision_comparison.py`, which writes `inspection/collision_comparison.png` and includes broad and side views of a finger.

Generate an isometric CAD/solid-envelope comparison of both arm spans with `.venv/bin/python scripts/render_arm_envelopes.py`; output is `inspection/arm_envelopes.png`.

Generate opposite isometric views of the CAD finger and its collision envelope with `.venv/bin/python scripts/render_finger_isometric.py`; output is `inspection/finger_isometric.png`. These are orthographic renders of the actual meshes with depth buffering. An individual contact piece is a **finger** (or **gripper finger**); the **gripper** is the complete assembly of fingers, guides and actuation mechanism.

MuJoCo has fixed base, gravity -9.81 m/s2, 1 ms integration, explicit inertias and seven bounded direct force/torque actuators. The two jaw sliders are coupled with a joint equality.

Arm torque is capped at +/-5 Nm, the RS00 nominal rated specification. The manufacturer's 14 Nm peak is not assumed sustainable. Rated performance depends on voltage, cooling and duration. The URDF's 2 rad/s velocity metadata is a chosen provisional cap; MuJoCo torque motors do not enforce it. Joint damping/friction/reflected rotor inertia remain zero/unidentified. The generic contact-friction setting is unmeasured.

The single gripper actuator's +/-10 N is generalized force conjugate to one jaw's displacement under the two-jaw equality: roughly 5 N per jaw with symmetric bilateral contacts. It is a simulation placeholder, not measured gripping force. Travel 52.4125 mm per jaw is inherited from the Maker reference and requires physical verification. CAD has a small residual closed gap. No linear or nonlinear encoder-to-gap calibration is supplied. Simplifying the crank into sliders is appropriate for initial kinematics but not transmission-accurate contact-force research.

## Reproduce

The verified environment is Python 3.10 on Linux with NVIDIA EGL rendering. Clone normally; Git LFS is not required:

```sh
git clone https://github.com/gavingavinchan/viscous-arm-simulation.git
cd viscous-arm-simulation
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
```

Rebuild the CAD-derived model and validate it:

```sh
.venv/bin/python scripts/inspect_cad.py
.venv/bin/python scripts/build_model.py
.venv/bin/python scripts/build_mujoco.py
.venv/bin/python scripts/validate_collisions.py
MUJOCO_GL=egl .venv/bin/python scripts/validate_model.py
```

`requirements-lock.txt` records the installed Python environment. For another machine, create a Python 3.10 environment and install that file. The virtual environment lives in this project; CAD and simulation dependencies do not replace system packages.

Start the local viewer:

```sh
.venv/bin/python viewer/server.py
```

The viewer binds `127.0.0.1:8766` by default. Set `--host` and `--port` explicitly for another interface. All viewer assets are local.

Interactive desktop simulation, with bounded controller holding the inspection pose:

```sh
.venv/bin/python scripts/simulate.py
```

This command requires a graphical desktop session. Close its window to stop the simulation. The model does not access CAN, robot serial ports or cameras.

`kinematics.py` provides `fk(q)` and `inverse(target, seed)`. IK is local numerical pose IK and does not by itself find collision-free paths; check a candidate in MuJoCo before treating it as a feasible simulated motion. Neither function converts physical encoder values.

## Validation and remaining work

The automated check compares FK with compiled MuJoCo frames at100 random configurations, solves12 reachable pose targets from perturbed seeds, checks three inspection poses for contact, and runs a two-second gravity-compensated hold with bounded torque. These checks establish consistency of the generated model, not accuracy of unmeasured physical parameters.

Remaining physical validation: known-pose encoder readings/photos for sign/zero mapping; measured jaw opening vs J7 encoder; actual camera/printed-part masses; hardware additions absent from CAD; friction, compliance, backlash, rotor inertia and thermal/torque identification. Dry-air scope only; underwater buoyancy/drag is not included.

## Pick-and-place demonstration

The demo grasps a 20 mm cube with physical finger contact, lifts it, carries it to an open cup, and releases it. The cube is a free body; there are no attachment welds or object-position resets.

```sh
MUJOCO_GL=egl .venv/bin/python scripts/pick_place_video.py --render
```

Output: `demo/viscous_pick_place_20mm.mp4` (1280 × 960, 30 fps, 21.4 seconds). Omit `--render` for numerical validation only. The scene uses a 10 g cube, provisional friction, a 25 mm pickup platform, a 50 mm high cup, and bounded position servos with model gravity feedforward. The original base model remains unchanged.

The verified run settled the cube inside the cup, with maximum arm torque 3.725 Nm, maximum arm speed 0.523 rad/s, no joint-limit excursions, no unintended robot collisions, and no solver warnings. See `demo/pick_place_report.json` and `demo/independent_demo_review.json`. This is a scripted simulated demonstration, not a trained policy or physical-arm test.

The refined collision model passed the numerical demonstration again; `demo/pick_place_report.json`, its scene and trajectory log reflect that run. The existing video and independent reviews describe the earlier collision revision; the video has not been re-rendered for this change.

Videos, intermediate CAD tessellations, environments, caches, and ZIP exports are not tracked. Source CAD and runtime meshes are stored directly in regular Git. Refer to `THIRD_PARTY_NOTICES.md` for asset provenance and third-party licenses.
