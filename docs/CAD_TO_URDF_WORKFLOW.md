# CAD to URDF: a repeatable workflow for another robot arm

This is the method used to create the Viscous Arm model, including the manual engineering decisions. It is a reusable procedure, **not a claim that the current scripts accept arbitrary robot CAD**. The scripts encode this arm's part indices, topology, coordinate convention, and gripper.

For the supported Viscous rebuild, use the commands below unchanged. For another arm, first complete the mapping and checks in sections 2–7, then adapt the listed code locations.

## 1. Preserve inputs and establish the modeled configuration

Obtain an assembly STEP with components/solids and their assembly placements preserved. Also retain the original native CAD archive: here the F3Z was archived and checked, but **STEP is the build input**. This build used CadQuery/OpenCascade programmatically; Blender and a Fusion URDF exporter were not used.

Record:

- Exact CAD revision, source filenames, SHA-256 hashes, units, and exported pose.
- Which components are physically fitted, missing from CAD, or present in CAD but absent in reality.
- Motor types and known mass data, structural materials, printed-part assumptions, camera/tool/payload masses.
- Intended operating conditions: fixed or floating base, gravity, air or water, contact tasks.
- Whether CAD zero corresponds to calibrated encoder zero. If not known, keep these separate.

For Viscous, the exported files were already the modified arm even though their names said Maker Arm. The real arm had no finger loops, so parts 44/45 were excluded. The model is dry-air only. The CAD pose is near, but not established as, physical zero.

Files: [intake record](../source/2026-09-18/intake.json), [material inputs](../materials.json), [model manifest](../model/model_manifest.json).

## 2. Inspect the CAD before assigning joints

[scripts/inspect_cad.py](../scripts/inspect_cad.py) imports STEP into a CadQuery assembly and applies each component's assembly location to its solids. It extracts:

- Assembly/component name and occurrence index.
- Analytic solid volume, center of mass, and central volume-inertia tensor.
- Bounding box and selected cylindrical faces: axis point, direction, radius, and face area.
- An inspection STL for each placed solid.

The current cylinder filter retains only radii greater than 10 mm. That is a Viscous inspection heuristic, not a general motor detector. Smaller joints, noncylindrical joints, or different CAD representations need other features.

The output `inspection/parts.json` is an inventory, not an automatically inferred robot. Part indices are enumeration order and can change after a CAD re-export or importer update. Check names, geometry, dimensions, and assembly placement together. Repeated identical motors cannot be distinguished by volume alone.

The Viscous assembly contained 72 instantiated solids. The excluded loops leave 70 solids contributing mass. Run extraction again after any input CAD change; the builder does not automatically detect stale inspection data.

Tessellation uses absolute 0.08 mm linear deflection and 0.25 rad angular deflection (`relative=False`). Mass properties come from the analytic CAD solid, not from this tessellated mesh. Choose and inspect tolerances for the new robot's smallest relevant features.

## 3. Build the rigid-link ownership map manually

A URDF link is a group of solids that move rigidly together. It is not necessarily one CAD component, one STL, or one whole actuator.

For every solid, decide which body owns it. In particular, determine which side of each motor or bearing is stationary relative to the parent and which belongs to the moving child. Use assembly constraints if available, CAD inspection, and knowledge of how the real mechanism moves.

Complete this worksheet before editing the exporter:

| Input | What to record |
| --- | --- |
| Solid/occurrence | Stable identifying features and current inventory index |
| Rigid link | Body to which the solid is attached |
| Treatment | Visual, collision, mass only, or physically absent |
| Joint parent/child | Two rigid bodies connected by each joint |
| Joint type | Fixed, revolute, continuous, prismatic, or a modeled coupling |
| Axis and pivot | Measured CAD axis line and chosen frame origin |
| Coordinate frame | Units and orientation relative to the assembly |
| Zero/limits | CAD reference, measured encoder mapping if available, confidence |
| Mass source | Weighed value, manufacturer value, CAD density, or estimate |
| Tool frame | Physical/geometric meaning of origin and axes |

Viscous ownership is explicit in `GROUPS` in [build_model.py](../scripts/build_model.py). Its assertion requires every included solid exactly once. Preserve this invariant for a new robot, replacing the excluded-part list as well.

Do not silently drop moving internal mechanisms. Here ten crank/rod/bearing solids are mass-lumped into the wrist at the exported pose and omitted from visible/contact geometry. That is a documented simplified gripper, not a dynamically exact model of the mechanism.

## 4. Locate joint axes and select coordinate conventions

Use actual motor/bearing geometry and the mechanism to identify each rotation-axis line. A cylindrical face supplies a point and direction, but the largest cylinder is only a candidate. Verify the coaxial surfaces and moving bodies.

For Viscous:

- J2/J3/J4 use motor-barrel cylinder axes and their intersection with a chosen CAD plane.
- J1 has a manually chosen CAD pivot.
- J5 uses a selected cylinder plus a frame-origin offset along its axis.
- J6 uses a selected cylinder on a neighboring component.
- CAD part indices such as 19, 18, 58, 53, and 49 identify geometry; they are not verified hardware CAN IDs.

These are deliberately explicit in `cyl`, `plane_center`, `j1…j6`, `PIVOTS`, and `axes`. None of these selectors is transferable without review.

The Viscous coordinate map is:

```text
robot XYZ = (-CAD Y, -CAD Z, CAD X) / 1000
```

This rotates CAD X-up into robot Z-up and converts millimetres to metres. All moving-link frame orientations initially share the robot orientation at CAD zero; their origins sit on the selected joint axes. Consequently the current joint origins can use differences of pivot positions.

For differently oriented joint frames, use complete rigid transforms:

```text
parent_T_child_at_zero = inverse(world_T_parent) @ world_T_child
axis_in_joint_frame = world_R_joint.T @ axis_in_world
point_in_link = world_R_link.T @ (point_in_world - link_origin_in_world)
```

Do not copy the Viscous subtraction-only origin calculation into a differently framed model. Normalize axes and preserve right-handed frames. URDF joint axes are expressed in the joint frame; positive rotations follow the right-hand rule.

A point anywhere along the same revolute axis represents the same rotation line, but moving frame origins requires consistent updates to joint origins, meshes, COM, and inertia.

A reference URDF can help identify conventions and matching parts. [compare_reference.py](../scripts/compare_reference.py) ranks candidates by volume; it does not prove correspondence or topology. In this build the reference's two 190 mm spacings became 160 mm and 150 mm, and an independent check found a 4 mm J5-axis discrepancy. The new CAD, rather than a scaled old URDF, determined the final geometry.

## 5. Generate visual, collision, and tool geometry

Transform each placed CAD solid into its owning link's local frame and convert to metres. The generated Viscous STLs are already in metres, so the URDF does not apply a second 0.001 scale.

- **Visuals:** preserve the visible CAD geometry and assign display materials.
- **Collisions:** choose the intended contact envelope explicitly. [outline_collision.py](../scripts/outline_collision.py) preserves the projected outer perimeter of arm assemblies, rail base and fingers, removes all projected interior rings, partitions that filled polygon into convex regions, and intersects each region’s prism with the full 3D convex envelope. This preserves concave outer edges while filling holes and depth grooves. [collision_geometry.py](../scripts/collision_geometry.py) uses CoACD for selected other brackets. [collision_settings.json](../collision_settings.json) records the collision policies and piece budgets. Every convex region is exported as a separate collision mesh, because a single mesh would let MuJoCo convexify the whole part and erase the required concave outline again.
- **Absent parts:** remove their mass, visuals, and collisions.
- **Omitted mechanism:** state exactly what is hidden and where its mass is retained.
- **Tiny parts:** collision omission is a modeling choice, not a universal rule.

The current collision filter excludes the `STEEL` list entirely, including rail/block items as well as fasteners. Reassess every exclusion on another arm; material alone is not a valid general collision criterion. The rail/finger projection uses the comparison image’s PCA basis, ordered by physical extent. The arm assemblies use their explicit link XZ profile and bridge the link Y plate spacing; the frame and member solids are recorded in `collision_settings.json`. Assembly membership cannot cross a moving joint. Projected triangle union supplies the silhouette; only interior rings are discarded. Constrained triangulation and convex-only region merges retain the exterior boundary. This is an envelope model: depth grooves are deliberately bridged. For the other brackets, CoACD hull caps can exceed its 5 mm search threshold, so that threshold is not a certified error bound. Those compounds restore source vertices, edge midpoints and triangle centres omitted by simplification. [validate_collisions.py](../scripts/validate_collisions.py) checks the exported meshes, assembly profile bounds, filled interior probes through the complete plate spacing, both directions of rail/finger silhouette coverage, absence of projected interior holes beyond export tolerance, sampled 3D coverage, URDF/MuJoCo asset parity and query timings against the former single-hull policy. Reported compound volumes sum the hulls and count any overlaps. Inspect important contact surfaces and rerun the grasp demonstration after changing these policies.

For J2–J3, source solids 8,9,10,11,18 are combined into one solid collision envelope. For J3–J4, source solids 13,17,58,60–69 include all four side plates (60–63) and all six spacers (64–69). Each member’s mass is still counted exactly once from the CAD, and its visual mesh is preserved. No artificial mass is assigned to the filled cavity. Assembly profile cleanup is 25 micrometres with a 25 micrometre outward margin, and the exported conservative outline is checked within 0.1 mm of the filled CAD silhouette. Surface coverage retains the 2 micrometre export tolerance. The validator also samples the volume between plates, requiring former cavity points to lie inside the new envelope.

Collision vertices are quantized to a 1 micrometre grid and convexified using STL's float32 coordinates before export, avoiding degenerate triangles from almost coincident vertices. The exported meshes are checked for convexity, watertightness and sampled coverage within 2 micrometres. This export tolerance is separate from the much coarser decomposition search threshold.

The rail base applies a 0.5 micrometre boundary cleanup before outline partitioning to remove slivers that would collapse on the STL export grid. Validation still compares its exported silhouette against the original filled CAD projection, within the same 2 micrometre tolerance. This cleanup is recorded per part in `collision_settings.json`.

The Viscous `tool0` origin is the area-weighted midpoint of the opposing inner finger-pad faces. +Z points toward the fingertips, +X follows one jaw's opening direction, and +Y completes the frame. The face-normal and mid-plane masks in the builder are specific to this CAD orientation. Identify the actual contact/tool features again on another robot.

A useful check from the cube demo: these fingertips extend about 34.33 mm beyond TCP along tool +Z. A desired TCP pose alone does not establish clearance between fingers and a table.

## 6. Build the mass model independently of display meshes

Keep a per-part material/mass input with provenance and uncertainty. Prefer measured complete-component mass, then suitable manufacturer data, then density-based CAD estimates. Avoid treating actuator internals as solid aluminium.

For a solid with CAD volume `V_mm3` and central geometric inertia tensor `J_mm5`:

```text
mass_kg = density_kg_m3 * V_mm3 * 1e-9
I_com_kg_m2 = R @ J_mm5 @ R.T * (mass_kg / V_mm3) * 1e-6
```

The same expression works when a known component mass overrides density: it normalizes the CAD shape to that mass. It remains an approximation of the component's internal mass distribution.

Aggregate parts in their link frame using the parallel-axis theorem:

```text
M = sum(m_i)
c = sum(m_i * c_i) / M
I_link_com = sum(I_i_com + m_i * (dot(d_i,d_i)*Identity - outer(d_i,d_i)))
d_i = c_i - c
```

URDF's inertial origin is the link COM; tensor components must be expressed in the corresponding inertial axes. Viscous uses inertial axes aligned with each link frame, so inertial RPY is zero.

Current examples are seven nominal 310 g RS00 assemblies, assumed aluminium at 2700 kg/m³, provisional steel at 7850 kg/m³, a 30 g camera placeholder, and PLA+ fingers/bracket at `1240 × 0.30 = 372 kg/m³`. The 30% is Gavin's effective solid fraction, allowing approximately for perimeters; it is not a reconstruction of slicer shells/infill or their true inertia.

**Keep the checked-in materials.json when reproducing this arm.** If absent, the current builder silently creates older provisional defaults, including 1200 kg/m³ polymer values. Missing entries also fall back to defaults, while some assumption text is fixed. Only the final mass_kg or density_kg_m3 values drive the calculation; solid_density_kg_m3 and effective_solid_fraction are metadata, so editing the fraction alone does not recalculate density. A new robot needs a complete, reviewed material map and matching manifest text; do not assume deleting the file creates equivalent inputs. A material's `excluded` flag is not consumed by this builder; topology/exclusion lists control membership.

Check positive masses, COM plausibility, tensor symmetry, positive eigenvalues, principal-moment triangle inequalities, total mass, and assembly COM. Check a separate aggregation independently of the exporter. Record unmodeled cables, electronics, potting, fasteners, payload, and rotor/transmission inertia.

## 7. Set joints, limits, couplings, and simulator assumptions

The current arm has six revolute joints plus two opposed prismatic jaws. One jaw mimics the other, giving seven independent coordinates. Its 52.4125 mm per-jaw travel came from the reference model and remains physically unverified.

For a new robot, define the actual topology and coupling. A four-bar, cable drive, geared differential, or nonlinear gripper transmission cannot be made dynamically correct merely by copying these symmetric sliders.

Keep CAD zero separate from measured hardware mapping. A future calibration may use a documented sign/offset conversion; nonlinear mechanisms need their own measured mapping. The present scripts provide no hardware mapping or robot I/O.

Review independently:

- Joint position ranges and their coordinate reference.
- Continuous versus peak actuator torque and thermal assumptions.
- Gear ratios, reflected inertia, damping, friction, and compliance.
- Gripper generalized force versus force at each finger.
- Payload and environmental effects.

Viscous uses provisional 5 Nm arm effort and 2 rad/s URDF velocity metadata. MuJoCo direct torque motors do not enforce that velocity metadata. The single 10 N generalized gripper force corresponds to roughly 5 N per jaw for symmetric opposing contacts in this simplified mechanism; it is not a measured RS00-to-fingertip conversion.

[build_mujoco.py](../scripts/build_mujoco.py) generates MuJoCo from the model manifest, not by importing the emitted URDF. It creates an explicit-inertia model with fixed base, gravity, direct bounded actuators, the two-jaw equality, and parent/child contact exclusions. Adapt these assumptions for the new topology. Continuous joints, floating bases, other grippers, and alternate coupling types are not handled generically.

The compiler uses `inertiafromgeom=false`. Any added moving demo object must have explicit mass and inertia; a mass attribute on its geometry alone is insufficient in this configuration.

For stable contact demos, the final pick-and-place script uses MuJoCo position servos with implicitly integrated velocity feedback, force limits, and model gravity feedforward. An earlier explicit high-gain velocity-feedback attempt caused oscillation; it was replaced before the delivered video. This is numerical controller tuning, not identified real actuator behavior.

## 8. Validate in layers, including an independent check

Do not equate a plausible render or a passing simulator run with a physically calibrated robot.

| Layer | Required evidence |
| --- | --- |
| Source/accounting | Known CAD hash, correct instantiated solids, exactly one mass owner per included part, explicit exclusions |
| Geometry | Assembled CAD-zero overlay, dimensions/pivots, no frame or scale errors |
| Joint motion | Rotate/slide one joint at a time; attached pieces move together; signs, axes, limits, and mimic behavior are sensible |
| URDF | Parse the emitted URDF itself; valid rooted tree, asset paths, units, joint origins, and inertia conventions |
| Kinematics | Compare independently derived FK with the emitted model at multiple poses; check tool frame and reachable IK targets |
| Physics | Independent mass/inertia aggregation, plausible gravity torques, finite bounded-control simulation, no solver warnings |
| Contact | Intended finger/object contacts, clearance through the whole trajectory, no unintended penetration, object settles after release |
| Hardware | Known-pose encoder/measurement checks and weighed components; separate work from offline simulation |

The bundled [validate_model.py](../scripts/validate_model.py) checks URDF tree/assets, 100 FK configurations, 12 reachable IK targets from nearby seeds, several inspection poses, and a two-second hold. Its FK implementation and MuJoCo generator both consume the same manifest. It does **not** compile the emitted URDF into another independent dynamics engine, and an internally consistent wrong axis or part map can pass. The IK tests are local checks, not global reachability or collision-free path planning. An optimizer success flag means termination, not an accurate reachable solution: inspect residual and pose error. The current validator reports contact penetrations, excessive static torque, and solver-warning counts without asserting all are zero; review those fields explicitly.

The saved fresh-kinematics and physics reviews in `inspection/` are evidence from this build, not generic scripts that automatically re-audit a changed robot. Arrange a fresh reviewer or an independent parser/implementation for a new model. Give the reviewer raw CAD measurements and emitted files, not only the exporter's conclusions. Preserve findings and reviewed file hashes.

For a manipulation demo, additionally verify actual bilateral contact during lift, object clearance, force/speed/range bounds, absence of object pose resets or artificial attachment, and all object corners inside the target with low final velocity. The cube demo's reports record these separately from rendering.

## 9. Rebuild this exact Viscous model

Verified base environment: Linux, Python 3.10, CadQuery 2.7.0 / OpenCascade bindings 7.8.1.1.post1, MuJoCo 3.13.0. Full package versions are in [requirements-lock.txt](../requirements-lock.txt). NVIDIA EGL was used for headless rendering; an appropriate graphics driver/EGL setup is a system prerequisite. A desktop viewer requires a graphical session. The video script also expects the DejaVu Sans font at its configured Linux path.

```sh
git clone https://github.com/gavingavinchan/viscous-arm-simulation.git
cd viscous-arm-simulation
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt

mkdir -p inspection
.venv/bin/python scripts/inspect_cad.py
.venv/bin/python scripts/build_model.py
.venv/bin/python scripts/build_mujoco.py
MUJOCO_GL=egl .venv/bin/python scripts/validate_model.py
MUJOCO_GL=egl .venv/bin/python scripts/pick_place_video.py --render
```

If the Linux Python installation lacks working `venv/ensurepip`, the original setup used `python3 -m pip install --user virtualenv` followed by `python3 -m virtualenv .venv`. A package lock does not install OS drivers, fonts, or guarantee identical output on other platforms.

The builder imports the source file path in both extraction and manifest hashing; change both when adapting CAD. Preserve `materials.json`. Use a clean checkout/build directory so removed parts do not leave misleading stale meshes or old success reports. Each script overwrites outputs without a transaction; stop on a nonzero exit and do not treat an older report as evidence for a failed rebuild. Extraction regenerates the untracked inspection/part_*.stl files; the checked-in parts.json alone is insufficient.

A clean tracked-file checkout was re-extracted, rebuilt, and validated while writing this guide, using the project's pinned Python environment. Repeating the physics checks is more meaningful than promising byte-identical mesh triangulation across arbitrary CAD-library versions.

## 10. New-arm adaptation map

| Location | Robot-specific content to replace/review |
| --- | --- |
| `scripts/inspect_cad.py` | Source path, assumed CAD units, cylinder radius filter, tessellation tolerances |
| `scripts/build_model.py` | `R`, `GROUPS`, excluded indices, `MOTORS`, `STEEL`, `INTERNAL`, cylinder selectors, pivots/axes, chain order, limits/efforts, gripper topology/travel, TCP selectors, defaults, source hash path, robot name, fixed assumption strings and motor mapping |
| `materials.json` | Every part key and its density/mass provenance; keys must match the new inventory |
| `scripts/build_mujoco.py` | Fixed base, joint type handling, equality coupling, contact exclusions, timestep/contact settings, actuator types, hardcoded keyframes |
| `scripts/kinematics.py` | Joint names/count/order, six-variable IK, right-jaw mimic, target link |
| `scripts/validate_model.py` | State-vector slices, six arm axes/eight movable coordinates, pose fixtures, IK seeds, controller limits/gains, test coverage |
| `scripts/simulate.py` | Model/keyframe names, arm/gripper state slicing, controller and camera |
| `scripts/pick_place_video.py` | Scene dimensions, task poses, IK seed, finger collision names, actuator gains, object indices, success conditions, font/camera |
| `viewer/app.js` | Expected model URL, sample poses, labels/limits; verify the new model is actually loaded rather than reference fallback |
| `scripts/package_model.py` | Output names, mesh checks, explicitly excluded Viscous loop filenames, archive coverage |

The reusable elements are the extraction/transform/inertia mathematics, output structure, and validation discipline. Turning this into a general converter would require moving the robot-specific map into a validated schema and teaching the builders/tests to consume it. That generalization has not been implemented here.

## 11. Handoff for the next person or agent

Commit the source revision, reviewed mappings/material inputs, builders, emitted models/meshes, assumptions, and validation evidence together. Include input hashes, dependency versions, exact commands, expected dimensions/mass, and calibration gaps. Ordinary Git is sufficient for this repository's current asset sizes.

Keep large recordings and datasets out of this code repository. Link the experiment or demo record to its storage location. For Gavin's Viscous project, MakerMods Brain has the project-specific OneDrive storage guide and file index; this public repository does not depend on access to that private storage.
