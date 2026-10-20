# Continue here: Star leader → Viscous physical mapping checks

Mac continuation on 18 October: see [URDF comparison, physical capture and Mac
posture trial](POSTURE_COMPARISON.md), including its final section. The CAD
models now align corresponding link directions/axes with derived CAD offsets.
The Mac paired MIT-feedback capture succeeded without torque enable. Gavin
subsequently clarified that obstructed starting poses should gradually converge
to fixed matching targets; do not preserve starting mismatch with relative
control. A 1:1 posture trial with non-expiring bounded warm-up is loaded in the
Mac GUI, awaiting Gavin's powered motion feedback. Physical fixed encoder
offsets remain provisional; do not equate CAD offsets with encoder offsets.
Saved profiles/motor zeros and the Linux Lab deployment remain unchanged.

Updated 18 October 2026. This is the handoff requested by Gavin for a new agent
session, including a session running on his MacBook.

## User decisions and current hardware location

- Gavin owns the purple **Star Arm 102-HD**, with the standard manufacturer
  handle and finger rings. Its physical J5 works.
- The desired control matches **corresponding joint angles and arm posture**.
  Six arm joints use signed unit angular gain, not travel-ratio scaling and not
  Cartesian tool-pose IK. Different hand positions due to link lengths are expected.
- Both waterproof and original non-waterproof Viscous units need their own
  mapping. Do not copy one unit's zero/limits/gripper reference to the other.
- Gavin deferred physical testing, requested offline preparation, and is now
  discussing the physical check. No physical verification has happened in this
  thread. Guide him through the manual check before a powered test.
- **The arms are currently USB-connected to Gavin's MacBook**, per his latest
  message. The MacBook can reach this Linux computer over Tailscale.
- Mac-to-Linux SSH alias `gavin-linux` is documented in the existing Lab launcher;
  its connectivity was not tested in this handoff session. Reverse SSH into the
  Mac is not established. Tailscale/SSH to Linux does not expose the Mac's USB
  devices: the hardware reader/controller must run on the Mac, or use a separately
  implemented and checked bridge. Do not run the Linux hardware launcher expecting
  it to see the Mac's arms.

From the Mac, the new agent can read this handoff with:

```sh
ssh gavin-linux 'cat /home/gavin/Documents/viscous-arm-model/leader/HANDOFF.md'
```

## Files and local commits

Linux repository: `/home/gavin/Documents/viscous-arm-model`.

- `5c2dcfb`: CAD-derived leader URDF, meshes, provenance, validation and viewer.
- `5b96a29`: earlier offline Cartesian retargeting experiment; **not selected**.
- `44129e0`: selected joint-posture mapper, separate draft profiles and tests.

These commits were made locally. No push, PR, Mac checkout update or Mac
deployment was performed. Check each machine's checkout before assuming it has
this work; inspect dirty files before syncing.

Start with [MAPPING.md](MAPPING.md), then:

- [joint_mapping.py](joint_mapping.py): no hardware interfaces; degrees in/out.
- [Original-unit draft](mapping/profiles/viscous_01.json).
- [Waterproof-unit draft](mapping/profiles/viscous_waterproof_01.json).
- [Saved historical inputs and source hashes](mapping/inputs.json).
- [Mapping test report](inspection/joint_mapping_validation.json).
- [Leader URDF](model/star_arm_102_hd.urdf) and [model notes](README.md).
- Follower URDF: `../model/viscous_arm.urdf`, relative to this directory.

Both profiles are deliberately `draft`. Reference anchors of zero are assumptions,
and signs/window centers/endpoints need physical verification. Loading a draft
requires explicit `allow_unverified=True` for offline evaluation. Do not relabel a
draft as verified simply because its software tests pass.

No live profiles, actuator zeros, hardware-control dependencies/services or robot
records were changed. The mapper is **not integrated into Lab teleoperation or recording**.
Unrelated cinematic work remains dirty in the Linux repository; preserve it.

## Established results and important frame issues

- The leader was rebuilt from pinned manufacturer HD STEP geometry. All six arm
  axes were corroborated against mating CAD features. J5 is revolute; its shaft
  belongs to link4 and its moving servo housing to link5. The rings mimic with
  opposite rotation. The model passed 101-pose FK checks. This does not establish
  physical encoder-to-CAD signs or zero offsets.
- The selected joint mapper passed **16 software tests**, covering unit gain,
  reference anchors, limits, raw-angle wrap, both units and gripper endpoints.
- Raw leader unwrapping must use an independent leader-frame window. It must
  not be derived from follower limits/scale/offsets.
- Waterproof J1 was re-zeroed at the ROV-mounted neutral on 11 October 2026:
  `old_angle = new_angle - 94.48°`. The draft shifts its captured limits to
  approximately **−32.37285584° to +282.00908635°**, retaining neutral at zero.
  The last-inspected Linux live profile still had old-frame limits of roughly
  −126.85° to +187.53°. Reinspect current files rather than assuming no later edits.
- Updating only those limits in the current driver could unwrap a leader reading
  of −107.1° to +252.9° and request the opposite stop. The new offline mapper's
  regression catches this. Do not deploy just the limit change or add +94.48°
  to every command.
- The original unit's historical gripper-closed motor reading is **−33°**.
  That offset does not apply to the waterproof unit. Both gripper drafts need
  fresh endpoint checks; interpolation is not a calibrated jaw-gap model.
- The original unit's folded leader wrist-flex was historically around **+10.6°**,
  while its old scaled mapping pressed the follower against its folded stop.
  Treat this as an unresolved reference-anchor issue, not proof of a current zero.
- J2/J3 backed-off limits exclude exact nominal zero. The mapper reports these
  clipped requests explicitly. Mechanical joint limits do not establish ROV hull
  clearance for combinations of joints.

## First physical session

1. Identify which follower is connected and which Mac calibration/profile is
   selected. Discover current serial ports; old names below are hints only.
2. Check whether an existing hardware session owns the ports. Have Gavin support
   and park the arm before stopping/releasing any active session. Do not open a
   second bus connection while another controller owns it.
3. Prepare and verify a **read-only paired encoder logger running on the Mac**.
   Such a paired logger has not been built in this work. The normal
   `MakerFollower.connect()` path enables torque and is not a read-only reader.
   Inspect the actual installed driver/API before choosing a low-level read path.
4. With the base secured, arm supported and follower torque off, manually match
   recognizable corresponding joint postures. Capture both arms' readings as
   anchors without resetting motor zeros. Compare angles relative to parent
   links, not gripper world position. Use an angle gauge/clear alignment where
   possible; do not assume the mechanical stop defines model zero.
5. Check one joint at a time with small comfortable movements (~5–10°), in both
   directions where clearance permits. Manually position the follower through
   the same angular change. Record signs and corresponding encoder values, plus
   a second pose to cross-check offsets/unit gain. Stay away from stops.
6. Check each gripper's comfortable closed/open references separately. Check
   leader wrap behavior across the intended working range without a powered
   full-travel sweep.
7. Save dated raw readings, unit identity, ports, calibration hashes, pose
   descriptions and fitted anchors/signs. Update that unit's draft and replay the
   observations offline. Preserve uncertainty rather than inventing missing data.
8. After the references are established, integrate the mapper in **both** Lab
   teleoperation and recording and test small, bounded powered moves with Gavin
   supporting/observing the arm. Preserve startup, stop, limit and thermal controls.

The arms have no brakes. Do not reset commissioned follower zeros, invoke
`lerobot-calibrate` on a follower, kill an active controller, or unplug it as a
routine stop. These are concrete project constraints, not a request to redo
commissioning.

## Related software and Mac details

Linux sibling repositories:

- `/home/gavin/Documents/makermodslab-viscous`: active Lab fork; see
  `docs/viscous-arm.md`, `makermodslab/arms/viscous.py`,
  `configs/viscous/viscous_waterproof_01/` and `pyproject.toml`.
- `/home/gavin/Documents/lerobot`: source checkout. **Its revision differs from
  Lab's installed pinned dependency.** Inspect the installed code; do not replace
  the entire Lab dependency with this older checkout just to patch one mapping.
  Read its `AGENTS.md` before working there, and `examples/viscous/EXPERIENCE.md`
  before planning autonomous motion.
- `/home/gavin/Documents/viscous-arm-calib/waterproof-2026-10-08`: saved capture
  evidence. The J1 re-zero record is also copied into `mapping/inputs.json` here.

Mac paths/ports must be rediscovered. Historical documentation used
`~/Documents/makermodslab-viscous`, Mac Lab state
`~/.makermods/makermodslab-viscous`, follower calibration `viscous_01` with leader
`star_viscous`, and waterproof pair `viscous_waterproof_01` with
`star_viscous_waterproof_01`. Historical ports were
`/dev/cu.usbmodem2050389538461` (CANable) and `/dev/cu.wchusbserial10` (Star).
The waterproof deployment was historically Linux-only; **Mac profile parity has
not been verified**. Back up and compare actual Mac files before any changes.

`makermods-viscous` on the Mac starts its local Lab at `http://127.0.0.1:8000`.
`makermods-viscous --remote` starts Linux Lab at `http://100.64.91.4:8000`; that
remote mode expects the hardware on Linux and is not suitable for Mac-attached
USB by itself. The Linux model viewer may be started separately; no stale viewer
process or port should be assumed alive.

## Recheck offline work

From the Linux model repository, or use an equivalent Python environment on Mac:

```sh
.venv/bin/python -m unittest leader.test_joint_mapping -v
.venv/bin/python -m leader.validate_joint_mapping
```

The joint mapper/tests use Python's standard library. Do not copy the Linux
virtual environment to macOS. CAD/model rebuilds have separate pinned dependencies;
they are not required merely to read encoders or evaluate the joint mapping.
