# Leader-to-Viscous mapping work

The CAD rebuild and the physical motor mapping are separate. The emitted URDFs
provide geometry, but both project manifests explicitly state that their encoder
signs and CAD-zero offsets are unverified. They cannot establish those values
without known physical poses.

## Implemented offline

`retarget.PoseMapper` maps relative leader hand motion to the Viscous tool pose,
using the actual tool frames in both URDFs and bounded inverse kinematics. This
replaces independent per-joint travel scaling in the offline calculation.

The corresponding hand reference poses align the two coordinate frames. At the
reference pose the follower has no target jump. Translation scale is explicit;
orientation is preserved. Unreachable targets and excessive joint steps raise
`PoseRejected`; a clipped IK result is never returned as an accepted solution.
The gripper remains a separate coordinate and is not changed by this six-axis
pose mapper.

```python
from leader.retarget import PoseMapper

# These must be MODEL coordinates in radians, after encoder calibration.
mapper = PoseMapper(leader_reference, follower_reference,
                    follower_bounds=calibrated_model_bounds)
solution = mapper.map(leader_model_angles, previous_follower_model_angles)
```

Run `.venv/bin/python -m unittest leader.test_retarget`. The tests include 100
small pose changes, reference-pose continuity, independent leader FK agreement,
translation scaling, and rejection of unreachable, discontinuous and out-of-range
solutions. These are offline geometry tests, not collision or hardware tests.
No realtime throughput is claimed. The solver is not wired into Lab or any motor
controller. All existing live profiles remain unchanged pending physical checks.

## Waterproof J1 frame mismatch found on 16 October 2026

The authoritative saved record is
`../makermodslab-viscous/configs/viscous/viscous_waterproof_01/evidence/j1-rezero-2026-10-11.json`
(relative to the repository root). It records:

- J1 was re-zeroed at the ROV-mounted neutral with the leader at neutral.
- Old-frame angle = new-frame angle − 94.48 degrees.
- The old soft limits were deliberately left in the profiles at that time.
- The backed-off mechanical range in the new frame is approximately
  −32.37 to +282.01 degrees.

The checked-in and active Linux profile still has −126.85285584 to +187.52908635
degrees. Re-expressing that same captured interval in the new frame gives
−32.37285584 to +282.00908635 degrees, or the inward-rounded leader output range
`[-32, 282]`.

**Do not change just that range in isolation.** The installed leader driver
centers raw-servo unwrapping on the follower range divided by its scale. After
the J1 range shift, its raw unwrap center would move to about +86 degrees. A
negative leader reading near −107 degrees could then unwrap to about +253 and
produce the opposite positive limit. Raw leader unwrapping must use its own
mechanical angle window independently of the follower clamp, and the new neutral
alignment must be retained. Merely adding +94.48 degrees to every leader command
would move the matched neutral and is not a valid correction either.

The original non-waterproof unit has a different calibration and its own
gripper-closed offset. The waterproof profile must not be copied over it.

## Remaining information for a live correction

1. Confirm whether the intended control matches Cartesian tool motion or the
   corresponding joint posture. Cartesian motion is the current offline mapper's
   assumption; they require different control laws for arms of different lengths.
2. Check encoder-to-URDF signs and offsets against known poses for the leader and
   each follower, including the ROV-mounted J1 zero. Preserve commissioned motor
   zeros; do not re-run follower calibration.
3. Establish the leader's raw angle unwrap windows in its current zero frame.
4. Verify gripper closed/open references separately for each follower. The old
   unit's −33-degree closed offset does not apply to the waterproof unit.
5. Use those calibrations to convert follower limits and IK output between model
   radians and motor degrees. Integrate the selected mapping into both Lab
   teleoperation and recording, then check supported small motions with the user.

The mathematical retargeter is ready to evaluate in model coordinates. A correct
live encoder mapping is not claimed until these physical references are checked.
