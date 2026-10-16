# Leader-to-Viscous joint-posture mapping

Gavin selected **corresponding joint angles and arm posture**, with physical
checks deferred until the next session. The selected mapper is
[joint_mapping.py](joint_mapping.py). The earlier Cartesian/IK mapper remains
an offline experiment and is not the selected control law.

## Mapping prepared offline

For each of the six arm joints:

```text
leader_angle = unwrap(raw_encoder, leader_window_center)
requested_follower = follower_reference + direction * (leader_angle - leader_reference)
applied_follower = clamp(requested_follower, this_unit_limits)
```

`direction` is exactly +1 or −1. A 10-degree leader change therefore requests a
10-degree follower change in the corresponding direction. Follower travel is
not divided by leader travel. Different link lengths need no angle scaling for
this control mode; the two hand positions will naturally differ.

The mapper returns both the requested angle and the limited target, with an
explicit `limited` flag and reason. Joint limits can prevent matching posture
at the ends of travel. The original software backoff also excludes the assumed
exact zero of J2/J3: those zero requests are visibly reported as limited rather
than silently moving the reference anchor.

Raw-angle unwrapping uses the **leader's own coordinate frame**, independently
of follower limits, offsets and direction. Wrapped readings such as −270.5 and
+89.5 degrees produce the same target. An exact ambiguous ±180-degree window
boundary is rejected. Each physical leader joint's entire travel must fit
inside its configured window; current windows still require physical checking.

The gripper is separate from the six posture joints. Its closed/open control
fraction is interpolated onto each follower's own closed/open motor endpoints.
The original unit retains its historical −33-degree closed reference. The
waterproof unit uses its backed-off closed limit. This is a control-endpoint
mapping, not a measured jaw-gap or nonlinear transmission model.

## Per-unit drafts and provenance

- [Original non-waterproof unit](mapping/profiles/viscous_01.json)
- [Waterproof ROV-mounted unit](mapping/profiles/viscous_waterproof_01.json)
- [Historical inputs and source hashes](mapping/inputs.json)

Both drafts have `verification.status = "draft"`. Their zero-to-zero arm
anchors are **assumptions**, not newly measured corresponding poses. Direction
signs are retained from the commissioned mapping. Historical raw travel and
motor endpoints are preserved with their sources; the leader's pan origin has
changed since its range capture, so its draft unwrap window is centered on
neutral instead. Replace the reference anchors, directions or window centers
where physical checks show a discrepancy. No commissioned motor zeros need to
be rewritten.

Default loading refuses these unverified profiles. Offline evaluation requires
an explicit opt-in, and the result remains labelled `physically_verified: false`:

```python
from leader.joint_mapping import JointMapper

mapper = JointMapper.from_file(
    "leader/mapping/profiles/viscous_waterproof_01.json",
    allow_unverified=True,  # Offline draft evaluation only.
)
result = mapper.map({
    "shoulder_pan": 0., "shoulder_lift": 30., "elbow_flex": -60.,
    "wrist_flex": 0., "wrist_yaw": 0., "wrist_roll": 0., "gripper": 20.,
})
```

The standalone CLI accepts the same seven-value JSON object in a file:

```sh
.venv/bin/python -m leader.joint_mapping \
  --profile leader/mapping/profiles/viscous_waterproof_01.json \
  --leader-degrees /path/to/raw-leader-angles.json --offline
```

## Waterproof J1 correction in the draft

The saved commissioning record says J1 was re-zeroed at its ROV-mounted neutral
on 11 October 2026 while the leader was also at neutral. It gives:

```text
old angle = new angle - 94.48 degrees
```

The existing Lab profile still has −126.85285584 to +187.52908635 degrees in the
old frame. Re-expressing its captured soft limits in the new frame gives
**−32.37285584 to +282.00908635 degrees**. These corrected limits are included
in the waterproof offline draft. Its matched-neutral assumption stays zero;
+94.48 degrees is not added to leader commands.

Changing just the range in the current driver would introduce a wrap error:
that driver derives the raw leader window from the follower range and scale.
With the new J1 limits, a negative raw reading near −107 degrees could unwrap
to +253 degrees and request the opposite positive stop. The offline mapper
uses the independent leader window and correctly produces the negative limited
target. This case is explicitly regression-tested.

The authoritative record and old profile are preserved inside
`mapping/inputs.json`, with hashes and original paths. The original
non-waterproof unit keeps its own frame, limits and gripper reference.

## Rebuild and tests

From the repository root:

```sh
.venv/bin/python -m leader.prepare_joint_profiles
.venv/bin/python -m unittest leader.test_joint_mapping -v
.venv/bin/python -m leader.validate_joint_mapping
```

The suite checks signed unit gain, arbitrary reference anchors, both unit
profiles, monotonic full-travel sweeps, full-turn equivalence, limit reporting,
gripper endpoints, malformed/nonfinite data, ambiguous boundaries, draft-loading
refusal and the J1 re-zero regression. The generated
[validation report](inspection/joint_mapping_validation.json) records input,
profile and implementation hashes plus numeric examples. These are software
checks under the draft anchor assumptions, not physical accuracy measurements.

## Next physical session

1. Check corresponding reference poses on the leader and each follower; replace
   the draft anchors, especially the original unit's folded wrist-flex anchor
   and the waterproof unit's ROV-mounted J1 frame.
2. Confirm the direction of a small movement at each joint and verify that its
   full raw travel stays inside the configured unwrap window.
3. Check each gripper's closed/open endpoints separately.
4. Review limited targets against the measured per-unit stops and ROV clearance.
5. Integrate the checked mapping consistently into Lab teleoperation and recording,
   retaining the existing startup, stop and thermal controls; then test small
   supported movements with the operator.

No live profiles, actuator zeros or running control sessions were changed.
Both URDF projects still mark encoder-to-CAD signs/zeros as unverified. The live
mapping should only be replaced after the corresponding physical references
are checked.

## Cartesian experiment

`retarget.PoseMapper` and `test_retarget.py` remain available for separate offline
research. They match relative tool poses through bounded IK and have their own
geometry tests. They are not used by `JointMapper` or the selected posture
profiles.
