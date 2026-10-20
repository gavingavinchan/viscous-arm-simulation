# URDF posture comparison and Mac continuation — 18 October 2026

The Mac now has a checkout at `/Users/gavin/Documents/viscous-arm-model`, cloned
from the Linux repository. The original cinematic work was not copied or changed.

Open the offline comparison with `python3 viewer/server.py --port 8776`, then
`http://127.0.0.1:8776/viewer/compare.html`. Both original URDFs load progressively
with the same camera orientation and scale. Sliders change the leader's CAD
angles. The follower uses signed unit gain and fixed CAD offsets. Turning off
the alignment checkbox exposes the mismatch from using signed angles without
the CAD datum corrections. No browser control opens hardware or changes Lab.

## Geometric result

`posture_geometry.py` reads the emitted URDFs. With aligned CAD bases, it aligns
the J2→J3 upper-arm direction, J3→J4 forearm direction and corresponding joint
axis lines. For follower CAD angle `f` and leader CAD angle `l`, in degrees:

| Joint | CAD conversion |
| --- | --- |
| J1 | `f = l` |
| J2 | `f = -l - 0.9421631282` |
| J3 | `f = -l - 9.5657026392` |
| J4 | `f = -l + 19.7597668145` |
| J5 | `f = l` |
| J6 | `f = l` |

Across 102 model poses, the largest link-direction/axis discrepancy is less
than `1e-7` degrees. This is geometric consistency, not physical accuracy. Model
roll zero is retained; the leader handle and follower jaw tool frames differ.
Hand world positions are not equated, and link lengths are not rescaled.
Collision clearance and physical limits are not established by this comparison.

These **are not encoder conversion coefficients**. In particular, model J5's
same-axis sign does not by itself disprove the live negative encoder sign.
Encoder-to-CAD signs and datums are still needed for both devices.

## Current Mac configuration

`mapping/mac_waterproof_20261018.json` preserves the actual Mac profile, source
hash and the user's confirmed aquarium reference. The six live arm magnitudes
are `1.446099, 0.981723, 0.912081, 1.040071, 1.246420, 0.945691`. These do not
implement the selected 1:1 joint-angle control. The current Mac gripper-closed
limit is `-0.03437746770784939` degrees; the Linux offline draft's older
`-2.9220847551671985` endpoint must not overwrite it.

The user confirmed this aquarium neutral in current follower coordinates:
J1 `+4.48`, J2 `-78.34577688899543`, J3 `+0.5384937739450696`,
J4 `+75.90521517413885`, J5/J6 `0`, gripper closed. J1 is original-frame
`-90`, correcting the ROV `-94.48` reference. The reference has not been applied
to motor zero, live leader mapping, startup or return behavior. J3 remains below
the current `+2.8647889756541165` startup limit.

## Physical correspondence and capture transport

Gavin clarified that he can match physical postures approximately, not perfectly.
Use several comfortable, roughly corresponding poses, small individual joint
changes and repeat captures. Estimate offsets and report their spread. A rough
pair is useful for direction checks and initial offsets; it is not an exact
calibration. CAD can supply the geometric relationship, but it cannot reveal an
unknown physical encoder zero without a physical reference or measurement.

`capture_pair.py` prepares a sensor-only path: direct Star `read_angle`, and
native RobStride READ_PARAMETER requests for `0x3016` / `0x7019`. It refuses an
active Lab session or owned USB port and never configures or zeroes the leader.
The follower path only sends those native read commands, not a driver handshake,
enable, disable, MIT motion, fault clear or mode-changing command. Native
parameter values are retained raw and are not automatically fitted as calibrated
MIT angles.

The Mac transport check at 17:04:15 used the current CANable and Star ports.
The leader's first angle answered, but follower motor 1 did not answer native
`0x3016`. The capture is incomplete and cannot be used as a matched reference.
Raw TX/RX and the error are retained in `mapping/captures/`. Do not change motor
mode to make this tool work. A supported MIT feedback capture path still needs
to be checked on hardware before taking full paired physical references. The
logger now has an explicit `--protocol mit-feedback --supported-torque-off`
path, based on the existing RS00 tester's non-enabling, zero-gain packet. Using
it requires supported, torque-off hardware and is described as a zero-effort
command, not a passive native parameter read. The default native mode never
falls back automatically. Do not run either while another controller owns the
bus. This MIT path has passed packet/decoder tests and awaits the user's
confirmation of the supported reference posture before its first hardware run.

The Lab server has not been restarted. No active profiles, limits, motor zeros
or mapping code were modified during this comparison work.

## Validation

`python3 -m unittest leader.test_joint_mapping leader.test_posture_geometry
leader.test_capture_pair -v` passes 23 tests. Browser rendering loaded 220 leader
pieces and 60 follower pieces, with independent Three.js FK showing zero
displayed corresponding link/axis error for nonzero poses.

## Later physical capture and requested warm-up behavior

The explicit MIT feedback path subsequently read all seven follower motors and
all seven leader encoders on the Mac. One capture stopped on an unreliable
leader wrist-yaw read; bounded fresh-read retries allowed the next capture to
complete. See `mapping/captures/viscous_waterproof_01-20261018-170716.json` and
its comparison sidecar. The completed sample was stable. The roughly matched
pose differed from the old live command by approximately 3.5–9.2 degrees across
the six arm joints. This includes manual positioning error and was not used to
fit or overwrite reference offsets. No torque-enable command was sent by this
capture tool.

Gavin then clarified that both arms encounter different starting constraints:
the aquarium bed and the leader trolley. Exact manual starting alignment is
unnecessary. He wants a gradual convergence toward the fixed matching posture,
followed by smooth absolute tracking after alignment. A relative/clutched mode
that preserves starting mismatch does not satisfy this; the proposed relative
implementation was removed before any deployment.

The Mac Lab now has an opt-in **1:1 posture trial**, restricted to the waterproof
unit. It removes travel-ratio scaling for the six arm joints, retains the
commissioned direction signs/fixed encoder offsets and gripper mapping, and
uses independent leader unwrap windows. Physical zero offsets are provisional;
the geometric CAD offsets are not blindly used as encoder offsets. No claim of
physically exact model matching has been made.

Its non-expiring warm-up limits each arm joint target to one degree from its
measured position and retains the existing 15-degree/second outgoing target
cap. It does not re-anchor to an obstructed starting pose or release a stalled
joint to a large correction after a timeout. Being within three degrees for
half a second switches to tracking; large mismatch re-enters warm-up. True
communication and thermal errors keep their existing stop behavior. This is
position convergence, not obstacle/contact detection.

J1's documented re-zero correction is applied to in-memory trial limits only.
Motor zeros and profile files remain unchanged. Normal startup position checks
still apply. Recording is unchanged and does not yet use the trial mapping.
The GUI explicitly displays warm-up/tracking and permits the original mode by
unchecking the trial box. The corrected Mac backend was loaded through the
LaunchAgent after normally stopping an idle, manually launched old process.
No physical teleoperation test was started by the agent. Powered motion feedback
from Gavin is the next step. The Linux Lab deployment was not changed.

The Mac backend change is in
`/Users/gavin/Documents/makermodslab-viscous/makermodslab/joint_posture_mapping.py`,
with the session option wired through the API and local Teleop launcher. Ten
new mapping/warm-up tests pass, as do the existing focused teleop/session tests,
OpenAPI contract checks, TypeScript checks, selected frontend tests, lint and
the production build. These tests do not qualify physical encoder alignment.

The first powered trial ended at 17:20:22 after the newly added leader freshness
gate rejected wrist-yaw replies. The new strict gate caused the shutdown; it
was unrelated to posture convergence or follower temperature (21 C). Gavin
explicitly requested preserving the leader behavior that had worked before.
That gate, extra leader polling and retries have been removed from the posture
controller. It now calls the original `get_action()` exactly once and reuses
the raw angles already collected by the existing driver, including its normal
cached-read behavior. The 1:1 mapper and warm-up remain. A regression test covers
the unchanged cached-read path and forbids separate monitor polling. The Mac
server was reloaded only after confirming it was idle. Further powered testing
remains operator-controlled.

Gavin's next trial stayed active but could not lift off the aquarium bed.
The added one-degree target-versus-measured-position cap was starving
proportional lifting torque. That cap has now been removed. Soft startup uses
the existing persistent bus reference: it is seeded at the measured engage
pose and advances toward the leader target at 15 degrees/second, independently
of whether the follower initially lags. Normal gains apply in both warm-up and
tracking; no motor gains or power settings were raised. Warm-up status still
ends on alignment and never redefines the reference pose. Thirty-two focused
mapping/speed-limit tests pass, including a stalled-follower regression proving
that command error can build beyond one degree while respecting the target
rate. The idle Mac service was reloaded; the revised lift behavior still needs
Gavin's physical trial.

Warm-up status was subsequently corrected: it previously used the unclipped
request and required no active limits, so an unreachable endpoint could keep
the label on indefinitely. Completion now checks the reachable clamped targets
against the same three-degree / half-second criterion, while keeping joint
limits visible as a separate status. Large lag after the first alignment is
labelled catching up. The GUI and a two-second log now identify the worst joint,
its tracking error, and limited joints. These are status/completion changes;
normal gains and target-rate behavior are unchanged. Thirteen mapping tests,
TypeScript, catalog checks and the production build passed.

Gavin then requested a fixed five-second warm-up, since the non-neutrally
buoyant follower may retain position error under gravity. The 17:33 trial had
actually reached alignment, then stayed in catching-up with roughly 4–5 degrees
of lag on J2/J3/J4; that does not establish gravity as the cause. Warm-up now
lasts five seconds from the first mapped action and then stays in tracking,
without re-entering warm-up or catching-up. The largest joint error and active
limits remain visible. This changes only startup status: absolute targets,
normal gains, the persistent 15-degree/second target limit, leader polling,
and existing failure/Stop handling are unchanged. Thirty-four mapping and
speed-limit tests pass, including stalled motion through the five-second
boundary; TypeScript, catalog checks, lint and the production build passed.
The idle Mac Lab service was reloaded. No powered test was started by the agent.
