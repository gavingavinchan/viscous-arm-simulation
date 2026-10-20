# Viscous — The art of articulation

Two 32-second, nine-shot launch films built from the single CAD-derived `model/viscous_arm.urdf`. Silver uses a bead-blasted/brushed aluminium shader; black uses fine satin anodized aluminium. Both share geometry, articulation, camera animation, typography, edit and soundtrack. Only structural aluminium shading and softbox power differ. Motors remain graphite satin, steel guides/bearings remain steel, and the PLA+ fingers remain charcoal polymer.

Open [the comparison and playback page](index.html). Each finish directory contains:

- `viscous_launch.blend`: editable Blender 4.5 scene, packed CAD geometry and title artwork.
- `viscous_launch.mp4`: 1920 × 1080, 24 fps H.264 master, 32 seconds, stereo AAC.
- `viscous_launch_web.mp4`: 1280 × 720 web copy, fast-start H.264.
- `poster.jpg`: hero frame.
- `review/01_surface.png` through `review/09_hero.png`: full-resolution shot studies, plus a contact sheet and additional gripper/hero checks.
- `motion.json`: shot list, URDF joint metadata and part/material ownership.
- `validation_report.json` and `render_report.json`: rig/framing validation and full video-decode results.
- `frames/frame_0001.png` through `frame_0768.png`: resumable source frames.

`finish_comparison.jpg` shows matching hero views side by side. `launch_soundtrack.wav` is an original synthesized electronic bed; it uses no licensed music or sampled Apple audio.

## Edit

Select **POSE | animate these URDF joint values** and edit/keyframe its custom properties. Values are radians for revolute joints and metres for the gripper. Each child link is parented through the exact URDF joint origin and uses an axis driver constrained to that joint's limits. `gripper_right_joint` reads `gripper_left_joint` via the URDF mimic multiplier/offset. Seven independent properties control the six arm joints and paired jaws.

Timeline camera markers define all nine cuts. Camera focus targets, lights and materials remain editable. Titles are packed transparent artwork in the animated Blender compositor; edit their text/font/placement in `scripts/launch_titles.py` and rebuild, or replace the packed image nodes. `FINISH | silver` / `FINISH | black` is the only structural material. Blender animation is kinematic, not a hardware program.

| Time | Shot |
|---|---|
| 0–4 s | Abstract macro glide down an arm span |
| 4–8 s | Exact tool-axis front view; close, open, close |
| 8–11 s | Base and mounting detail |
| 11–14 s | J2–J3 span |
| 14–17 s | J3–J4 span |
| 17–20 s | RS00 motor detail |
| 20–23 s | Wrist articulation |
| 23–26 s | Fingers and linear guide |
| 26–32 s | Full arm in motion, brand lockup |

## Rebuild and render

From the repository root, using the existing environments:

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 .blender-venv/bin/python scripts/build_launch_cinematic.py --finish silver
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 .blender-venv/bin/python scripts/build_launch_cinematic.py --finish black
.blender-venv/bin/python scripts/validate_launch_cinematic.py --finish silver
.blender-venv/bin/python scripts/validate_launch_cinematic.py --finish black
.blender-venv/bin/python scripts/render_launch_cinematic.py --finish silver --review
.blender-venv/bin/python scripts/render_launch_cinematic.py --finish black --review
.blender-venv/bin/python scripts/render_launch_cinematic.py --finish silver
.venv/bin/python scripts/encode_launch_cinematic.py --finish silver
.blender-venv/bin/python scripts/render_launch_cinematic.py --finish black
.venv/bin/python scripts/encode_launch_cinematic.py --finish black
```

The finish switch is `--finish silver|black`. Build silver first; building black uses the saved silver scene as the canonical shared rig/edit and changes only the finish and light power. If no silver scene exists, the same builder can create black directly. Rebuild both after any camera, geometry or motion change.

Cycles uses OptiX on the RTX 5070 Ti, 48-sample adaptive sampling, GPU denoising and numbered PNGs, matching the existing cinematic pipeline conventions. Existing frames are skipped. Scene/settings mismatch refuses unsafe resume; archive/remove that variant's frames before rendering a changed scene, or use `--force`. Selected previews go into `review/`, never into the master sequence. Example: `--frames 116 152 188 --width 1280 --samples 24`.

The encoder verifies every PNG, applies a half-second opening fade and a three-quarter-second closing fade, encodes both sizes, decodes each video in full, and regenerates the review page. Frame sequences and editable .blend contain the clean cut edit; the delivery fades and synthesized audio are applied by the encoder. No ROV/ship files are inputs or outputs.

## Assumptions and limits

- Gavin authorized unattended production; duration, shot timings, restrained labels and original ambient audio were chosen to complete the brief.
- This is a visual finish study. Exact alloy, anodizing dye and roughness are unmeasured. Fine procedural microtexture represents bead blasting/brushing, not measured surface scans.
- The existing URDF's simplified symmetric linear gripper is preserved. Hidden crank internals and absent finger loops are not invented. The camera placeholder remains the original CAD shape.
- The source model's axis signs/zero offsets and travel limits are provisional hardware values. The film validates URDF consistency and framing, not physical dynamics or collision-free hardware operation.
- The supplied reference was fetched, captions read, and sampled video frames visually inspected. See [reference study](reference/STYLE_NOTES.md). The original reference media is study material only and is not embedded in the outputs.
- All work remains uncommitted for Gavin's review.
