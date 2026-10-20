# ROV propeller-cleaning website film

The current film is in `rov_ship/`: **the actual BlueROV2 Heavy with its two mounted Viscous arms**, beside a large ship with **two 4.6 m propellers and one centreline rudder**. The native vehicle mesh hierarchy, original arm mounts, grippers, cameras, buoyancy blocks, and eight thrusters come directly from the existing ROV Blender rig. There is no presentation platform or replacement vehicle body.

The 20-second silent film establishes the full stern, moves to the ROV gripping a blade edge, shows overlapping passes of a flat spinning abrasive pad, then pulls back from the cleaned bronze. The propellers stay stationary. Everything uses metre scale; the sub-metre ROV cleans a small portion of one large blade.

## Current deliverables

- `rov_ship/viscous_propeller_cleaning.mp4`: 1280 × 720, 24 fps, 20 seconds.
- `rov_ship/viscous_propeller_cleaning_web.mp4`: 960 × 540 H.264 website copy.
- `rov_ship/viscous_rov_ship_cleaning.blend`: editable Blender 4.5 scene with embedded geometry and procedural materials.
- `rov_ship/poster.jpg`: actual ROV working beside the blade.
- `rov_ship/detail.png`: cleaning close-up.
- `rov_ship/render_report.json`: dimensions, duration, size, IK residual, and full decode checks.
- `rov_ship/asset_review.json`: exact source mesh and parent matching, original mount transforms, joint limits, and pad-target tracking checks.
- `index.html`: current playback and download page.
- `ship_stern_asset.blend` and `ship_stern_aft_preview.png`: separate ship asset and twin-propeller / single-rudder review.
- [Ship references](ship_references.md): online images and source notes used by the ship asset subagent.

The complete ROV source is `../viscous-rov-model/blender/viscous_rov.blend`, built from the user's full ROV CAD. Motion uses that project's URDF and `bimanual_common.py` inverse kinematics. All poses solve within the existing joint bounds. The saved film remains a kinematic animation, not validated underwater dynamics or contact-force simulation. The cleaned trail is driven by the pad's sampled footprint.

## Reproduce

Keep the sibling `viscous-rov-model` project available to rebuild. The delivered `.blend` itself embeds all geometry and does not need the sibling project for playback or editing.

```sh
uv venv --python 3.11 .blender-venv
uv pip install --python .blender-venv/bin/python bpy==4.5.3 scipy
.blender-venv/bin/python scripts/plan_rov_ship_cinematic.py
.blender-venv/bin/python scripts/build_rov_ship_cinematic.py --preview 250
.blender-venv/bin/python scripts/validate_rov_ship_cinematic.py
.blender-venv/bin/python scripts/render_cleaning_cinematic.py --native-rov
.venv/bin/python scripts/encode_cleaning_cinematic.py --native-rov
```

Rendering uses NVIDIA OptiX where available. The renderer resumes by skipping existing numbered frames. Move/remove old `rov_ship/frames/` before rendering changes. To render selected review frames:

```sh
.blender-venv/bin/python scripts/render_cleaning_cinematic.py --native-rov --frames 1 140 250 480
```

Run the existing local viewer server and visit `http://127.0.0.1:8766/cinematic/`, or open `index.html` directly.

## Still-image gallery

Open [the 34-view gallery](gallery/) to browse working angles, tool close-ups, ship context, and standalone ROV views. The gallery supports category filters, browser-local favourites, copying selected view numbers, keyboard navigation, full-size JPG/PNG downloads, a contact sheet, and a ZIP of all JPGs. Most images are 1920 × 1080; views 31, 32, and 34 provide portrait, square, and wide-banner alternatives.

From the MacBook on the same Tailscale network, visit `http://100.64.91.4:8766/cinematic/gallery/`. The video page also links to the gallery.

Camera definitions and framing live in `scripts/render_rov_gallery.py`; `gallery/shots.json` records the numbered output manifest. Rendering opens the existing film scene in memory and does not save over it or alter the video. Standalone views hide the ship while retaining the original vehicle geometry and underwater lighting.

```sh
.blender-venv/bin/python scripts/render_rov_gallery.py
.venv/bin/python scripts/package_rov_gallery.py
```

For draft previews, add `--draft`; to rerender selected final views, use `--ids 01 11 --force`, then rerun packaging. Packaging verifies every image's dimensions and writes `gallery/gallery_report.json`.

## Website video use

The film contains no baked-in text or sound. It is an edited sequence rather than a seamless loop. Keep a visible pause control and respect reduced-motion settings when used as a repeating background. Use `object-fit: contain` if both ship propellers must remain visible on narrow screens.

```html
<video muted loop playsinline preload="metadata" poster="rov_ship/poster.jpg"
       aria-label="BlueROV2 Heavy cleaning a ship propeller" id="cleaning-film">
  <source src="rov_ship/viscous_propeller_cleaning_web.mp4" type="video/mp4">
</video>
<button type="button" id="film-toggle">Play animation</button>
<script>
  const film = document.querySelector('#cleaning-film');
  const toggle = document.querySelector('#film-toggle');
  const reduced = matchMedia('(prefers-reduced-motion: reduce)');
  toggle.onclick = () => film.paused ? film.play().catch(() => {}) : film.pause();
  film.onplay = () => toggle.textContent = 'Pause animation';
  film.onpause = () => toggle.textContent = 'Play animation';
  if (!reduced.matches) film.play().catch(() => {});
  reduced.addEventListener('change', () => { if (reduced.matches) film.pause(); });
</script>
```

## Single-arm launch films: silver and black

The separate [Viscous launch page](launch/index.html) presents two matching 32-second product films of the single CAD arm: brushed aluminium silver and anodized black. Each finish includes an editable Blender scene, 1920 × 1080 / 24 fps master, 720p web copy, hero poster and nine shot-review stills. A side-by-side finish comparison and synchronized playback controls are included. The shared URDF rig has six arm controls and a mimic-driven gripper; the edit progresses from an abstract macro to component studies and a moving hero reveal.

See [launch production/rebuild notes](launch/README.md). New `scripts/*launch_cinematic.py` scripts take `--finish silver|black` and use the existing Blender/OptiX/resumable-frame conventions. The launch assets live entirely under `launch/`; the existing `rov_ship/` deliverables remain separate.
