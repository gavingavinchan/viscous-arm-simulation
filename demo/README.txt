Viscous Arm: 20 mm cube pick and place into an open cup

Rendered in MuJoCo on gavin-linux. 1280 x 960, 30 fps, 21.4 seconds.
The free 10 g cube is grasped through finger friction, lifted, carried, released,
and settles on the cup bottom. No object welding or teleporting.

Run from the repository root:
.venv/bin/python scripts/pick_place_video.py --render

Requires imageio-ffmpeg in addition to the model Python environment.
The base URDF and MuJoCo model are unchanged. The demo scene adds an open cup,
a 25 mm pickup platform, a 20 mm cube, and bounded position servos with gravity
feedforward. Arm force limits remain +/-5 Nm; gripper generalized force +/-10 N.

Demo assumptions: cube mass 10 g, cube friction .8. Robot material masses, motor
inertias, printed-part infill, joint calibration and gripper transmission remain
provisional. This is an offline simulation, not hardware qualification.

See pick_place_report.json for physical outcome checks and simulation_states.npz
for 10 Hz state snapshots. Scene: pick_place_scene.xml.
