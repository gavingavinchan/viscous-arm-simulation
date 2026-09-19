"""Interactive offline dry-air inspection. No robot I/O."""
from pathlib import Path
import time
import numpy as np
import mujoco
import mujoco.viewer
root=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(root/'model/viscous_arm.xml'));d=mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m,d,m.key('inspection_pose').id);target=d.qpos.copy();mujoco.mj_forward(m,d)
with mujoco.viewer.launch_passive(m,d) as viewer:
 viewer.opt.geomgroup[3]=0
 viewer.cam.lookat[:]=[.02,0,.22];viewer.cam.distance=.9;viewer.cam.azimuth=135;viewer.cam.elevation=-24
 while viewer.is_running():
  started=time.monotonic()
  for _ in range(10):
   d.ctrl[:6]=np.clip(d.qfrc_bias[:6]+18*(target[:6]-d.qpos[:6])-1.2*d.qvel[:6],-5,5)
   d.ctrl[6]=np.clip(d.qfrc_bias[6]+d.qfrc_bias[7]+300*(target[6]-d.qpos[6])-4*d.qvel[6],-10,10)
   mujoco.mj_step(m,d)
  viewer.sync();time.sleep(max(0,.01-(time.monotonic()-started)))
