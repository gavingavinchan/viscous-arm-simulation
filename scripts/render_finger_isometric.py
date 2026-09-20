"""Depth-buffered orthographic isometric views of CAD and collision geometry."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from outline_collision import broad_frame
from mesh_render import render

ROOT = Path(__file__).resolve().parents[1]
manifest = json.loads((ROOT / 'model/model_manifest.json').read_text())
source = trimesh.load_mesh(ROOT / 'model/meshes/part_042.stl')
hulls = [trimesh.load_mesh(ROOT / 'model' / c['mesh'])
         for d in manifest['links'].values() for c in d['collisions'] if c['part'] == 42]
center, basis = broad_frame(source)
if np.linalg.det(basis) < 0:
    basis[2] *= -1
background = '#f6f8fa'

fig, axes = plt.subplots(2, 2, figsize=(12, 9), facecolor=background)
elevation = np.degrees(np.arcsin(1 / np.sqrt(3)))
for row, (elev, azim) in enumerate([(elevation, -45), (-elevation, 135)]):
    for col, meshes in enumerate([[source], hulls]):
        ax = axes[row, col]
        ax.imshow(render(meshes, '#91a9bb' if col == 0 else '#eaa267', elev, azim, reference=source, center=center, basis=basis))
        ax.set_axis_off()
        ax.set_title(['CAD finger', 'Collision envelope'][col] + (' · opposite side' if row else ''),
                     fontsize=14, color='#263846', pad=14)
fig.suptitle('Gripper finger · isometric comparison', fontsize=19, y=.98)
fig.text(.5, .018, 'Same scale and view direction in each pair. Broad outline retained; deep groove and holes filled.',
         ha='center', fontsize=11, color='#364451')
fig.subplots_adjust(left=.025, right=.975, top=.91, bottom=.06, wspace=.02, hspace=.16)
output = ROOT / 'inspection/finger_isometric.png'
fig.savefig(output, dpi=170, facecolor=background)
plt.close(fig)
print(output)
