"""Show the unchanged CAD assemblies beside their solid collision envelopes."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mesh_render import render

ROOT = Path(__file__).resolve().parents[1]
model = json.loads((ROOT / 'model/model_manifest.json').read_text())
groups = model['collision_geometry']['assemblies']
fig, axes = plt.subplots(2, 2, figsize=(12, 9), facecolor='#f6f8fa')
elevation = np.degrees(np.arcsin(1 / np.sqrt(3)))
labels = {'upper_arm_shell': 'J2–J3 · two plates',
          'forearm_shell': 'J3–J4 · four plates + six spacers'}
for row, (name, spec) in enumerate(groups.items()):
    cad = [trimesh.load_mesh(ROOT / f'model/meshes/part_{p:03d}.stl') for p in spec['parts']]
    source = trimesh.util.concatenate(cad)
    hulls = [trimesh.load_mesh(ROOT / 'model' / c['mesh'])
             for c in model['links'][spec['link']]['collisions'] if c.get('assembly') == name]
    center, basis = source.bounds.mean(axis=0), np.asarray(spec['profile_basis'])
    for col, meshes in enumerate([cad, hulls]):
        ax = axes[row, col]
        ax.imshow(render(meshes, '#91a9bb' if col == 0 else '#eaa267', elevation, -45,
                         reference=source, center=center, basis=basis))
        ax.set_axis_off()
        ax.set_title(labels[name] + (' · CAD' if col == 0 else ' · solid collision envelope'),
                     fontsize=12, color='#263846', pad=12)
fig.suptitle('Arm collision envelopes · internal cavities filled', fontsize=18, y=.98)
fig.text(.5, .018, 'CAD appearance and mass properties unchanged. Each span has one continuous collision envelope.',
         ha='center', fontsize=10, color='#364451')
fig.subplots_adjust(left=.025, right=.975, top=.91, bottom=.06, wspace=.02, hspace=.16)
output = ROOT / 'inspection/arm_envelopes.png'
fig.savefig(output, dpi=170, facecolor='#f6f8fa')
plt.close(fig)
print(output)
