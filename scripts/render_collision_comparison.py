"""Static CAD / current collision comparison, including two finger views."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np
import trimesh
from outline_collision import broad_frame, filled_outline

ROOT = Path(__file__).resolve().parents[1]
manifest = json.loads((ROOT / 'model/model_manifest.json').read_text())
entries = [c for d in manifest['links'].values() for c in d['collisions']]
parts = [('upper_arm_shell', 'J2–J3 complete span', False), ('forearm_shell', 'J3–J4 complete span', False),
         (24, 'Gripper rail base', False), (42, 'Finger · broad view', False),
         (42, 'Finger · side view', True)]
fig, axes = plt.subplots(len(parts), 2, figsize=(10, 11), facecolor='#f6f8fa')
for row, (part, label, side_view) in enumerate(parts):
    assembly = manifest['collision_geometry'].get('assemblies', {}).get(part)
    if assembly:
        source = trimesh.util.concatenate([trimesh.load_mesh(ROOT / f'model/meshes/part_{i:03d}.stl') for i in assembly['parts']])
        center, basis = source.bounds.mean(axis=0), np.asarray(assembly['profile_basis'])
        chosen = [c for c in entries if c.get('assembly') == part]
    else:
        source = trimesh.load_mesh(ROOT / f'model/meshes/part_{part:03d}.stl')
        center, basis = broad_frame(source)
        chosen = [c for c in entries if c['part'] == part]
    hulls = [trimesh.load_mesh(ROOT / 'model' / c['mesh']) for c in chosen]
    if side_view:
        basis = basis[[0, 2, 1]]
    projected = (source.vertices - center) @ basis.T * 1000
    extent = np.ptp(projected[:, :2], axis=0)
    pad = extent.max() * .07
    for col, meshes in enumerate([[source], hulls]):
        ax = axes[row, col]
        triangles, colors = [], []
        for index, mesh in enumerate(meshes):
            vertices = (mesh.vertices - center) @ basis.T * 1000
            triangles.extend(vertices[mesh.faces])
            color = '#8296a6' if col == 0 else '#e8a065'
            colors.extend([color] * len(mesh.faces))
        triangles = np.asarray(triangles)
        order = np.argsort(triangles[:, :, 2].mean(axis=1))
        ax.add_collection(PolyCollection(triangles[order, :, :2], facecolors=np.asarray(colors)[order],
                                         edgecolors='none', linewidths=0))
        if col == 1 and not side_view and (assembly or str(part) in manifest['collision_geometry'].get('outline_parts', {})):
            perimeter = np.asarray(filled_outline(source, center, basis).exterior.coords) * 1000
            ax.plot(perimeter[:, 0], perimeter[:, 1], color='#364451', linewidth=.7)
        ax.set_xlim(projected[:, 0].min() - pad, projected[:, 0].max() + pad)
        ax.set_ylim(projected[:, 1].min() - pad, projected[:, 1].max() + pad)
        ax.set_aspect('equal')
        ax.set_axis_off()
        if row == 0:
            ax.set_title(['CAD geometry', 'Current collision envelope'][col], fontsize=12, pad=20)
        caption = '1 convex hull' if len(hulls) == 1 else f'Filled outline · {len(hulls)} convex pieces'
        ax.text(.5, -.12, label if col == 0 else caption,
                transform=ax.transAxes, ha='center', fontsize=10, color='#364451')
fig.suptitle('Viscous Arm collision refinement', fontsize=17, y=.99)
fig.text(.5, .012, 'Dark line: original CAD perimeter. Plate/rail/finger outlines preserved; internal holes and deep grooves filled.', ha='center', fontsize=9)
fig.tight_layout(rect=(0, .035, 1, .96), h_pad=2.5)
output = ROOT / 'inspection/collision_comparison.png'
fig.savefig(output, dpi=160, facecolor=fig.get_facecolor())
print(output)
