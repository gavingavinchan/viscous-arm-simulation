"""Modest convex compounds for CAD parts; all coordinates remain link-local metres."""
from importlib.metadata import version
from pathlib import Path
import hashlib
import json
import os

# Avoid oversubscribing the workstation during the offline mesh build.
os.environ.setdefault('OMP_NUM_THREADS', '4')
import coacd
import numpy as np
from scipy.spatial import ConvexHull
import trimesh
from outline_collision import outline_hulls

ROOT = Path(__file__).resolve().parents[1]
SETTINGS = json.loads((ROOT / 'collision_settings.json').read_text())
assert not (SETTINGS['parts'].keys() & SETTINGS.get('single_hull_parts', {}).keys()), 'A part cannot request both one hull and decomposition'
assert not (SETTINGS.get('outline_parts', {}).keys() & (SETTINGS['parts'].keys() | SETTINGS.get('single_hull_parts', {}).keys())), 'Outline parts must have exactly one collision policy'


def surface_samples(mesh):
    """Deterministic vertices, triangle centres and edge midpoints (not a proof)."""
    return np.vstack((mesh.vertices, mesh.triangles_center,
                      mesh.vertices[mesh.edges_unique].mean(axis=1)))


def plane_distances(points, hulls):
    """Max signed face-plane distance per hull; <= 0 means inside that hull."""
    result = []
    for hull in hulls:
        eq = ConvexHull(hull.vertices).equations
        # Bound temporary memory for high-resolution source meshes.
        result.append(np.concatenate([
            np.max(chunk @ eq[:, :3].T + eq[:, 3], axis=1)
            for chunk in np.array_split(points, max(1, len(points) // 2048))
        ]))
    return np.asarray(result).T


def collision_hulls(mesh, part):
    if str(part) in SETTINGS.get('outline_parts', {}):
        hulls = outline_hulls(mesh, SETTINGS['outline_parts'][str(part)].get('boundary_cleanup_m', 1e-8))
        assert len(hulls) <= SETTINGS['outline_parts'][str(part)]['max_pieces']
        return hulls
    cap = SETTINGS['parts'].get(str(part))
    if cap is None:
        # The full CAD convex envelope is the tightest possible convex wrap.
        # Do not decimate fingers: retain tip and gripping-face support planes.
        return [mesh.convex_hull]
    params = dict(threshold=SETTINGS['threshold_m'], real_metric=True,
                  max_convex_hull=cap, preprocess_mode='auto',
                  preprocess_resolution=100, resolution=2000,
                  mcts_iterations=100, seed=SETTINGS['seed'],
                  decimate=True,
                  max_ch_vertex=SETTINGS['max_vertices_before_surface_repair'])
    digest = hashlib.sha256(mesh.vertices.tobytes() + mesh.faces.tobytes()
                            + json.dumps(params, sort_keys=True).encode()
                            + version('coacd').encode() + b'surface-repair-v1').hexdigest()
    cache_dir = ROOT / 'inspection' / '.collision_cache'
    cache_dir.mkdir(exist_ok=True)
    cache = cache_dir / (digest + '.npz')
    if cache.exists():
        with np.load(cache) as data:
            return [trimesh.Trimesh(data[f'v{i}'], data[f'f{i}'], process=False)
                    for i in range(int(data['count']))]
    coacd.set_log_level('error')
    pieces = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), **params)
    hulls = [trimesh.Trimesh(v, f).convex_hull for v, f in pieces]
    if not hulls:
        raise RuntimeError(f'No collision hulls for part {part}')
    # Decimation/remeshing can shrink contact surfaces. Restore omitted source
    # samples into their nearest hull, allowing extra vertices where needed.
    points = surface_samples(mesh)
    distances = plane_distances(points, hulls)
    nearest = distances.argmin(axis=1)
    outside = distances.min(axis=1) > 1e-8
    for i, hull in enumerate(hulls):
        extra = points[outside & (nearest == i)]
        if len(extra):
            hulls[i] = trimesh.convex.convex_hull(np.vstack((hull.vertices, extra)))
    hulls.sort(key=lambda h: tuple(h.centroid))
    assert len(hulls) <= cap
    assert all(h.is_volume and h.is_convex for h in hulls)
    assert plane_distances(points, hulls).min(axis=1).max() < 1e-7
    arrays = {'count': np.array(len(hulls))}
    for i, hull in enumerate(hulls):
        arrays.update({f'v{i}': hull.vertices, f'f{i}': hull.faces})
    np.savez_compressed(cache, **arrays)
    print(f'Collision part {part:03d}: {len(hulls)} hulls', flush=True)
    return hulls
