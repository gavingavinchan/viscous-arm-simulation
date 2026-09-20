"""Check convex compounds, sampled CAD coverage, asset parity and collision cost."""
from pathlib import Path
from collections import defaultdict
import json
import tempfile
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh
import shapely
from collision_geometry import plane_distances, surface_samples, SETTINGS
from outline_collision import broad_frame, filled_outline

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'model'
manifest = json.loads((MODEL / 'model_manifest.json').read_text())
urdf = ET.parse(MODEL / 'viscous_arm.urdf')
mjcf = ET.parse(MODEL / 'viscous_arm.xml')
report = {
    'scope': 'Convexity, asset parity, sampled surface coverage and local CPU timings; not a continuous geometric error bound or hardware validation',
    'coverage_tolerance_m': 2e-6,
    'parts': [],
    'links': {},
    'filled_outlines': [],
    'solid_assemblies': [],
}
old_hulls = {}
total_old = total_new = total_source = 0.
old_faces = new_faces = count = 0
for link, data in manifest['links'].items():
    grouped = defaultdict(list)
    for item in data['collisions']:
        grouped[item.get('assembly',item['part'])].append(item)
    urdf_entries = urdf.findall(f"./link[@name='{link}']/collision")
    assert len({e.get('name') for e in urdf_entries}) == len(urdf_entries)
    assert sorted(e.find('geometry/mesh').get('filename') for e in urdf_entries) == sorted(c['mesh'] for c in data['collisions'])
    link_old = link_new = link_source = 0.
    for part, items in grouped.items():
        assembly = SETTINGS.get('assemblies', {}).get(part)
        ids = assembly['parts'] if assembly else [part]
        sources = [trimesh.load_mesh(MODEL / f'meshes/part_{i:03d}.stl') for i in ids]
        source = trimesh.util.concatenate(sources) if assembly else sources[0]
        prior = [s.convex_hull for s in sources]
        old = source.convex_hull
        old_volume = sum(h.volume for h in prior)
        for i, h in zip(ids, prior):old_hulls[i] = (link, h)
        hulls = [trimesh.load_mesh(MODEL / item['mesh']) for item in items]
        cap = assembly['max_pieces'] if assembly else SETTINGS.get('outline_parts', {}).get(str(part), {}).get('max_pieces', SETTINGS['parts'].get(str(part), 1))
        assert len(hulls) <= cap
        assert all(h.is_volume and h.is_convex for h in hulls), part
        if assembly:
            assert assembly['link'] == link
            assert all(item.get('parts') == ids and item.get('assembly') == part for item in items)
            center, basis = source.bounds.mean(axis=0), np.asarray(assembly['profile_basis'])
            expected = filled_outline(source, center, basis)
            actual = shapely.union_all([shapely.MultiPoint(((h.vertices-center)@basis.T)[:,:2]).convex_hull for h in hulls],grid_size=1e-9)
            # Profile cleanup is conservative; coverage of CAD stays at the
            # STL tolerance, while extra material may extend by 0.1 mm.
            assert expected.difference(actual.buffer(report['coverage_tolerance_m'])).area < 1e-12
            assert actual.difference(expected.buffer(.0001)).area < 1e-12
            # Probe the whole span through the plate spacing, including the
            # former central cavity. Reference = filled profile intersected
            # with the full 3D convex envelope, not just the original surfaces.
            xyz = (source.vertices-center)@basis.T
            axes = [np.linspace(xyz[:,i].min()+1e-5,xyz[:,i].max()-1e-5,n) for i,n in enumerate([31,21,13])]
            grid = np.array(np.meshgrid(*axes,indexing='ij')).reshape(3,-1).T
            inside_profile = shapely.contains_xy(expected,grid[:,0],grid[:,1])
            points = grid@basis+center
            inside_envelope = plane_distances(points,[old])[:,0] < -2e-6
            probes = points[inside_profile & inside_envelope]
            assert len(probes)>100
            assert plane_distances(probes,hulls).min(axis=1).max() < report['coverage_tolerance_m']
            formerly_empty = int(np.count_nonzero(plane_distances(probes,prior).min(axis=1)>report['coverage_tolerance_m']))
            assert formerly_empty>100
            report['solid_assemblies'].append(dict(name=part,source_parts=ids,passed=True,convex_pieces=len(hulls),solid_interior_probes=len(probes),formerly_empty_probes_now_filled=formerly_empty,conservative_profile_tolerance_m=.0001))
        if str(part) in SETTINGS.get('single_hull_parts', {}):
            assert len(hulls) == 1, part
            assert np.max(np.abs(hulls[0].bounds - old.bounds)) < report['coverage_tolerance_m'], part
            assert plane_distances(hulls[0].vertices, [old]).max() < report['coverage_tolerance_m'], part
            assert abs(hulls[0].volume / old.volume - 1) < .001, part
        if str(part) in SETTINGS.get('outline_parts', {}):
            center, basis = broad_frame(source)
            expected = filled_outline(source, center, basis)
            actual = shapely.union_all([
                shapely.MultiPoint(((h.vertices - center) @ basis.T)[:, :2]).convex_hull
                for h in hulls
            ], grid_size=1e-9)
            tolerance = report['coverage_tolerance_m']
            # Both directions matter: forbid filling outward concavities as
            # well as missing material or leaving interior holes in the shape.
            missing = expected.difference(actual.buffer(tolerance)).area
            excess = actual.difference(expected.buffer(tolerance)).area
            assert missing < 1e-12 and excess < 1e-12, (part, missing, excess)
            report['filled_outlines'].append(dict(
                part=part, passed=True, convex_pieces=len(hulls),
                perimeter_tolerance_m=tolerance,
                missing_area_beyond_tolerance_m2=missing,
                excess_area_beyond_tolerance_m2=excess,
                projected_symmetric_difference_mm2=expected.symmetric_difference(actual).area * 1e6))
        for item in items:
            name = Path(item['mesh']).stem
            geom = mjcf.find(f".//body[@name='{link}']/geom[@name='{name}']")
            assert geom is not None and geom.get('mesh') == name
            asset = mjcf.find(f"./asset/mesh[@name='{name}']")
            assert asset is not None and asset.get('file') == Path(item['mesh']).name
        distances = plane_distances(surface_samples(source), hulls).min(axis=1)
        assert distances.max() < report['coverage_tolerance_m'], (part, distances.max())
        volume = sum(h.volume for h in hulls)
        # Sum of piece volumes counts any overlaps; it is not exact union volume.
        source_volume = abs(source.volume)
        report['parts'].append(dict(part=part, link=link, hulls=len(hulls),
                                    faces=sum(len(h.faces) for h in hulls),
                                    source_volume_cm3=source_volume * 1e6,
                                    old_hull_volume_cm3=old_volume * 1e6,
                                    new_summed_hull_volume_cm3=volume * 1e6,
                                    max_sample_plane_gap_m=max(0., float(distances.max()))))
        link_old += old_volume
        link_new += volume
        link_source += source_volume
        old_faces += sum(len(h.faces) for h in prior)
        new_faces += sum(len(h.faces) for h in hulls)
        count += len(hulls)
    report['links'][link] = dict(old_hull_volume_cm3=link_old * 1e6,
                               new_summed_hull_volume_cm3=link_new * 1e6,
                               summed_volume_reduction_percent=100 * (1 - link_new / link_old))
    total_old += link_old
    total_new += link_new
    total_source += link_source
report['summary'] = dict(old_hulls=len(old_hulls), new_hulls=count,
                         old_faces=old_faces, new_faces=new_faces,
                         old_hull_volume_cm3=total_old * 1e6,
                         new_summed_hull_volume_cm3=total_new * 1e6,
                         summed_volume_change_percent=100 * (total_new / total_old - 1),
                         note='Assembly envelopes intentionally add volume in former plate-pair cavities; volumes sum convex pieces and may count overlaps.')
report['single_hull_envelopes'] = {'passed': True, 'parts': sorted(map(int, SETTINGS.get('single_hull_parts', {}))),
                                  'checks': 'Exactly one convex piece, CAD convex envelope bounds and support planes within export tolerance, volume within 0.1%'}


def benchmark(model):
    data = mujoco.MjData(model)
    rng = np.random.default_rng(98741)
    poses = rng.uniform(model.jnt_range[:, 0], model.jnt_range[:, 1], (1000, 8))
    poses[:, 7] = poses[:, 6]
    timings = []
    for _ in range(4):
        start = time.perf_counter()
        for pose in poses:
            data.qpos[:] = pose
            mujoco.mj_forward(model, data)
        timings.append(time.perf_counter() - start)
    # Exclude the warm-up round. This measures collision/FK calls, not a controller.
    return dict(median_1000_forward_seconds=float(np.median(timings[1:])),
                rounds_seconds=timings)


current = mujoco.MjModel.from_xml_path(str(MODEL / 'viscous_arm.xml'))
# Reconstruct the prior single-hull policy with otherwise identical settings.
with tempfile.TemporaryDirectory(prefix='viscous-collision-baseline-') as tmp:
    tmp = Path(tmp)
    baseline = ET.parse(MODEL / 'viscous_arm.xml')
    baseline.find('compiler').set('meshdir', str(tmp))
    asset = baseline.find('asset')
    for mesh in list(asset):
        if mesh.get('name', '').startswith('collision_'):
            asset.remove(mesh)
        else:
            mesh.set('file', str(MODEL / 'meshes' / mesh.get('file')))
    for body in baseline.findall('.//body'):
        for geom in list(body.findall('geom')):
            if geom.get('name', '').startswith('collision_'):
                body.remove(geom)
    for part, (link, hull) in old_hulls.items():
        name = f'collision_{part:03d}'
        hull.export(tmp / (name + '.stl'))
        ET.SubElement(asset, 'mesh', name=name, file=name + '.stl')
        body = baseline.find(f".//body[@name='{link}']")
        ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name,
                      contype='1', conaffinity='1', group='3', density='0')
    baseline.write(tmp / 'baseline.xml')
    previous = mujoco.MjModel.from_xml_path(str(tmp / 'baseline.xml'))
    report['benchmark'] = {'old': benchmark(previous), 'new': benchmark(current)}
report['benchmark']['forward_time_ratio'] = report['benchmark']['new']['median_1000_forward_seconds'] / report['benchmark']['old']['median_1000_forward_seconds']
report['passed'] = True
(ROOT / 'inspection/collision_validation.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({k: v for k, v in report.items() if k != 'parts'}, indent=2))
