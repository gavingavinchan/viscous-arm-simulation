"""Preserve the broad-view CAD perimeter, fill holes, and bridge depth grooves.

Intersect the full 3D convex envelope with a prism of the filled 2D silhouette.
Partition only that exterior outline into convex regions for simulation; no
interior hole or groove is reconstructed. Coordinates are link-local metres.
"""
import heapq
import numpy as np
from scipy.spatial import ConvexHull
import shapely
from shapely import Polygon
import trimesh


def broad_frame(mesh):
    center = mesh.vertices.mean(axis=0)
    vertices = mesh.vertices - center
    _, _, basis = np.linalg.svd(vertices, full_matrices=False)
    basis = basis[np.argsort(np.ptp(vertices @ basis.T, axis=0))[::-1]]
    return center, basis


def projected_shape(mesh, center, basis):
    points = (mesh.vertices - center) @ basis.T
    triangles = shapely.polygons(points[mesh.faces, :2])
    triangles = triangles[shapely.area(triangles) > 1e-18]
    return shapely.union_all(triangles, grid_size=1e-9)


def filled_outline(mesh, center, basis):
    shape = projected_shape(mesh, center, basis)
    if shape.geom_type != 'Polygon':
        raise ValueError(f'Expected one connected silhouette, got {shape.geom_type}')
    # Drop interior rings. Remove only effectively collinear boundary points;
    # 10 nm is far below the subsequent 1 micrometre STL export precision.
    return Polygon(shape.exterior).simplify(1e-8, preserve_topology=True)


def convex_regions(outline):
    regions = dict(enumerate(shapely.constrained_delaunay_triangles(outline).geoms))
    queue = []

    def candidate(i, j):
        shared = regions[i].boundary.intersection(regions[j].boundary).length
        if shared > 1e-9:
            heapq.heappush(queue, (-shared, i, j))

    for i in regions:
        for j in range(i):
            candidate(j, i)
    next_id = len(regions)
    while queue:
        _, i, j = heapq.heappop(queue)
        if i not in regions or j not in regions:
            continue
        merged = shapely.union(regions[i], regions[j])
        convex = merged.convex_hull
        if convex.area - merged.area > 1e-15:
            continue
        del regions[i], regions[j]
        regions[next_id] = convex
        for other in regions:
            if other != next_id:
                candidate(other, next_id)
        next_id += 1
    result = list(regions.values())
    assert shapely.union_all(result).symmetric_difference(outline).area < 1e-12
    return result


def clip_to_region(hull, region):
    equations = ConvexHull(np.asarray(region.exterior.coords)[:-1]).equations
    for a, b, offset in equations:
        normal = np.array([a, b, 0.])
        distances = hull.vertices @ normal + offset
        inside = distances <= 1e-12
        if inside.all():
            continue
        edges = hull.edges_unique
        crossings = edges[inside[edges[:, 0]] != inside[edges[:, 1]]]
        d0, d1 = distances[crossings[:, 0]], distances[crossings[:, 1]]
        start = hull.vertices[crossings[:, 0]]
        end = hull.vertices[crossings[:, 1]]
        cuts = start + (d0 / (d0 - d1))[:, None] * (end - start)
        hull = trimesh.convex.convex_hull(np.vstack((hull.vertices[inside], cuts)))
    return hull


def outline_hulls(mesh, boundary_cleanup_m=1e-8, frame=None, outward_margin_m=0):
    center, basis = broad_frame(mesh) if frame is None else frame
    outline = filled_outline(mesh, center, basis).simplify(boundary_cleanup_m, preserve_topology=True)
    if outward_margin_m:
        # Assemblies combine independently tessellated circular parts. Cover
        # sub-tolerance notches conservatively rather than multiplying pieces.
        outline = outline.buffer(outward_margin_m, join_style='mitre')
    envelope = trimesh.convex.convex_hull((mesh.vertices - center) @ basis.T)
    pieces = []
    for region in convex_regions(outline):
        clipped = clip_to_region(envelope.copy(), region)
        # Re-hull in the link frame to preserve outward winding under reflected
        # PCA bases as well as ordinary rotations.
        pieces.append(trimesh.convex.convex_hull(clipped.vertices @ basis + center))
    pieces.sort(key=lambda h: tuple(h.centroid))
    return pieces
