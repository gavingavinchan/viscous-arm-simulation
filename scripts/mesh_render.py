"""Shared orthographic depth-buffered mesh renderer for inspection figures."""
import numpy as np
from PIL import Image
from matplotlib.colors import to_rgb


def render(meshes, color, elevation, azimuth, *, reference, center, basis, background='#f6f8fa'):
    """Rasterize the real triangles with an orthographic camera and depth buffer.

    A depth buffer hides the compound's internal faces, avoiding the misleading
    triangle-order artifacts of painter-style 3D plotting. No geometry is edited.
    """
    vertices = (reference.vertices - center) @ basis.T * 1000
    elev, azim = np.radians([elevation, azimuth])
    direction = np.array([np.cos(elev) * np.cos(azim), np.cos(elev) * np.sin(azim), np.sin(elev)])
    right = np.array([-np.sin(azim), np.cos(azim), 0.])
    up = np.cross(direction, right)
    camera = np.array([right, up, direction])
    projected = vertices @ camera.T
    midpoint = (projected.max(axis=0) + projected.min(axis=0)) / 2
    # Render at 2x resolution, then downsample for smooth silhouettes.
    width, height = 1800, 1200
    scale = min((width - 180) / np.ptp(projected[:, 0]), (height - 140) / np.ptp(projected[:, 1]))
    pixels = np.empty((height, width, 3), dtype=np.uint8)
    pixels[:] = np.array(to_rgb(background)) * 255
    depth = np.full((height, width), -np.inf)
    light = direction * .9 + up * .7 - right * .4
    light /= np.linalg.norm(light)
    base = np.array(to_rgb(color))
    for mesh in meshes:
        triangles = (((mesh.vertices - center) @ basis.T) * 1000)[mesh.faces]
        for triangle in triangles:
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            length = np.linalg.norm(normal)
            if length < 1e-12:
                continue
            normal /= length
            if normal @ direction <= 0:
                continue
            q = triangle @ camera.T
            x = (q[:, 0] - midpoint[0]) * scale + width / 2
            y = -(q[:, 1] - midpoint[1]) * scale + height / 2
            xmin, xmax = max(0, int(np.floor(x.min()))), min(width - 1, int(np.ceil(x.max())))
            ymin, ymax = max(0, int(np.floor(y.min()))), min(height - 1, int(np.ceil(y.max())))
            if xmin > xmax or ymin > ymax:
                continue
            denominator = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
            if abs(denominator) < 1e-12:
                continue
            yy, xx = np.mgrid[ymin:ymax + 1, xmin:xmax + 1] + .5
            a = ((y[1] - y[2]) * (xx - x[2]) + (x[2] - x[1]) * (yy - y[2])) / denominator
            b = ((y[2] - y[0]) * (xx - x[2]) + (x[0] - x[2]) * (yy - y[2])) / denominator
            c = 1 - a - b
            z = a * q[0, 2] + b * q[1, 2] + c * q[2, 2]
            local_depth = depth[ymin:ymax + 1, xmin:xmax + 1]
            mask = (a >= -1e-9) & (b >= -1e-9) & (c >= -1e-9) & (z > local_depth)
            local_depth[mask] = z[mask]
            shade = .4 + .6 * max(0., normal @ light)
            pixels[ymin:ymax + 1, xmin:xmax + 1][mask] = np.clip(base * shade * 255, 0, 255)
    return Image.fromarray(pixels).resize((width // 2, height // 2), Image.Resampling.LANCZOS)
