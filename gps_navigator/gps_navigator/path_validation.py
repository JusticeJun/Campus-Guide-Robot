"""Conservative raster footprint check for the published static route map."""

import json
import math

import numpy as np


def shared_footprint(parameters):
    """Require one base frame, polygon and padding for both costmaps and guard."""
    footprints = []
    for name in ('local_costmap', 'global_costmap'):
        config = parameters[name][name]['ros__parameters']
        polygon = json.loads(config['footprint'])
        padding = config.get('footprint_padding', 0.01)
        if (
            config['robot_base_frame'] != 'base_link'
            or len(polygon) < 3
            or any(
                len(point) != 2 or not all(math.isfinite(v) for v in point) for point in polygon
            )
            or not math.isfinite(padding)
            or padding < 0
        ):
            raise ValueError('Invalid base_link footprint or padding')
        footprints.append((polygon, padding))
    if footprints[0] != footprints[1]:
        raise ValueError('Local/global costmaps must use identical footprint and padding')
    return footprints[0]


def path_is_clear(poses, grid, footprint, padding):
    """Check interpolated bounding rectangles, including unknown and map edges."""
    if not poses or grid is None:
        return False
    info = grid.info
    cells = np.asarray(grid.data).reshape(info.height, info.width)
    # Bounding rectangle contains even a non-rectangular supplied footprint.
    fx, fy = zip(*footprint)
    cell_margin = info.resolution / math.sqrt(2)
    xmin, xmax = min(fx) - padding - cell_margin, max(fx) + padding + cell_margin
    ymin, ymax = min(fy) - padding - cell_margin, max(fy) + padding + cell_margin
    radius = max(math.hypot(x, y) for x in (xmin, xmax) for y in (ymin, ymax))
    previous = None
    for item in poses:
        if item.header.frame_id not in ('', 'map'):
            return False
        p, q = item.pose.position, item.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return False
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 0.01:
            return False
        current = (
            p.x,
            p.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        )
        previous = previous or current
        dx, dy = current[0] - previous[0], current[1] - previous[1]
        da = math.atan2(math.sin(current[2] - previous[2]), math.cos(current[2] - previous[2]))
        steps = max(
            1, math.ceil(math.hypot(dx, dy) / (info.resolution / 2)), math.ceil(abs(da) / 0.1)
        )
        for step in range(steps + 1):
            f = step / steps
            x, y, yaw = previous[0] + f * dx, previous[1] + f * dy, previous[2] + f * da
            ox, oy = info.origin.position.x, info.origin.position.y
            c0 = math.floor((x - radius - ox) / info.resolution)
            c1 = math.floor((x + radius - ox) / info.resolution)
            r0 = math.floor((y - radius - oy) / info.resolution)
            r1 = math.floor((y + radius - oy) / info.resolution)
            if c0 < 0 or r0 < 0 or c1 >= info.width or r1 >= info.height:
                return False
            rr, cc = np.mgrid[r0:r1 + 1, c0:c1 + 1]
            wx = ox + (cc + 0.5) * info.resolution - x
            wy = oy + (rr + 0.5) * info.resolution - y
            bx = math.cos(yaw) * wx + math.sin(yaw) * wy
            by = -math.sin(yaw) * wx + math.cos(yaw) * wy
            inside = (bx >= xmin) & (bx <= xmax) & (by >= ymin) & (by <= ymax)
            if np.any(inside & (cells[r0:r1 + 1, c0:c1 + 1] != 0)):
                return False
        previous = current
    return True
