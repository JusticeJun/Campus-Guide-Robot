"""Raster travel areas from stable graph geometry, not hard via-pose goals."""

import math

import numpy as np

from gps_navigator.navigation_profile import gps_to_map


def route_points(route, graph, alignment, entry_edge=None):
    """Return a selected edge chain; an entry edge supports a mid-edge start."""
    if not route or any(key not in graph for key in route):
        raise ValueError('empty route or unknown waypoint')
    if any(b not in graph[a]['neighbors'] for a, b in zip(route, route[1:])):
        raise ValueError('route includes an unreviewed edge')
    ids = list(route)
    if entry_edge:
        a, b = entry_edge
        if a not in graph or b not in graph or b not in graph[a]['neighbors'] or b != route[0]:
            raise ValueError('invalid directed entry edge')
        ids.insert(0, a)
    if len(ids) < 2:
        # At the destination: a point area is enough; do not fabricate an edge.
        ids *= 2
    return [gps_to_map(graph[key]['latitude'], graph[key]['longitude'], alignment) for key in ids]


def travel_grid(info, original, points, half_width):
    """Keep source occupancy inside selected edge tubes, block everything else."""
    if not math.isfinite(half_width) or half_width <= 0:
        raise ValueError('route_half_width_m must be positive')
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('route needs finite points')
    # Costmap2D's StaticLayer assumes an axis-aligned occupancy origin.
    q = info.origin.orientation
    if abs(q.x) + abs(q.y) + abs(q.z) > 1e-8 or abs(q.w - 1) > 1e-8:
        raise ValueError('Nav2 StaticLayer requires an axis-aligned map origin')
    rows, cols = np.indices((info.height, info.width))
    x = info.origin.position.x + (cols + 0.5) * info.resolution
    y = info.origin.position.y + (rows + 0.5) * info.resolution
    allowed = np.zeros(x.shape, dtype=bool)
    # Shrink half a cell diagonal: cells touching the boundary stay blocked.
    radius = max(0.0, half_width - info.resolution / math.sqrt(2))
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = (
            np.clip(((x - ax) * dx + (y - ay) * dy) / length2, 0, 1)
            if length2 > 0
            else np.zeros(x.shape)
        )
        allowed |= (x - ax - t * dx) ** 2 + (y - ay - t * dy) ** 2 <= radius**2
    source = np.asarray(original, dtype=np.int8).reshape(x.shape)
    return np.where(allowed, source, 100).astype(np.int8).ravel().tolist()
