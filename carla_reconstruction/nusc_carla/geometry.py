"""Small dependency-free geometry helpers used by the preparation stage."""

import hashlib
import math
import random


def stable_seed(value):
    """Return a process-independent 32-bit seed for *value*."""
    digest = hashlib.sha1(str(value).encode("utf-8")).digest()
    return int.from_bytes(digest[:4], byteorder="little", signed=False)


def stable_random(value):
    return random.Random(stable_seed(value))


def point_in_polygon(point, polygon):
    """Ray-casting test. Boundary points are treated as inside."""
    x, y = point
    if len(polygon) < 3:
        return False
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if _point_segment_distance_sq(point, (xi, yi), (xj, yj)) < 1.0e-12:
            return True
        crosses = ((yi > y) != (yj > y))
        if crosses:
            x_cross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x <= x_cross:
                inside = not inside
        j = i
    return inside


def _point_segment_distance_sq(point, a, b):
    px, py = point
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= 1.0e-15:
        return (px - ax) ** 2 + (py - ay) ** 2
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = max(0.0, min(1.0, t))
    qx, qy = ax + t * dx, ay + t * dy
    return (px - qx) ** 2 + (py - qy) ** 2


def distance_to_samples(point, samples):
    """Distance to the closest (x, y, ...) sample, or infinity."""
    if not samples:
        return float("inf")
    x, y = point
    return math.sqrt(min((x - p[0]) ** 2 + (y - p[1]) ** 2 for p in samples))


def nearest_sample(point, samples):
    """Return the closest sample and its distance."""
    if not samples:
        return None, float("inf")
    x, y = point
    best = min(samples, key=lambda p: (x - p[0]) ** 2 + (y - p[1]) ** 2)
    return best, math.hypot(x - best[0], y - best[1])


def point_in_oriented_box(point, box, margin=0.0):
    """Test a point against a manifest building box."""
    dx = point[0] - box["x"]
    dy = point[1] - box["y"]
    c = math.cos(box["yaw_rad"])
    s = math.sin(box["yaw_rad"])
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    return (abs(local_x) <= box["half_x"] + margin and
            abs(local_y) <= box["half_y"] + margin)


def sample_polygon_grid(polygon, spacing, seed):
    """Generate deterministic jittered grid samples inside a polygon."""
    if len(polygon) < 3 or spacing <= 0:
        return []
    rng = stable_random(seed)
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    x0 = math.floor(min(xs) / spacing) * spacing
    y0 = math.floor(min(ys) / spacing) * spacing
    x1, y1 = max(xs), max(ys)
    result = []
    x = x0 + spacing * 0.5
    while x <= x1:
        y = y0 + spacing * 0.5
        while y <= y1:
            jitter = spacing * 0.22
            point = (x + rng.uniform(-jitter, jitter),
                     y + rng.uniform(-jitter, jitter))
            if point_in_polygon(point, polygon):
                result.append(point)
            y += spacing
        x += spacing
    return result
