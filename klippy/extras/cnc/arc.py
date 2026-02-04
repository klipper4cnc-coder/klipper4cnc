# klippy/extras/cnc/arc.py
import math

# klippy/extras/cnc/arc.py

import math


def segment_arc(
    start,
    end,
    center,
    clockwise,
    tolerance,
):
    """
    Segment a circular arc into linear points.

    Supports:
    - normal arcs
    - full-circle arcs (start == end)
    """

    sx, sy = start
    ex, ey = end
    cx, cy = center

    # Radius
    rs = math.hypot(sx - cx, sy - cy)
    re = math.hypot(ex - cx, ey - cy)

    if rs == 0 or re == 0:
        raise ValueError("Arc radius is zero")

    # Angles
    start_ang = math.atan2(sy - cy, sx - cx)
    end_ang = math.atan2(ey - cy, ex - cx)

    # Detect full circle
    is_full_circle = (
        abs(sx - ex) < 1e-9 and
        abs(sy - ey) < 1e-9
    )

    if is_full_circle:
        sweep = -2 * math.pi if clockwise else 2 * math.pi
    else:
        sweep = end_ang - start_ang
        if clockwise and sweep > 0:
            sweep -= 2 * math.pi
        elif not clockwise and sweep < 0:
            sweep += 2 * math.pi

    arc_length = abs(sweep) * rs

    if arc_length == 0:
        return []

    # Segment count based on tolerance
    max_seg_angle = 2 * math.acos(max(0.0, 1 - tolerance / rs))
    segments = max(1, int(abs(sweep) / max_seg_angle))

    points = []
    for i in range(1, segments + 1):
        t = i / segments
        ang = start_ang + sweep * t
        x = cx + rs * math.cos(ang)
        y = cy + rs * math.sin(ang)
        points.append((x, y))

    return points


def compute_arc_center_from_r(start, end, r, clockwise):
    x0, y0 = start
    x1, y1 = end

    dx = x1 - x0
    dy = y1 - y0
    chord_len = math.hypot(dx, dy)

    r_abs = abs(r)

    if chord_len == 0:
        raise ValueError("Arc start and end points are identical")

    if chord_len > 2 * r_abs:
        raise ValueError("Arc radius too small for given endpoints")

    # Midpoint of chord
    mx = (x0 + x1) / 2
    my = (y0 + y1) / 2

    # Distance from midpoint to center
    h = math.sqrt(r_abs**2 - (chord_len / 2)**2)

    # Normalize perpendicular vector
    nx = -dy / chord_len
    ny = dx / chord_len

    # Two possible centers
    cx1 = mx + nx * h
    cy1 = my + ny * h
    cx2 = mx - nx * h
    cy2 = my - ny * h

    # Choose correct center based on direction and R sign
    def is_clockwise(cx, cy):
        cross = (x0 - cx) * (y1 - cy) - (y0 - cy) * (x1 - cx)
        return cross < 0

    if clockwise:
        center = (cx1, cy1) if is_clockwise(cx1, cy1) else (cx2, cy2)
    else:
        center = (cx1, cy1) if not is_clockwise(cx1, cy1) else (cx2, cy2)

    # R < 0 means "long way around"
    if r < 0:
        center = (cx2, cy2) if center == (cx1, cy1) else (cx1, cy1)

    return center
