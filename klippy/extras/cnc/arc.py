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

    This function takes a circular arc definition and approximates it
    as a list of linear points suitable for Klipper-style motion planning.

    Supported cases:
    - Normal arcs where start != end
    - Full-circle arcs where start == end

    The returned points do NOT include the start point, only the
    intermediate points up to and including the end of the arc.

    :param start: (x, y) start position
    :param end: (x, y) end position
    :param center: (x, y) arc center
    :param clockwise: True for G2 (CW), False for G3 (CCW)
    :param tolerance: Maximum allowed chord error for segmentation
    :return: List of (x, y) points approximating the arc
    """

    # Unpack coordinates for readability
    sx, sy = start
    ex, ey = end
    cx, cy = center

    # Compute radius from center to start and end
    # (Both should be equal for a valid arc)
    rs = math.hypot(sx - cx, sy - cy)
    re = math.hypot(ex - cx, ey - cy)

    if rs == 0 or re == 0:
        # Degenerate case: arc radius is zero
        raise ValueError("Arc radius is zero")

    # Compute angular positions of start and end points
    start_ang = math.atan2(sy - cy, sx - cx)
    end_ang = math.atan2(ey - cy, ex - cx)

    # Detect full-circle arcs (start and end coincide)
    # CNC allows this to indicate a complete revolution
    is_full_circle = (
        abs(sx - ex) < 1e-9 and
        abs(sy - ey) < 1e-9
    )

    if is_full_circle:
        # Full circle sweep direction depends only on CW/CCW
        sweep = -2 * math.pi if clockwise else 2 * math.pi
    else:
        # Partial arc: compute signed sweep angle
        sweep = end_ang - start_ang

        # Adjust sweep to match requested direction
        if clockwise and sweep > 0:
            sweep -= 2 * math.pi
        elif not clockwise and sweep < 0:
            sweep += 2 * math.pi

    # Total arc length = radius * angular sweep
    arc_length = abs(sweep) * rs

    if arc_length == 0:
        # No movement required
        return []

    # Determine segmentation based on chordal tolerance
    #
    # max_seg_angle is the maximum angular step such that
    # the deviation between the arc and straight chord
    # is within the specified tolerance.
    max_seg_angle = 2 * math.acos(max(0.0, 1 - tolerance / rs))

    # Number of segments needed to respect tolerance
    segments = max(1, int(abs(sweep) / max_seg_angle))

    points = []
    for i in range(1, segments + 1):
        # Interpolate parameter along the arc (0..1)
        t = i / segments

        # Compute angle for this segment point
        ang = start_ang + sweep * t

        # Convert polar back to Cartesian coordinates
        x = cx + rs * math.cos(ang)
        y = cy + rs * math.sin(ang)

        points.append((x, y))

    return points


def compute_arc_center_from_r(start, end, r, clockwise):
    """
    Compute the arc center from start/end points and an R parameter.

    This implements the CNC G-code R-format arc definition, where
    the arc is defined by:
      - start point
      - end point
      - radius R (sign indicates short/long arc)

    There are always two possible circle centers for a given
    chord and radius; the correct one is selected based on:
    - motion direction (CW / CCW)
    - sign of R (short vs long arc)

    :param start: (x, y) start position
    :param end: (x, y) end position
    :param r: Radius (negative means "long way around")
    :param clockwise: True for G2, False for G3
    :return: (x, y) center position
    """

    x0, y0 = start
    x1, y1 = end

    # Vector from start to end
    dx = x1 - x0
    dy = y1 - y0
    chord_len = math.hypot(dx, dy)

    r_abs = abs(r)

    if chord_len == 0:
        # R-format arcs cannot have identical start/end
        raise ValueError("Arc start and end points are identical")

    if chord_len > 2 * r_abs:
        # Geometry impossible: radius too small
        raise ValueError("Arc radius too small for given endpoints")

    # Midpoint of the chord between start and end
    mx = (x0 + x1) / 2
    my = (y0 + y1) / 2

    # Distance from chord midpoint to circle center
    h = math.sqrt(r_abs**2 - (chord_len / 2)**2)

    # Unit vector perpendicular to the chord
    nx = -dy / chord_len
    ny = dx / chord_len

    # Two possible circle centers
    cx1 = mx + nx * h
    cy1 = my + ny * h
    cx2 = mx - nx * h
    cy2 = my - ny * h

    # Determine arc direction using cross product sign
    def is_clockwise(cx, cy):
        """
        Determine whether the arc from start to end around
        this center is clockwise.

        Uses the sign of the 2D cross product.
        """
        cross = (x0 - cx) * (y1 - cy) - (y0 - cy) * (x1 - cx)
        return cross < 0

    # Select center that matches requested direction
    if clockwise:
        center = (cx1, cy1) if is_clockwise(cx1, cy1) else (cx2, cy2)
    else:
        center = (cx1, cy1) if not is_clockwise(cx1, cy1) else (cx2, cy2)

    # Negative R means "long way around" → flip center choice
    if r < 0:
        center = (cx2, cy2) if center == (cx1, cy1) else (cx1, cy1)

    return center
