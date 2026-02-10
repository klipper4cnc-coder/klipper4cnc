import math


def segment_linear(start, end, feedrate, max_segment_time):
    """
    Segment a linear move into time-bounded segments.

    This function breaks a single linear motion into smaller segments
    such that no segment exceeds the specified maximum execution time.
    This is important for smooth motion and for respecting backend
    motion planner constraints.

    :param start: (x, y, z) start position in machine space
    :param end: (x, y, z) end position in machine space
    :param feedrate: Feedrate in units per minute
    :param max_segment_time: Maximum allowed time per segment (seconds)
    :return: List of (seg_start, seg_end) tuples
    """
    # No motion required
    if start == end:
        return []

    # Convert feedrate from units/min to units/sec
    feed_ups = feedrate / 60.0 if feedrate > 0 else 0.0

    # Total distance of the move
    length = math.dist(start, end)

    # If feedrate or length is invalid, execute as a single segment
    if feed_ups <= 0 or length <= 0:
        return [(start, end)]

    # Total time required for the move
    move_time = length / feed_ups

    # Determine number of segments required to respect max_segment_time
    segments = max(1, math.ceil(move_time / max_segment_time))

    segs = []
    prev = start

    # Generate linearly interpolated segments
    for i in range(1, segments + 1):
        # Normalized interpolation parameter (0..1)
        t = i / segments

        # Interpolate XYZ coordinates
        next_pt = (
            start[0] + (end[0] - start[0]) * t,
            start[1] + (end[1] - start[1]) * t,
            start[2] + (end[2] - start[2]) * t,
        )

        # Store segment
        segs.append((prev, next_pt))
        prev = next_pt

    return segs
