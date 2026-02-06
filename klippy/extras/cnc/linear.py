import math

def segment_linear(start, end, feedrate, max_segment_time):
    """
    Segment a linear move into time-bounded segments.

    start, end: (x, y, z) in machine space
    feedrate: units/min
    max_segment_time: seconds

    Returns: list of (seg_start, seg_end)
    """
    if start == end:
        return []

    # Convert feedrate to units/sec
    feed_ups = feedrate / 60.0 if feedrate > 0 else 0.0
    length = math.dist(start, end)

    if feed_ups <= 0 or length <= 0:
        return [(start, end)]

    move_time = length / feed_ups
    segments = max(1, math.ceil(move_time / max_segment_time))

    segs = []
    prev = start

    for i in range(1, segments + 1):
        t = i / segments
        next_pt = (
            start[0] + (end[0] - start[0]) * t,
            start[1] + (end[1] - start[1]) * t,
            start[2] + (end[2] - start[2]) * t,
        )
        segs.append((prev, next_pt))
        prev = next_pt

    return segs
