# klippy/extras/cnc/limits.py

class SoftLimitError(Exception):
    pass


class SoftLimits:
    def __init__(self, limits):
        """
        limits: dict like {
            "X": (min, max),
            "Y": (min, max),
            "Z": (min, max),
        }
        """
        self.limits = limits

    def check_point(self, point):
        """
        point: (x, y, z)
        """
        for axis, value in zip(("X", "Y", "Z"), point):
            if axis not in self.limits:
                continue

            min_v, max_v = self.limits[axis]
            if value < min_v or value > max_v:
                raise SoftLimitError(
                    f"{axis}-axis soft limit exceeded: {value:.3f} "
                    f"(limits {min_v} to {max_v})"
                )

    def check_primitive(self, primitive):
        """
        Check both start and end of a motion primitive.
        """
        self.check_point(primitive.start)
        self.check_point(primitive.end)
