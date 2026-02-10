# klippy/extras/cnc/limits.py


class SoftLimitError(Exception):
    """
    Exception raised when a soft axis limit is exceeded.

    Soft limits are enforced in software to prevent the interpreter
    or controller from generating motion outside configured bounds.
    """
    pass


class SoftLimits:
    """
    Software-enforced axis travel limits.

    This class provides lightweight boundary checking for CNC motion.
    It does NOT stop motion mid-move; it is intended to be used
    before motion is executed.
    """

    def __init__(self, limits):
        """
        Initialize soft limits.

        :param limits: Dictionary mapping axis letters to (min, max) tuples.
                       Example:
                       {
                           "X": (0.0, 300.0),
                           "Y": (0.0, 300.0),
                           "Z": (-100.0, 0.0),
                       }
        """
        self.limits = limits

    def check_point(self, point):
        """
        Check a single XYZ point against configured soft limits.

        :param point: (x, y, z) tuple in machine coordinates
        :raises SoftLimitError: if any axis exceeds its limits
        """
        for axis, value in zip(("X", "Y", "Z"), point):
            # Ignore axes without configured limits
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
        Check both endpoints of a motion primitive.

        This ensures the entire motion stays within bounds,
        assuming straight-line interpolation.
        """
        self.check_point(primitive.start)
        self.check_point(primitive.end)
