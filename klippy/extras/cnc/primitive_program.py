# klippy/extras/cnc/primitive_program.py


class PrimitiveProgram:
    """
    Simple container for a sequence of motion primitives.

    This class represents a precomputed CNC "program" consisting
    of MotionPrimitive objects and provides sequential access
    for execution by a controller.
    """

    def __init__(self, primitives):
        # Ordered list of motion primitives
        self.primitives = primitives

        # Current execution index
        self.index = 0

    def has_next(self):
        """
        Check whether there are remaining primitives to execute.
        """
        return self.index < len(self.primitives)

    def next(self):
        """
        Return the next motion primitive and advance the program counter.
        """
        p = self.primitives[self.index]
        self.index += 1
        return p

    def current(self):
        """
        Return the current execution index.
        """
        return self.index

    def total(self):
        """
        Return the total number of primitives in the program.
        """
        return len(self.primitives)

    def percent_complete(self):
        """
        Return completion percentage based on primitive count.
        """
        if self.total() == 0:
            return 100.0
        return (self.index / self.total()) * 100.0

    def remaining_time_seconds(self):
        """
        Placeholder for estimated remaining execution time.

        Intended to be implemented later using executor feedback
        or feedrate-based estimation.
        """
        return 0
