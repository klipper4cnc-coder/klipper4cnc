# klippy/extras/cnc/program.py


class CNCProgram:
    """
    Container for a CNC program represented as motion primitives.

    This class manages:
    - Storage of motion primitives
    - Sequential execution state
    - Basic progress and time estimation
    """

    def __init__(self):
        # Ordered list of motion primitives
        self.primitives = []

        # Current execution index
        self.cursor = 0

    def load_primitives(self, primitives):
        """
        Append a list of motion primitives to the program.
        """
        self.primitives.extend(primitives)

    def has_next(self):
        """
        Check whether there are remaining primitives to execute.
        """
        return self.cursor < len(self.primitives)

    def next(self):
        """
        Return the next motion primitive and advance the cursor.
        """
        p = self.primitives[self.cursor]
        self.cursor += 1
        return p

    def reset(self):
        """
        Clear the program and reset execution state.
        """
        self.primitives.clear()
        self.cursor = 0

    # ---- progress helpers ----

    def total(self):
        """
        Return total number of primitives in the program.
        """
        return len(self.primitives)

    def current(self):
        """
        Return the current execution index.
        """
        return self.cursor

    def percent_complete(self):
        """
        Return program completion percentage based on primitive count.
        """
        if not self.primitives:
            return 0.0
        return (self.cursor / len(self.primitives)) * 100.0

    def total_time_seconds(self):
        """
        Estimate total execution time for the program.

        Time is estimated by summing length / feedrate
        for all primitives that have a valid feedrate.
        """
        total = 0.0
        for p in self.primitives:
            if p.feedrate and p.feedrate > 0:
                total += p.length() / (p.feedrate / 60.0)
        return total

    def remaining_time_seconds(self):
        """
        Estimate remaining execution time from the current cursor position.
        """
        remaining = 0.0
        for p in self.primitives[self.cursor:]:
            if p.feedrate and p.feedrate > 0:
                remaining += p.length() / (p.feedrate / 60.0)
        return remaining
