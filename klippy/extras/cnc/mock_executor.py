# klippy/extras/cnc/mock_executor.py

from executor import MotionExecutor


class MockMotionExecutor(MotionExecutor):
    """
    Mock MotionExecutor for testing and debugging.

    This executor does not perform real motion. Instead, it:
    - Counts executed primitives
    - Records the last feedrate
    - Prints execution information to stdout

    Useful for unit tests and dry-run validation.
    """

    def __init__(self):
        # Number of motion primitives executed
        self.count = 0

        # Last feedrate received (used for progress / ETA testing)
        self.last_feedrate = None

    def execute(self, primitive):
        """
        "Execute" a motion primitive by logging it.
        """
        self.count += 1
        self.last_feedrate = primitive.feedrate
        print(f"[EXEC {self.count}] {primitive}")

    def flush(self):
        """
        Flush execution (no-op for mock executor).
        """
        print("[EXEC] flush")
