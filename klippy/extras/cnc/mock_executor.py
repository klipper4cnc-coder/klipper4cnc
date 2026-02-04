# klippy/extras/cnc/mock_executor.py

from executor import MotionExecutor

class MockMotionExecutor(MotionExecutor):
    def __init__(self):
        self.count = 0
        self.last_feedrate = None

    def execute(self, primitive):
        self.count += 1
        self.last_feedrate = primitive.feedrate
        print(f"[EXEC {self.count}] {primitive}")

    def flush(self):
        print("[EXEC] flush")
