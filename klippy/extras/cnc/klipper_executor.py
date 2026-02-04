# klippy/extras/cnc/klipper_executor.py

from executor import MotionExecutor

class KlipperMotionExecutor(MotionExecutor):
    def __init__(self, printer):
        self.printer = printer
        self.toolhead = printer.lookup_object("toolhead")

    def execute(self, primitive):
        # Convert feedrate: mm/min → mm/s
        if primitive.feedrate is None:
            speed = None
        else:
            speed = primitive.feedrate / 60.0

        # Klipper wants absolute positions
        x, y, z = primitive.end

        self.toolhead.move(
            [x, y, z],
            speed
        )

    def flush(self):
        self.toolhead.wait_moves()
