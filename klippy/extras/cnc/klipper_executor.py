# klippy/extras/cnc/klipper_executor.py

from executor import MotionExecutor


class KlipperMotionExecutor(MotionExecutor):
    """
    MotionExecutor implementation backed by Klipper's toolhead.

    This class translates fully-resolved MotionPrimitive objects
    into Klipper toolhead moves.
    """

    def __init__(self, printer):
        # Reference to Klipper's printer object
        self.printer = printer

        # Toolhead object used to queue motion
        self.toolhead = printer.lookup_object("toolhead")

    def execute(self, primitive):
        """
        Execute a single motion primitive using Klipper.

        Motion primitives are expected to:
        - Be in absolute machine coordinates
        - Have a resolved feedrate (mm/min)
        """

        # Convert feedrate from mm/min (CNC) to mm/s (Klipper)
        if primitive.feedrate is None:
            speed = None
        else:
            speed = primitive.feedrate / 60.0

        # Klipper expects absolute XYZ positions
        x, y, z = primitive.end

        # Queue the move with the toolhead
        self.toolhead.move(
            [x, y, z],
            speed
        )

    def flush(self):
        """
        Block until all queued toolhead motion has completed.
        """
        self.toolhead.wait_moves()
