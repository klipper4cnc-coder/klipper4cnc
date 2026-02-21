# klippy/extras/cnc/klipper_executor.py

try:
    from .executor import MotionExecutor
except ImportError:
    from executor import MotionExecutor


class KlipperMotionExecutor(MotionExecutor):
    def __init__(self, printer):
        self.printer = printer
        self.toolhead = printer.lookup_object("toolhead")

    def execute(self, primitive):
        # Feedrate is in units/min (mm/min). Klipper expects speed in mm/s.
        if primitive.feedrate is None:
            raise ValueError("Primitive has no feedrate; cannot execute on toolhead")
        speed = primitive.feedrate / 60.0

        x, y, z = primitive.end

        # Preserve E axis (toolhead uses 4-axis commanded_pos)
        curpos = list(self.toolhead.get_position())
        if len(curpos) < 4:
            # Extremely defensive; toolhead should always be 4.
            curpos = (curpos + [0.0, 0.0, 0.0, 0.0])[:4]
        newpos = curpos[:]
        newpos[0], newpos[1], newpos[2] = x, y, z

        self.toolhead.move(newpos, speed)

    def buffer_time(self, eventtime):
        # Approximate queued motion time remaining in toolhead.
        # This is how you keep HOLD/CANCEL responsive by not over-buffering.
        mcu = getattr(self.toolhead, "mcu", None)
        if mcu is None:
            return 0.0
        est = mcu.estimated_print_time(eventtime)
        last = self.toolhead.get_last_move_time()
        return max(0.0, last - est)

    def flush(self):
        # Avoid calling wait_moves() from a timer-driven runner.
        # Keep this for legacy/manual usage only.
        self.toolhead.wait_moves()