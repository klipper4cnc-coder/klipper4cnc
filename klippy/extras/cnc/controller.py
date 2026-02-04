# klippy/extras/cnc/controller.py

from enum import Enum
from collections import deque
from parser import parse_gcode_line


def format_time(seconds):
    seconds = int(max(0, seconds))
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60

    if h > 0:
        return f"{h}h {m}m {s}s"
    elif m > 0:
        return f"{m}m {s}s"
    else:
        return f"{s}s"


class ControllerState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    HOLD = "hold"
    CANCELLED = "cancelled"


class CNCController:
    def __init__(self, executor, program=None):
        self.executor = executor
        self.program = program

        self.state = ControllerState.IDLE

        # Lookahead buffer
        self.buffer = deque()
        self.lookahead_size = 20

        # Streaming state
        self.eof = False

        # Progress tracking (stream-safe)
        self.total_length = 0.0
        self.completed_length = 0.0

        self._last_reported = -1

    # -------------------------
    # Streaming execution
    # -------------------------

    def set_total_length(self, total_length):
        self.total_length = total_length

    def run_stream(self, streamer, interpreter):
        print("[DBG] run_stream entered")
        streamer.open()

        try:
            while True:
                # ---- CANCEL ----
                if self.state == ControllerState.CANCELLED:
                    print("[DBG] cancelled")
                    break

                # ---- FILL LOOKAHEAD BUFFER ----
                while not self.eof and len(self.buffer) < self.lookahead_size:
                    line = streamer.next_line()

                    if line is None:
                        self.eof = True
                        print("[DBG] EOF reached")
                        break

                    words = parse_gcode_line(line)
                    primitives = interpreter.interpret(words)

                    for p in primitives:
                        self.buffer.append(p)
                        # ❌ DO NOT TOUCH TOTALS HERE
                        # self.total_units += 1
                        # self.total_length += p.length()

                # ---- HOLD ----
                if self.state != ControllerState.RUNNING:
                    continue

                # ---- EXECUTE ONE STEP ----
                did_step = self.step()

                # ---- TERMINATION CONDITION ----
                if not did_step:
                    if self.eof and not self.buffer:
                        print("[CTRL] job complete")
                        break

        finally:
            streamer.close()
            print("[DBG] streamer closed")

    # -------------------------
    # Control commands
    # -------------------------

    def start(self):
        if self.state == ControllerState.CANCELLED:
            raise RuntimeError("Controller is cancelled; reset required")
        self.state = ControllerState.RUNNING

    def feed_hold(self):
        if self.state == ControllerState.RUNNING:
            self.state = ControllerState.HOLD
            self.report_progress()
            print("[CTRL] feed hold")

    def resume(self):
        if self.state == ControllerState.HOLD:
            self.state = ControllerState.RUNNING
            print("[CTRL] resume")

    def reset(self):
        self.state = ControllerState.IDLE
        self.buffer.clear()
        self.eof = False
        self.total_length = 0.0
        self.completed_length = 0.0
        self._last_reported = -1
        print("[CTRL] reset")

    def cancel(self):
        if self.state != ControllerState.CANCELLED:
            self.state = ControllerState.CANCELLED
            print("[CTRL] cancelled")

    # -------------------------
    # Execution
    # -------------------------

    def step(self):
        if self.state != ControllerState.RUNNING:
            return False

        if not self.buffer:
            return False

        p = self.buffer.popleft()

        if p.feedrate is None:
            raise RuntimeError(
                f"Feedrate not resolved at execution time "
                f"(pos={p.start} → {p.end}, mode={p.motion})"
            )

        self.executor.execute(p)

        self.completed_length += p.length()

        # Report every 10mm of actual motion
        REPORT_STEP = 10.0  # mm

        if self.completed_length - self._last_reported >= REPORT_STEP:
            self._last_reported = self.completed_length
            self.report_progress()

        return True

    # -------------------------
    # Progress / ETA
    # -------------------------

    def report_progress(self):
        if self.total_length <= 0.0:
            return

        pct = (self.completed_length / self.total_length) * 100.0
        remaining_len = max(self.total_length - self.completed_length, 0.0)

        feed = getattr(self.executor, "last_feedrate", None)

        if feed and feed > 0:
            remaining_time = (remaining_len / feed) * 60.0
            eta = format_time(remaining_time)
        else:
            eta = "?"

        print(
            f"[PROGRESS] "
            f"{pct:.1f}% | ETA {eta}"
        )

    def flush(self):
        self.executor.flush()
