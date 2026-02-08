# klippy/extras/cnc/controller.py

from enum import Enum
from collections import deque
from parser import parse_gcode_line


def format_time(seconds):
    """
    Convert a time duration in seconds into a human-readable string.

    Examples:
    - 45    -> "45s"
    - 125   -> "2m 5s"
    - 3723  -> "1h 2m 3s"

    Used for ETA reporting during CNC job execution.
    """
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
    """
    High-level execution state of the CNC controller.
    """
    IDLE = "idle"          # Not running a job
    RUNNING = "running"    # Actively executing motion
    HOLD = "hold"          # Feed hold / paused
    CANCELLED = "cancelled"  # Cancelled, requires reset


class CNCController:
    """
    Central execution controller for CNC jobs.

    Responsibilities:
    - Stream G-code from a source (via streamer)
    - Convert G-code lines into motion primitives
    - Maintain a lookahead buffer
    - Execute motion primitives one at a time
    - Handle feed hold, resume, cancel, and reset
    - Track progress and estimate remaining time
    """

    def __init__(self, executor, program=None):
        # Motion executor (e.g. KlipperMotionExecutor)
        self.executor = executor

        # Program object (may be None in streaming mode)
        self.program = program

        # Current controller state
        self.state = ControllerState.IDLE

        # Lookahead buffer for motion primitives
        self.buffer = deque()
        self.lookahead_size = 20

        # End-of-file flag for streaming mode
        self.eof = False

        # Progress tracking (safe for streaming execution)
        self.total_length = 0.0
        self.completed_length = 0.0

        # Last distance at which progress was reported
        self._last_reported = -1

    # -------------------------
    # Streaming execution
    # -------------------------

    def set_total_length(self, total_length):
        """
        Set the total planned motion length for progress reporting.
        """
        self.total_length = total_length

    def run_stream(self, streamer, interpreter):
        """
        Execute a CNC job by streaming G-code lines.

        This method:
        - Reads G-code lines incrementally from the streamer
        - Parses and interprets them into motion primitives
        - Maintains a lookahead buffer
        - Executes primitives as long as the controller is RUNNING
        """
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
                        # No more input from streamer
                        self.eof = True
                        print("[DBG] EOF reached")
                        break

                    # Parse raw G-code line into words
                    words = parse_gcode_line(line)

                    # Interpret words into motion primitives
                    primitives = interpreter.interpret(words)

                    for p in primitives:
                        self.buffer.append(p)
                        # ❌ IMPORTANT:
                        # Do NOT modify total_length or progress here.
                        # Length tracking is handled elsewhere and must
                        # remain consistent during streaming.
                        # self.total_units += 1
                        # self.total_length += p.length()

                # ---- HOLD ----
                # If not actively running, do not execute motion
                if self.state != ControllerState.RUNNING:
                    continue

                # ---- EXECUTE ONE STEP ----
                did_step = self.step()

                # ---- TERMINATION CONDITION ----
                if not did_step:
                    # End when no more data AND buffer is empty
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
        """
        Begin or resume execution.

        Raises if the controller has been cancelled and not reset.
        """
        if self.state == ControllerState.CANCELLED:
            raise RuntimeError("Controller is cancelled; reset required")
        self.state = ControllerState.RUNNING

    def feed_hold(self):
        """
        Pause execution immediately (feed hold).
        """
        if self.state == ControllerState.RUNNING:
            self.state = ControllerState.HOLD
            self.report_progress()
            print("[CTRL] feed hold")

    def resume(self):
        """
        Resume execution after a feed hold.
        """
        if self.state == ControllerState.HOLD:
            self.state = ControllerState.RUNNING
            print("[CTRL] resume")

    def reset(self):
        """
        Reset controller state and clear all execution buffers.
        """
        self.state = ControllerState.IDLE
        self.buffer.clear()
        self.eof = False
        self.total_length = 0.0
        self.completed_length = 0.0
        self._last_reported = -1
        print("[CTRL] reset")

    def cancel(self):
        """
        Cancel execution permanently until reset is called.
        """
        if self.state != ControllerState.CANCELLED:
            self.state = ControllerState.CANCELLED
            print("[CTRL] cancelled")

    # -------------------------
    # Execution
    # -------------------------

    def step(self):
        """
        Execute a single motion primitive from the lookahead buffer.

        :return: True if a step was executed, False otherwise
        """
        if self.state != ControllerState.RUNNING:
            return False

        if not self.buffer:
            return False

        # Pop next motion primitive
        p = self.buffer.popleft()

        # Feedrate must be fully resolved by execution time
        if p.feedrate is None:
            raise RuntimeError(
                f"Feedrate not resolved at execution time "
                f"(pos={p.start} → {p.end}, mode={p.motion})"
            )

        # Send motion to the executor (Klipper)
        self.executor.execute(p)

        # Update completed motion length
        self.completed_length += p.length()

        # Report progress every fixed distance of actual motion
        REPORT_STEP = 10.0  # mm

        if self.completed_length - self._last_reported >= REPORT_STEP:
            self._last_reported = self.completed_length
            self.report_progress()

        return True

    # -------------------------
    # Progress / ETA
    # -------------------------

    def report_progress(self):
        """
        Print progress percentage and estimated remaining time.
        """
        if self.total_length <= 0.0:
            return

        pct = (self.completed_length / self.total_length) * 100.0
        remaining_len = max(self.total_length - self.completed_length, 0.0)

        # Feedrate is tracked by the executor
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
        """
        Flush any queued motion in the executor.
        """
        self.executor.flush()
