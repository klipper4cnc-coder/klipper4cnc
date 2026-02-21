# klippy/extras/cnc/controller.py

from enum import Enum
from collections import deque


# New: streaming planner (raw MotionPrimitive -> PlannedPrimitive)
try:
    from .parser import parse_gcode_line
    from .planner import CNCPlanner, PlannedPrimitive
except ImportError:
    from parser import parse_gcode_line
    from planner import CNCPlanner, PlannedPrimitive


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
    - Maintain a planned lookahead queue (PlannedPrimitive)
    - Execute motion primitives one at a time
    - Handle feed hold, resume, cancel, and reset
    - Track progress and estimate remaining time
    """

    def __init__(self, executor, program=None, planner: CNCPlanner = None):
        # Motion executor (e.g. KlipperMotionExecutor)
        self.executor = executor

        # Program object (may be None in streaming mode)
        self.program = program

        # Streaming planner (required for accel/JD lookahead)
        # If you want to keep legacy behavior, you can pass planner=None and it will
        # fall back to the old raw primitive buffer mode (see below).
        self.planner = planner

        # Current controller state
        self.state = ControllerState.IDLE

        # Planned queue feeding executor
        # (In legacy mode, we use self.buffer instead)
        self.ready_queue = deque()   # deque[PlannedPrimitive]
        self.lookahead_size = 20     # number of planned moves to keep ready

        # Legacy raw buffer (used only if planner is None)
        self.buffer = deque()        # deque[MotionPrimitive]

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
        - Feeds primitives into a streaming planner (if present)
        - Maintains a planned lookahead queue
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

                # ---- FILL LOOKAHEAD ----
                if self.planner is None:
                    # Legacy mode: fill raw primitive buffer
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
                else:
                    # Planned mode: fill planned queue
                    # (Keep reading until we have enough *planned* moves ready to execute)
                    while not self.eof and len(self.ready_queue) < self.lookahead_size:
                        line = streamer.next_line()

                        if line is None:
                            # No more input: finalize planner and push remaining planned moves
                            self.eof = True
                            print("[DBG] EOF reached")
                            self.ready_queue.extend(self.planner.finish())
                            break

                        words = parse_gcode_line(line)
                        primitives = interpreter.interpret(words)

                        for p in primitives:
                            committed = self.planner.push(p)
                            if committed:
                                self.ready_queue.extend(committed)

                # ---- HOLD ----
                # If not actively running, do not execute motion
                if self.state != ControllerState.RUNNING:
                    continue

                # ---- EXECUTE ONE STEP ----
                did_step = self.step()

                # ---- TERMINATION CONDITION ----
                if not did_step:
                    if self.planner is None:
                        # End when no more data AND raw buffer is empty
                        if self.eof and not self.buffer:
                            print("[CTRL] job complete")
                            break
                    else:
                        # End when no more data AND planned queue is empty
                        if self.eof and not self.ready_queue:
                            print("[CTRL] job complete")
                            break

        finally:
            streamer.close()
            print("[DBG] streamer closed")

    def pump(self, streamer, interpreter, max_lines=50, max_steps=20):
        """
        Incremental execution: read <= max_lines, then execute <= max_steps.
        Safe to call from a reactor timer callback.
        """
        lines_read = 0
        steps = 0

        # Fill lookahead / ready queue
        if self.planner is None:
            # Legacy raw-buffer mode
            while (not self.eof
                   and len(self.buffer) < self.lookahead_size
                   and lines_read < max_lines):
                line = streamer.next_line()
                if line is None:
                    self.eof = True
                    break
                parsed = parse_gcode_line(line)
                prims = interpreter.interpret(parsed)
                for p in prims:
                    self.buffer.append(p)
                lines_read += 1
        else:
            # Planned mode
            while (not self.eof
                   and len(self.ready_queue) < self.lookahead_size
                   and lines_read < max_lines):
                line = streamer.next_line()
                if line is None:
                    self.eof = True
                    # Flush remaining planned moves
                    self.ready_queue.extend(self.planner.finish())
                    break

                parsed = parse_gcode_line(line)
                prims = interpreter.interpret(parsed)
                for p in prims:
                    committed = self.planner.push(p)
                    if committed:
                        self.ready_queue.extend(committed)
                lines_read += 1

        # Execute a few steps if RUNNING
        if self.state == ControllerState.RUNNING and max_steps > 0:
            while steps < max_steps:
                did = self.step()
                if not did:
                    break
                steps += 1

        return lines_read, steps

    def is_done(self):
        """
        True when the input is exhausted and there's no more internal work to execute.
        Does not account for toolhead buffering (the runner handles draining).
        """
        if self.planner is None:
            return self.eof and not self.buffer
        return self.eof and not self.ready_queue

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

        # Clear buffers
        self.buffer.clear()
        self.ready_queue.clear()

        # Reset planner
        if self.planner is not None:
            self.planner.reset()

        # Reset stream state / progress
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
        Execute a single motion primitive from the lookahead queue/buffer.

        :return: True if a step was executed, False otherwise
        """
        if self.state != ControllerState.RUNNING:
            return False

        # ---- Legacy mode ----
        if self.planner is None:
            if not self.buffer:
                return False

            p = self.buffer.popleft()
            return self._execute_primitive(p)

        # ---- Planned mode ----
        if not self.ready_queue:
            return False

        item = self.ready_queue.popleft()
        if isinstance(item, PlannedPrimitive):
            p = item.primitive
        else:
            # Safety: in case something non-planned was queued
            p = item

        return self._execute_primitive(p)

    def _execute_primitive(self, p):
        """
        Execute a MotionPrimitive and update progress.
        """
        # Feedrate must be fully resolved by execution time
        if p.feedrate is None:
            raise RuntimeError(
                f"Feedrate not resolved at execution time "
                f"(pos={p.start} → {p.end}, mode={p.motion})"
            )

        # Send motion to the executor (Klipper backend or mock)
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

        # Feedrate is tracked by the executor (optional).
        # If your executor doesn't provide this, ETA will be '?'.
        feed = getattr(self.executor, "last_feedrate", None)

        if feed and feed > 0:
            remaining_time = (remaining_len / feed) * 60.0
            eta = format_time(remaining_time)
        else:
            eta = "?"

        print(f"[PROGRESS] {pct:.1f}% | ETA {eta}")

    def flush(self):
        """
        Flush any queued motion in the executor.
        """
        self.executor.flush()
