# klippy/extras/cnc/cnc_mode.py

import os

from modal_state import CNCModalState
from interpreter import CNCInterpreter
from limits import SoftLimits
from controller import CNCController
from klipper_executor import KlipperMotionExecutor
from planner import CNCPlanner
from streamer import GCodeStreamer


class CNCMode:
    """
    CNC mode integration (streaming).

    - No CNCProgram.
    - CNC_START streams a file through interpreter -> planner -> controller -> executor.
    """

    def __init__(self, config):
        self.printer = config.get_printer()

        # Modal state + interpreter
        self.state = CNCModalState()

        limits = SoftLimits({
            "X": (0.0, 300.0),
            "Y": (0.0, 300.0),
            "Z": (-100.0, 0.0),
        })
        self.interpreter = CNCInterpreter(self.state, soft_limits=limits)

        # Executor -> Klipper toolhead
        self.executor = KlipperMotionExecutor(self.printer)

        # Planner settings (use config values with safe defaults)
        # In real Klipper, config provides getfloat/getint.
        max_velocity = getattr(config, "getfloat", lambda k, d=None, **kw: d)("max_velocity", 150.0)
        max_accel = getattr(config, "getfloat", lambda k, d=None, **kw: d)("max_accel", 1000.0)
        junction_deviation = getattr(config, "getfloat", lambda k, d=None, **kw: d)("junction_deviation", 0.05)

        buffer_time = getattr(config, "getfloat", lambda k, d=None, **kw: d)("planner_buffer_time", 0.250)
        keep_tail_moves = getattr(config, "getint", lambda k, d=None, **kw: d)("planner_keep_tail_moves", 2)
        max_window_moves = getattr(config, "getint", lambda k, d=None, **kw: d)("planner_max_window_moves", 200)

        self.planner = CNCPlanner(
            max_velocity=max_velocity,
            max_accel=max_accel,
            junction_deviation=junction_deviation,
            buffer_time=buffer_time,
            keep_tail_moves=keep_tail_moves,
            max_window_moves=max_window_moves,
        )

        # Controller coordinates streaming + planned execution
        self.controller = CNCController(self.executor, program=None, planner=self.planner)

        # Register CNC commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("CNC_START", self.cmd_start)
        gcode.register_command("CNC_FEED_HOLD", self.cmd_hold)
        gcode.register_command("CNC_RESUME", self.cmd_resume)
        gcode.register_command("CNC_RESET", self.cmd_reset)
        gcode.register_command("CNC_CANCEL", self.cmd_cancel)

    def _resolve_filepath(self, gcmd):
        # 1) Explicit FILE=...
        filename = None
        try:
            filename = gcmd.get("FILE", None)
        except Exception:
            filename = None

        if filename:
            return filename

        # 2) Optional: infer from virtual_sdcard "current file"
        vsd = self.printer.lookup_object("virtual_sdcard", None)
        if vsd is not None:
            for attr in ("file_path", "get_file_path"):
                fn = getattr(vsd, attr, None)
                if callable(fn):
                    path = fn()
                    if path:
                        return path
            path = getattr(vsd, "current_file", None)
            if path:
                return path

        raise gcmd.error("CNC_START requires FILE=<path>, or select a file via virtual_sdcard and run CNC_START with no args.")

    def cmd_start(self, gcmd):
        """
        Start streaming a file:
          CNC_START FILE=/path/to/job.nc
        """
        filepath = self._resolve_filepath(gcmd)

        if not os.path.isabs(filepath):
            # If relative, try resolving relative to Klipper's vSD card dir if present
            vsd = self.printer.lookup_object("virtual_sdcard", None)
            sdroot = getattr(vsd, "sdcard_dirname", None) if vsd is not None else None
            if sdroot:
                filepath = os.path.join(sdroot, filepath)

        if not os.path.exists(filepath):
            raise gcmd.error(f"File not found: {filepath}")

        # Reset state for a clean run
        self.controller.reset()

        # Stream file
        streamer = GCodeStreamer(filepath)
        self.controller.start()
        self.controller.run_stream(streamer, self.interpreter)

        # Ensure all queued motion is sent to toolhead
        self.executor.flush()

    def cmd_hold(self, gcmd):
        self.controller.feed_hold()

    def cmd_resume(self, gcmd):
        self.controller.resume()

    def cmd_reset(self, gcmd):
        self.controller.reset()

    def cmd_cancel(self, gcmd):
        self.controller.cancel()


def load_config(config):
    return CNCMode(config)
