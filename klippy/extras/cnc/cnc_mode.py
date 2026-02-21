# klippy/extras/cnc/cnc_mode.py

import os
import logging

try:
    from .modal_state import CNCModalState
    from .interpreter import CNCInterpreter
    from .limits import SoftLimits
    from .controller import CNCController, ControllerState
    from .klipper_executor import KlipperMotionExecutor
    from .planner import CNCPlanner
    from .streamer import GCodeStreamer
except ImportError:
    from modal_state import CNCModalState
    from interpreter import CNCInterpreter
    from limits import SoftLimits
    from controller import CNCController, ControllerState
    from klipper_executor import KlipperMotionExecutor
    from planner import CNCPlanner
    from streamer import GCodeStreamer


class CNCMode:
    """
    Reactor/timer-driven CNC streaming execution.
    """

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.log = logging.getLogger(__name__)

        def cfg_getfloat(name, default):
            fn = getattr(config, "getfloat", None)
            return fn(name, default) if fn else default

        def cfg_getint(name, default):
            fn = getattr(config, "getint", None)
            return fn(name, default) if fn else default

        # Modal state + interpreter
        self.state = CNCModalState()

        # Soft limits (you’ll likely want these configurable later)
        limits = SoftLimits({
            "X": (0.0, 300.0),
            "Y": (0.0, 300.0),
            "Z": (-100.0, 0.0),
        })
        self.interpreter = CNCInterpreter(self.state, soft_limits=limits)

        # Executor -> toolhead
        self.executor = KlipperMotionExecutor(self.printer)

        # Planner
        self.planner = CNCPlanner(
            max_velocity=cfg_getfloat("max_velocity", 150.0),
            max_accel=cfg_getfloat("max_accel", 1000.0),
            junction_deviation=cfg_getfloat("junction_deviation", 0.05),
            buffer_time=cfg_getfloat("planner_buffer_time", 0.250),
            keep_tail_moves=cfg_getint("planner_keep_tail_moves", 2),
            max_window_moves=cfg_getint("planner_max_window_moves", 200),
        )

        # Controller
        self.controller = CNCController(self.executor, program=None, planner=self.planner)

        # Runner tuning (important for HOLD/CANCEL responsiveness)
        self.pump_interval = cfg_getfloat("pump_interval", 0.010)  # seconds
        self.max_lines_per_pump = cfg_getint("max_lines_per_pump", 25)
        self.max_steps_per_pump = cfg_getint("max_steps_per_pump", 15)
        self.max_toolhead_buffer_time = cfg_getfloat("max_toolhead_buffer_time", 0.25)
        self.drain_check_interval = cfg_getfloat("drain_check_interval", 0.050)

        # Job state
        self._streamer = None
        self._filepath = None
        self._job_state = "idle"  # idle|running|hold|draining
        self._work_timer = self.reactor.register_timer(self._work_handler)

        # Register commands
        self.gcode.register_command("CNC_START", self.cmd_start)
        self.gcode.register_command("CNC_FEED_HOLD", self.cmd_hold)
        self.gcode.register_command("CNC_RESUME", self.cmd_resume)
        self.gcode.register_command("CNC_CANCEL", self.cmd_cancel)
        self.gcode.register_command("CNC_RESET", self.cmd_reset)

    def _resolve_filepath(self, gcmd):
        filename = None
        try:
            filename = gcmd.get("FILE", None)
        except Exception:
            filename = None

        if filename:
            return filename

        # Optional: infer from virtual_sdcard if present
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

        raise gcmd.error("CNC_START requires FILE=<path>, or select a file via virtual_sdcard.")

    def cmd_start(self, gcmd):
        if self._job_state != "idle":
            raise gcmd.error(f"CNC job already active (state={self._job_state}). CNC_RESET first.")

        filepath = self._resolve_filepath(gcmd)

        if not os.path.isabs(filepath):
            vsd = self.printer.lookup_object("virtual_sdcard", None)
            sdroot = getattr(vsd, "sdcard_dirname", None) if vsd is not None else None
            if sdroot:
                filepath = os.path.join(sdroot, filepath)

        if not os.path.exists(filepath):
            raise gcmd.error(f"File not found: {filepath}")

        # Clean run
        self.state.reset()
        self.controller.reset()

        self._filepath = filepath
        self._streamer = GCodeStreamer(filepath)
        self._streamer.open()

        self.controller.start()
        self._job_state = "running"

        gcmd.respond_info(f"CNC_START: started {os.path.basename(filepath)}")
        self.reactor.update_timer(self._work_timer, self.reactor.NOW)

    def cmd_hold(self, gcmd):
        if self._job_state not in ("running", "draining"):
            gcmd.respond_info("CNC_FEED_HOLD: no active job")
            return
        self.controller.feed_hold()
        self._job_state = "hold"
        gcmd.respond_info("CNC_FEED_HOLD: holding (no new moves will be queued)")
        self.reactor.update_timer(self._work_timer, self.reactor.NOW)

    def cmd_resume(self, gcmd):
        if self._job_state != "hold":
            gcmd.respond_info("CNC_RESUME: not in hold")
            return
        self.controller.resume()
        self._job_state = "running"
        gcmd.respond_info("CNC_RESUME: resuming")
        self.reactor.update_timer(self._work_timer, self.reactor.NOW)

    def cmd_cancel(self, gcmd):
        if self._job_state == "idle":
            gcmd.respond_info("CNC_CANCEL: no active job")
            return
        self.controller.cancel()
        self._stop_job("cancelled")
        gcmd.respond_info("CNC_CANCEL: stopped (queued toolhead motion may finish quickly)")

    def cmd_reset(self, gcmd):
        # Hard reset of module state
        if self._job_state != "idle":
            self._stop_job("reset")
        self.state.reset()
        self.controller.reset()
        gcmd.respond_info("CNC_RESET: state cleared")

    def _stop_job(self, why):
        try:
            if self._streamer is not None:
                self._streamer.close()
        finally:
            self._streamer = None
            self._filepath = None
            self._job_state = "idle"
            self.reactor.update_timer(self._work_timer, self.reactor.NEVER)
        self.log.info("CNC job stopped: %s", why)

    def _work_handler(self, eventtime):
        if self._job_state == "idle":
            return self.reactor.NEVER

        try:
            # Cancelled state from controller
            if self.controller.state == ControllerState.CANCELLED:
                self._stop_job("cancelled")
                return self.reactor.NEVER

            # If we’re over-buffered in toolhead, yield
            buf = self.executor.buffer_time(eventtime)
            if buf > self.max_toolhead_buffer_time:
                return eventtime + self.pump_interval

            # Pump controller
            if self._job_state == "hold":
                # Fill lookahead but do not execute steps
                self.controller.pump(self._streamer, self.interpreter,
                                     max_lines=self.max_lines_per_pump,
                                     max_steps=0)
            else:
                self.controller.pump(self._streamer, self.interpreter,
                                     max_lines=self.max_lines_per_pump,
                                     max_steps=self.max_steps_per_pump)

            # Transition to draining after controller reports no more internal work
            if self._job_state != "hold" and self.controller.is_done():
                self._job_state = "draining"

            if self._job_state == "draining":
                # Wait until toolhead queue drains, then declare done
                buf2 = self.executor.buffer_time(eventtime)
                if buf2 <= 0.010:
                    fname = os.path.basename(self._filepath or "job")
                    self._stop_job("complete")
                    self.gcode.respond_info(f"CNC job complete: {fname}")
                    return self.reactor.NEVER
                return eventtime + self.drain_check_interval

            return eventtime + self.pump_interval

        except Exception as e:
            # Stop job and surface error
            self.log.exception("CNC runner exception")
            self._stop_job(f"error: {e}")
            self.gcode.respond_info(f"CNC job error: {e}")
            return self.reactor.NEVER


def load_config(config):
    return CNCMode(config)