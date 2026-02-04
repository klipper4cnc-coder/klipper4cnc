from modal_state import CNCModalState
from interpreter import CNCInterpreter
from limits import SoftLimits
#from program import CNCProgram
from controller import CNCController
from klipper_executor import KlipperMotionExecutor

class CNCMode:
    def __init__(self, config):
        self.printer = config.get_printer()

        self.state = CNCModalState()
        #self.program = CNCProgram()

        limits = SoftLimits({
            "X": (0.0, 300.0),
            "Y": (0.0, 300.0),
            "Z": (-100.0, 0.0),
        })

        self.interpreter = CNCInterpreter(self.state, soft_limits=limits)
        self.executor = KlipperMotionExecutor(self.printer)
        self.controller = CNCController(self.executor, self.program)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("CNC_START", self.cmd_start)
        gcode.register_command("CNC_FEED_HOLD", self.cmd_hold)
        gcode.register_command("CNC_RESUME", self.cmd_resume)
        gcode.register_command("CNC_RESET", self.cmd_reset)

    def cmd_start(self, gcmd):
        self.controller.start()
        while self.program.has_next():
            if not self.controller.step():
                break
        self.executor.flush()

    def cmd_hold(self, gcmd):
        self.controller.feed_hold()

    def cmd_resume(self, gcmd):
        self.controller.resume()
        while self.program.has_next():
            if not self.controller.step():
                break
        self.executor.flush()

    def cmd_reset(self, gcmd):
        self.controller.reset()
        self.program.reset()

def load_config(config):
    return CNCMode(config)
