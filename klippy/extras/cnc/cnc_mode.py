from modal_state import CNCModalState
from interpreter import CNCInterpreter
from limits import SoftLimits
#from program import CNCProgram
from controller import CNCController
from klipper_executor import KlipperMotionExecutor


class CNCMode:
    """
    Top-level CNC mode integration class.

    This class acts as the glue layer between:
    - Klipper's printer / gcode system
    - CNC modal state
    - The CNC interpreter
    - The motion executor
    - The controller that sequences execution
    """

    def __init__(self, config):
        # Reference to Klipper's printer object
        self.printer = config.get_printer()

        # CNC modal state (units, distance mode, feed rate, etc.)
        self.state = CNCModalState()

        # CNCProgram is currently disabled / not yet wired up
        # self.program = CNCProgram()

        # Define software-enforced axis travel limits
        # These are applied by the interpreter before motion is executed
        limits = SoftLimits({
            "X": (0.0, 300.0),
            "Y": (0.0, 300.0),
            "Z": (-100.0, 0.0),
        })

        # Interpreter converts parsed G-code into motion primitives
        self.interpreter = CNCInterpreter(self.state, soft_limits=limits)

        # Executor converts motion primitives into Klipper moves
        self.executor = KlipperMotionExecutor(self.printer)

        # Controller coordinates stepping through the CNC program
        self.controller = CNCController(self.executor, self.program)

        # Register CNC control commands with Klipper's gcode object
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("CNC_START", self.cmd_start)
        gcode.register_command("CNC_FEED_HOLD", self.cmd_hold)
        gcode.register_command("CNC_RESUME", self.cmd_resume)
        gcode.register_command("CNC_RESET", self.cmd_reset)

    def cmd_start(self, gcmd):
        """
        Begin execution of the loaded CNC program.

        Steps through the program until completion or interruption.
        """
        self.controller.start()

        while self.program.has_next():
            # controller.step() returns False when execution should stop
            if not self.controller.step():
                break

        # Ensure all queued motion is sent to Klipper
        self.executor.flush()

    def cmd_hold(self, gcmd):
        """
        Immediately pause CNC execution (feed hold).
        """
        self.controller.feed_hold()

    def cmd_resume(self, gcmd):
        """
        Resume execution after a feed hold.
        """
        self.controller.resume()

        while self.program.has_next():
            if not self.controller.step():
                break

        self.executor.flush()

    def cmd_reset(self, gcmd):
        """
        Reset controller and program state.
        """
        self.controller.reset()
        self.program.reset()


def load_config(config):
    # Klipper module entry point
    return CNCMode(config)
