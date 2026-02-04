from file_loader import GCodeFileLoader
#from program import CNCProgram
from modal_state import CNCModalState
from interpreter import CNCInterpreter
from limits import SoftLimits
from mock_executor import MockMotionExecutor
from controller import CNCController
from parser import parse_gcode_line


# -------------------------
# Functions
# -------------------------
def prescan_total_length(
    filepath: str,
    arc_tolerance: float,
    soft_limits=None,
):
    from file_loader import GCodeFileLoader
    from modal_state import CNCModalState
    from interpreter import CNCInterpreter
    from parser import parse_gcode_line

    # Fresh, isolated modal state
    state = CNCModalState()
    state.arc_tolerance = arc_tolerance

    # IMPORTANT: match the real constructor signature
    interpreter = CNCInterpreter(state, soft_limits=None)

    loader = GCodeFileLoader(filepath)
    lines = loader.load()

    total_length = 0.0

    for line in lines:
        parsed = parse_gcode_line(line)
        primitives = interpreter.interpret(parsed)

        for p in primitives:
            total_length += p.length()

    return total_length



# -------------------------
# Setup
# -------------------------

limits = SoftLimits({
    "X": (-1000.0, 1000.0),
    "Y": (-1000.0, 1000.0),
    "Z": (-1000.0, 1000.0),
})

# --- Prescan state (isolated) ---
prescan_state = CNCModalState()
prescan_state.arc_tolerance = 0.01  # or whatever you want

total_length = prescan_total_length(
    filepath="test.nc",
    arc_tolerance=prescan_state.arc_tolerance,
)

# --- Runtime state ---
state = CNCModalState()
state.arc_tolerance = prescan_state.arc_tolerance

interp = CNCInterpreter(state, soft_limits=limits)

executor = MockMotionExecutor()
controller = CNCController(executor, program=None)
controller.set_total_length(total_length)

from streamer import GCodeStreamer
streamer = GCodeStreamer("test.nc")

controller.start()
controller.run_stream(streamer, interp)
controller.flush()
