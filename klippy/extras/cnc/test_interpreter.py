from modal_state import CNCModalState
from parser import parse_gcode_line
from interpreter import CNCInterpreter
from limits import SoftLimits
from mock_executor import MockMotionExecutor
from controller import CNCController
#from program import CNCProgram


# -------------------------
# Setup
# -------------------------

# Define soft limits for test execution
limits = SoftLimits({
    "X": (-100.0, 100.0),
    "Y": (-100.0, 100.0),
    "Z": (-50.0, 100.0),
})

# Fresh modal state for the interpreter
state = CNCModalState()

# Interpreter converts parsed G-code into motion primitives
interp = CNCInterpreter(state, soft_limits=limits)

# Program buffer is intentionally commented out here
# program_buffer = CNCProgram()

# Mock executor to observe execution behavior
executor = MockMotionExecutor()

# Controller drives execution of primitives
# Program object is omitted / commented out
controller = CNCController(executor)  # , program_buffer)


# -------------------------
# Test program
# -------------------------

# Simple test G-code program exercising:
# - unit selection
# - absolute positioning
# - feedrate
# - linear move
# - full-circle arc
program = [
    "G21",              # millimeters
    "G90",              # absolute positioning
    "F800",             # feedrate
    "G1 X0 Y0",         # linear move
    "G2 X0 Y0 I10 J0",  # full circle CW arc
]


# -------------------------
# Execution loop
# -------------------------

# ---- interpret once ----
# Convert all G-code lines into motion primitives up front
for line in program:
    words = parse_gcode_line(line)
    primitives = interp.interpret(words)

    # NOTE:
    # program_buffer is commented out, but shown here
    # to illustrate the intended usage pattern
    program_buffer.load_primitives(primitives)

# ---- execute with hold ----
controller.start()

while program_buffer.has_next():
    # Trigger a feed hold after some execution steps
    if executor.count == 10:
        controller.feed_hold()

    # Step controller execution
    if not controller.step():
        break

# ---- resume ----
controller.resume()

while program_buffer.has_next():
    controller.step()

# Flush any remaining queued motion
controller.flush()
