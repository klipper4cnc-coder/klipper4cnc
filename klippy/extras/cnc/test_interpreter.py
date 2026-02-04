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

limits = SoftLimits({
    "X": (-100.0, 100.0),
    "Y": (-100.0, 100.0),
    "Z": (-50.0, 100.0),
})

state = CNCModalState()
interp = CNCInterpreter(state, soft_limits=limits)
#program_buffer = CNCProgram()
executor = MockMotionExecutor()
controller = CNCController(executor) #, program_buffer)

# -------------------------
# Test program
# -------------------------

program = [
    "G21",
    "G90",
    "F800",
    "G1 X0 Y0",
    "G2 X0 Y0 I10 J0",
]








# -------------------------
# Execution loop
# -------------------------
# ---- interpret once ----
for line in program:
    words = parse_gcode_line(line)
    primitives = interp.interpret(words)
    program_buffer.load_primitives(primitives)

# ---- execute with hold ----
controller.start()

while program_buffer.has_next():
    if executor.count == 10:
        controller.feed_hold()

    if not controller.step():
        break

# ---- resume ----
controller.resume()

while program_buffer.has_next():
    controller.step()

controller.flush()


