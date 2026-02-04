# klippy/extras/cnc/test_modal.py

from modal_state import CNCModalState

state = CNCModalState()

state.set_units("G21")
state.set_distance_mode("G90")
state.update_feedrate(1200)

print("Initial position:", state.position)

target = {"X": 10, "Y": 5}
pos = state.resolve_target(target)

print("Resolved target:", pos)

state.position = list(pos)

state.set_distance_mode("G91")
pos2 = state.resolve_target({"Z": -2})

print("Relative Z move:", pos2)
