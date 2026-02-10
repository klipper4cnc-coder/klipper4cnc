# klippy/extras/cnc/test_modal.py

from modal_state import CNCModalState


# Create a fresh modal state instance
state = CNCModalState()

# -------------------------
# Absolute positioning test
# -------------------------

# Set units to millimeters
state.set_units("G21")

# Set absolute distance mode
state.set_distance_mode("G90")

# Set feedrate (units/min)
state.update_feedrate(1200)

print("Initial position:", state.position)

# Define an absolute target position
target = {"X": 10, "Y": 5}

# Resolve target into absolute machine coordinates
pos = state.resolve_target(target)

print("Resolved target:", pos)

# Update current position to the resolved target
state.position = list(pos)


# -------------------------
# Incremental positioning test
# -------------------------

# Switch to incremental distance mode
state.set_distance_mode("G91")

# Resolve a relative Z move
pos2 = state.resolve_target({"Z": -2})

print("Relative Z move:", pos2)
