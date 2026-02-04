# klippy/extras/cnc/modal_state.py

class CNCModalState:
    def __init__(self):
        # Units
        self.units = "mm"          # G21 / G20
        self.units_scale = 1.0     # mm

        # Distance mode
        self.absolute = True       # G90 / G91

        # Motion plane
#        self.plane = "XY"          # G17 (XY only for now)
        self.plane = Plane.XY  # default per spec

        # Feedrate (mm/min)
        self.feedrate = 1000 # default but can change

        # Coordinate system
        self.work_offset = (0.0, 0.0, 0.0)  # G54 only for now

        # Current position (machine space)
        self.position = [0.0, 0.0, 0.0]

        # Active motion mode
        self.motion_mode = None    # G0/G1/G2/G3

        #Arc tolerance
        self.arc_tolerance = 0.01  # mm chordal tolerance
        # tolerance is overridden in the test_file.py

    def set_units(self, gcode):
        if gcode == "G20":
            self.units = "inch"
            self.units_scale = 25.4
        elif gcode == "G21":
            self.units = "mm"
            self.units_scale = 1.0

    def set_plane(self, gcode):
        if gcode == "G17":
            self.plane = Plane.XY
        elif gcode == "G18":
            self.plane = Plane.XZ
        elif gcode == "G19":
            self.plane = Plane.YZ


    def set_distance_mode(self, gcode):
        self.absolute = (gcode == "G90")

    def update_feedrate(self, f):
        self.feedrate = f * self.units_scale

    def resolve_target(self, target):
        """Convert G-code target to absolute machine coordinates"""
        resolved = list(self.position)
        for i, axis in enumerate(("X", "Y", "Z")):
            if axis in target:
                value = target[axis] * self.units_scale
                if self.absolute:
                    resolved[i] = value + self.work_offset[i]
                else:
                    resolved[i] += value
        return tuple(resolved)


class Plane:
    XY = "G17"
    XZ = "G18"
    YZ = "G19"
