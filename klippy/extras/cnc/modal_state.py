# klippy/extras/cnc/modal_state.py


class CNCModalState:
    """
    Holds modal CNC state across G-code lines.

    Modal state persists until explicitly changed by a G-code command.
    This includes units, distance mode, plane selection, feedrate,
    coordinate systems, and current position.
    """

    def __init__(self):
        # -------------------------
        # Units
        # -------------------------

        # Active units ("mm" or "inch")
        self.units = "mm"          # G21 / G20

        # Scale factor to convert program units to millimeters
        self.units_scale = 1.0     # mm

        # -------------------------
        # Distance mode
        # -------------------------

        # Absolute (G90) or incremental (G91) positioning
        self.absolute = True       # G90 / G91

        # -------------------------
        # Motion plane
        # -------------------------

        # Active motion plane for arc commands
        # Default is XY plane per CNC spec
#        self.plane = "XY"          # G17 (XY only for now)
        self.plane = Plane.XY      # default per spec

        # -------------------------
        # Feedrate
        # -------------------------

        # Active feedrate in mm/min
        self.feedrate = 1000       # default but can change

        # -------------------------
        # Work coordinate systems
        # -------------------------

        # Active work coordinate system (G54–G59)
        self.active_wcs = 'G54'

        # Offsets for each work coordinate system
        self.work_offsets = {
            'G54': (0.0, 0.0, 0.0),
            'G55': (0.0, 0.0, 0.0),
            'G56': (0.0, 0.0, 0.0),
            'G57': (0.0, 0.0, 0.0),
            'G58': (0.0, 0.0, 0.0),
            'G59': (0.0, 0.0, 0.0),
        }

        # -------------------------
        # Position tracking
        # -------------------------

        # Current machine-space position
        self.position = [0.0, 0.0, 0.0]

        # -------------------------
        # Motion mode
        # -------------------------

        # Active motion mode (G0 / G1 / G2 / G3)
        self.motion_mode = None

        # -------------------------
        # Arc settings
        # -------------------------

        # Maximum allowed chordal deviation when segmenting arcs
        self.arc_tolerance = 0.01  # mm

        # NOTE: tolerance is overridden in test_file.py

    def set_units(self, gcode):
        """
        Set active units based on G-code.
        """
        if gcode == "G20":
            self.units = "inch"
            self.units_scale = 25.4
        elif gcode == "G21":
            self.units = "mm"
            self.units_scale = 1.0

    def set_plane(self, gcode):
        """
        Set active arc plane.
        """
        if gcode == "G17":
            self.plane = Plane.XY
        elif gcode == "G18":
            self.plane = Plane.XZ
        elif gcode == "G19":
            self.plane = Plane.YZ

    def set_distance_mode(self, gcode):
        """
        Set distance mode (absolute or incremental).
        """
        self.absolute = (gcode == "G90")

    def update_feedrate(self, f):
        """
        Update feedrate, applying unit scaling.
        """
        self.feedrate = f * self.units_scale

    def resolve_target(self, target):
        """
        Convert a G-code target dictionary into absolute coordinates.

        Takes into account:
        - Distance mode (absolute vs incremental)
        - Active unit scaling
        - Current position

        :param target: dict like {"X": 10, "Y": 5}
        :return: Absolute (x, y, z) position tuple
        """
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
    """
    Enumeration of CNC motion planes.
    """
    XY = "G17"
    XZ = "G18"
    YZ = "G19"
