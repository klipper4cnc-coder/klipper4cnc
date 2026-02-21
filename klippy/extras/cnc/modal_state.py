# klippy/extras/cnc/modal_state.py

try:
    from .primitives import MotionType
except ImportError:
    from primitives import MotionType


class CNCModalState:
    """
    CNC modal state across G-code lines.
    Stores *program-space* XYZ in mm (units scaling applied on input).
    Work offsets are applied separately when generating machine-space primitives.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        # Program-space position (mm), without WCS applied
        self.position = [0.0, 0.0, 0.0]

        # G90/G91
        self.absolute = True

        # Current feedrate (mm/min). For G1/G2/G3 this must be set.
        self.feedrate = None

        # G0/G1/G2/G3
        self.motion_mode = MotionType.RAPID

        # G17/G18/G19 stored as string (interpreter compares strings)
        self.plane = "G17"

        # G20/G21
        self.units = "mm"
        self.units_scale = 1.0  # multiply incoming words by this to get mm

        # Work coordinate system selection (G54..G59 -> 0..5)
        self.active_wcs = 0
        self.work_offsets = [(0.0, 0.0, 0.0)] * 6

        # Arc + segmentation knobs (in mm / seconds)
        self.arc_tolerance = 0.025
        self.max_arc_segment_length = 1.0
        self.max_core_segment_length = 2.0
        self.max_segment_time = 0.01

        # A safe default rapid rate if none provided elsewhere (mm/min)
        self.rapid_feedrate = 3000.0

    def set_distance_mode(self, gcode):
        if gcode == "G90":
            self.absolute = True
        elif gcode == "G91":
            self.absolute = False

    def set_units(self, gcode):
        if gcode == "G21":
            self.units = "mm"
            self.units_scale = 1.0
        elif gcode == "G20":
            self.units = "in"
            self.units_scale = 25.4

    def set_plane(self, gcode):
        if gcode in ("G17", "G18", "G19"):
            self.plane = gcode

    def set_motion_mode(self, motion_mode):
        self.motion_mode = motion_mode

    def update_feedrate(self, f):
        # G-code feedrate is in current units/min. Convert to mm/min.
        if f is None:
            return
        self.feedrate = float(f) * self.units_scale

    def resolve_target(self, target_words):
        """
        Resolve X/Y/Z targets in program space (mm).
        Applies units scaling and absolute/incremental mode.
        Does NOT apply WCS offsets.
        """
        resolved = list(self.position)
        for axis, idx in (("X", 0), ("Y", 1), ("Z", 2)):
            if axis not in target_words:
                continue
            val_mm = float(target_words[axis]) * self.units_scale
            if self.absolute:
                resolved[idx] = val_mm
            else:
                resolved[idx] += val_mm
        return resolved