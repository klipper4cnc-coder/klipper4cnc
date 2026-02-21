# klippy/extras/cnc/interpreter.py

import math

try:
    from .primitives import MotionPrimitive, MotionType
    from .arc import segment_arc, compute_arc_center_from_r
    from .linear import segment_linear
    from .limits import SoftLimitError
except ImportError:
    from primitives import MotionPrimitive, MotionType
    from arc import segment_arc, compute_arc_center_from_r
    from linear import segment_linear
    from limits import SoftLimitError


class CNCInterpreter:
    """
    Convert parsed G-code dict -> list[MotionPrimitive], while updating modal state.
    """

    def __init__(self, modal_state, soft_limits=None):
        self.state = modal_state
        self.soft_limits = soft_limits

    def apply_work_offset(self, position_mm):
        ox, oy, oz = self.state.work_offsets[self.state.active_wcs]
        return (position_mm[0] + ox, position_mm[1] + oy, position_mm[2] + oz)

    def interpret(self, parsed):
        if not parsed:
            return []

        words = parsed.get("words", {})
        gcodes = parsed.get("gcodes", [])

        # Modal updates first
        for g in gcodes:
            self._handle_g(g)

        # Feedrate update
        if "F" in words:
            self.state.update_feedrate(words["F"])

        # Collect motion words
        target = {}
        for k in ("X", "Y", "Z", "I", "J", "K", "R"):
            if k in words:
                target[k] = words[k]

        motion = self.state.motion_mode

        # If no XYZ target and not arc geometry, nothing to do
        if not any(k in target for k in ("X", "Y", "Z")) and motion not in (MotionType.ARC_CW, MotionType.ARC_CCW):
            return []

        # Resolve program-space endpoints (mm) and update program position
        start_prog = tuple(self.state.position)
        end_prog = tuple(self.state.resolve_target(target))
        self.state.position = list(end_prog)

        # Convert to machine-space (apply WCS)
        start = self.apply_work_offset(start_prog)
        end = self.apply_work_offset(end_prog)

        # Optional: bounds check final endpoint
        if self.soft_limits is not None:
            self.soft_limits.check_point(end)  # may raise SoftLimitError

        # Dispatch motion
        if motion in (MotionType.ARC_CW, MotionType.ARC_CCW):
            return self._interp_arc(motion, start_prog, end_prog, target)
        else:
            return self._interp_linear(motion, start, end)

    def _interp_linear(self, motion, start, end):
        # Choose feedrate
        if motion == MotionType.RAPID:
            feed = self.state.rapid_feedrate
        else:
            feed = self.state.feedrate
            if feed is None:
                raise ValueError("Feedrate not set (missing F...) for G1/G2/G3 motion")

        segs = segment_linear(start, end, feed, self.state.max_segment_time)
        prims = []
        for s0, s1 in segs:
            prims.append(MotionPrimitive(motion, tuple(s0), tuple(s1), feed))
        return prims

    def _interp_arc(self, motion, start_prog, end_prog, target):
        clockwise = (motion == MotionType.ARC_CW)
        plane = self.state.plane

        # Plane axis mapping: (a,b) is arc plane, c is perpendicular axis
        if plane == "G17":      # XY, Z perpendicular
            ax, ay, az = 0, 1, 2
        elif plane == "G18":    # XZ, Y perpendicular
            ax, ay, az = 0, 2, 1
        elif plane == "G19":    # YZ, X perpendicular
            ax, ay, az = 1, 2, 0
        else:
            raise ValueError(f"Unsupported plane: {plane}")

        s2 = (start_prog[ax], start_prog[ay])
        e2 = (end_prog[ax], end_prog[ay])

        # Center from R or IJK
        scale = self.state.units_scale
        if "R" in target:
            r = float(target["R"]) * scale
            center = compute_arc_center_from_r(s2, e2, r, clockwise)
        else:
            # IJK are offsets in the fixed XYZ axes, regardless of plane
            i = float(target.get("I", 0.0)) * scale
            j = float(target.get("J", 0.0)) * scale
            k = float(target.get("K", 0.0)) * scale
            if plane == "G17":
                center = (s2[0] + i, s2[1] + j)
            elif plane == "G18":
                center = (s2[0] + i, s2[1] + k)
            else:  # G19
                center = (s2[0] + j, s2[1] + k)

        points = segment_arc(
            start=s2,
            end=e2,
            center=center,
            clockwise=clockwise,
            tolerance=self.state.arc_tolerance,
        )

        # Helix interpolation on perpendicular axis
        delta_perp = end_prog[az] - start_prog[az]

        # Total XY/XZ/YZ length for proper helix fraction
        total_len = 0.0
        prev2 = s2
        for p in points:
            total_len += math.hypot(p[0] - prev2[0], p[1] - prev2[1])
            prev2 = p
        if total_len <= 1e-12:
            return []

        prims = []
        prev_prog = list(start_prog)
        prev_machine = list(self.apply_work_offset(prev_prog))

        traveled = 0.0
        prev2 = s2
        for p in points:
            seg_len = math.hypot(p[0] - prev2[0], p[1] - prev2[1])
            traveled += seg_len
            frac = min(1.0, traveled / total_len)

            next_prog = list(prev_prog)
            next_prog[ax] = p[0]
            next_prog[ay] = p[1]
            next_prog[az] = start_prog[az] + delta_perp * frac

            next_machine = list(self.apply_work_offset(next_prog))

            # For segmented arcs, emit linear primitives (your planner handles smoothing)
            feed = self.state.feedrate
            if feed is None:
                raise ValueError("Feedrate not set (missing F...) for G2/G3 motion")
            prims.append(MotionPrimitive(MotionType.LINEAR, tuple(prev_machine), tuple(next_machine), feed))

            prev_prog = next_prog
            prev_machine = next_machine
            prev2 = p

        return prims

    def _handle_g(self, gcode_num):
        # Motion modes
        if gcode_num == 0:
            self.state.set_motion_mode(MotionType.RAPID)
        elif gcode_num == 1:
            self.state.set_motion_mode(MotionType.LINEAR)
        elif gcode_num == 2:
            self.state.set_motion_mode(MotionType.ARC_CW)
        elif gcode_num == 3:
            self.state.set_motion_mode(MotionType.ARC_CCW)

        # Distance mode
        elif gcode_num == 90:
            self.state.set_distance_mode("G90")
        elif gcode_num == 91:
            self.state.set_distance_mode("G91")

        # Units
        elif gcode_num == 20:
            self.state.set_units("G20")
        elif gcode_num == 21:
            self.state.set_units("G21")

        # Plane
        elif gcode_num in (17, 18, 19):
            self.state.set_plane(f"G{gcode_num}")

        # WCS selection (G54..G59)
        elif 54 <= gcode_num <= 59:
            self.state.active_wcs = gcode_num - 54