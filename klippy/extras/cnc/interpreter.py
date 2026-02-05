# klippy/extras/cnc/interpreter.py

from primitives import MotionPrimitive, MotionType
from arc import segment_arc, compute_arc_center_from_r
from limits import SoftLimits, SoftLimitError


class CNCInterpreter:
    def __init__(self, modal_state, soft_limits=None):
        self.state = modal_state
        self.soft_limits = soft_limits

    def apply_work_offset(self, position):
        ox, oy, oz = self.state.work_offsets[self.state.active_wcs]
        return (
            position[0] + ox,
            position[1] + oy,
            position[2] + oz,
        )

    def interpret(self, parsed):
        """
        Interpret parsed G-code data and return motion primitives.

        parsed format:
        {
            "words": { "X": 1.0, "Y": 2.0, ... },
            "gcodes": [0, 1, ...],
            "mcodes": [3, 5, ...],
        }
        """
        if not parsed:
            return []

        import math

        primitives = []
        target = {}

        words = parsed["words"]
        gcodes = parsed["gcodes"]

        # --- Handle G-codes first (modal updates) ---
        for g in gcodes:
            self._handle_g(g)

        # --- Handle axis words / feedrate ---
        for letter, value in words.items():
            if letter in ("X", "Y", "Z", "I", "J", "K", "R"):
                target[letter] = value
            elif letter == "F":
                self.state.update_feedrate(value)

        # --- Generate motion primitives ---
            # Program-space positions
            start_prog = tuple(self.state.position)
            end_prog = self.state.resolve_target(target)

            # Apply work coordinate offset (program → machine space)
            start = self.apply_work_offset(start_prog)
            end = self.apply_work_offset(end_prog)

        # ---------------- ARC MOTION ----------------
            if self.state.motion_mode in (MotionType.ARC_CW, MotionType.ARC_CCW):
                plane = self.state.plane
                pos = start

                # Plane mapping
                if plane == "G17":  # XY
                    ax, ay, az = 0, 1, 2
                    ia, ib = "I", "J"
                elif plane == "G18":  # XZ
                    ax, ay, az = 0, 2, 1
                    ia, ib = "I", "K"
                elif plane == "G19":  # YZ
                    ax, ay, az = 1, 2, 0
                    ia, ib = "J", "K"
                else:
                    raise RuntimeError(f"Unsupported plane {plane}")

                a0, b0 = pos[ax], pos[ay]
                a1, b1 = end[ax], end[ay]

                # --- Determine arc center ---
                if "R" in target:
                    from arc import compute_arc_center_from_r
                    center = compute_arc_center_from_r(
                        start=(a0, b0),
                        end=(a1, b1),
                        r=target["R"] * self.state.units_scale,
                        clockwise=(self.state.motion_mode == MotionType.ARC_CW),
                    )
                else:
                    ia_val = target.get(ia, 0.0) * self.state.units_scale
                    ib_val = target.get(ib, 0.0) * self.state.units_scale
                    center = (a0 + ia_val, b0 + ib_val)

                # --- Segment arc in plane ---
                points = segment_arc(
                    start=(a0, b0),
                    end=(a1, b1),
                    center=center,
                    clockwise=(self.state.motion_mode == MotionType.ARC_CW),
                    tolerance=self.state.arc_tolerance,
                )

                # --- Compute total arc length (XY + helix) ---
                xy_len = 0.0
                for i in range(len(points) - 1):
                    dx = points[i + 1][0] - points[i][0]
                    dy = points[i + 1][1] - points[i][1]
                    xy_len += math.hypot(dx, dy)

                z0 = pos[az]
                z1 = end[az]
                dz_total = z1 - z0

                total_len = math.hypot(xy_len, dz_total)

                prev = start
                traveled_xy = 0.0

                for i, (a, b) in enumerate(points):
                    next_pt = list(prev)
                    next_pt[ax] = a
                    next_pt[ay] = b

                    # --- Helical Z interpolation ---
                    if xy_len > 0:
                        if i > 0:
                            dx = a - points[i - 1][0]
                            dy = b - points[i - 1][1]
                            traveled_xy += math.hypot(dx, dy)
                        frac = traveled_xy / xy_len
                    else:
                        frac = 1.0

                    next_pt[az] = z0 + dz_total * frac

                    # --- Segment length ---
                    seg_len = math.dist(prev, next_pt)

                    # --- Arc feedrate correction ---
                    seg_feed = self.state.feedrate

                    primitives.append(
                        MotionPrimitive(
                            motion=MotionType.LINEAR,
                            start=prev,
                            end=tuple(next_pt),
                            feedrate=seg_feed,
                        )
                    )

                    prev = tuple(next_pt)

            # ---------------- LINEAR / RAPID ----------------
            else:
                primitives.append(
                    MotionPrimitive(
                        motion=self.state.motion_mode,
                        start=start,
                        end=end,
                        feedrate=self.state.feedrate,
                    )
                )

            self.state.position = list(end)

        # --- Soft limit validation ---
        validated = []
        for p in primitives:
            if self.soft_limits:
                self.soft_limits.check_primitive(p)
            validated.append(p)

        return validated

    def _handle_g(self, gcode):
        if gcode in (0, 1):
            self.state.motion_mode = (
                MotionType.RAPID if gcode == 0 else MotionType.LINEAR
            )

        elif gcode in (2, 3):
            self.state.motion_mode = (
                MotionType.ARC_CW if gcode == 2 else MotionType.ARC_CCW
            )

        elif gcode in (90, 91):
            self.state.set_distance_mode(f"G{gcode}")

        elif gcode in (20, 21):
            self.state.set_units(f"G{gcode}")

        elif gcode in (17, 18, 19):
            self.state.set_plane(f"G{gcode}")
