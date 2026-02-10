# klippy/extras/cnc/planner.py

from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple
from collections import deque
import math

from primitives import MotionPrimitive, MotionType

EPS = 1e-12


@dataclass
class PlannedPrimitive:
    """
    Planned wrapper around a MotionPrimitive.

    Speeds are in mm/s.
    Timing is optional but useful for ETA/prescan later.
    """
    primitive: MotionPrimitive
    v_entry: float
    v_exit: float
    v_peak: float
    accel: float
    t_accel: float
    t_cruise: float
    t_decel: float


@dataclass
class _MoveInfo:
    p: MotionPrimitive
    L: float                    # segment length (mm)
    u: Tuple[float, float, float]  # unit direction vector
    vmax: float                 # max allowed speed on this segment (mm/s)
    accel: float                # accel along path (mm/s^2)
    min_time: float             # optimistic time at vmax (s)
    delta_v2: float             # 2 * L * accel


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _unit_vec(p: MotionPrimitive) -> Tuple[float, float, float]:
    dx = p.end[0] - p.start[0]
    dy = p.end[1] - p.start[1]
    dz = p.end[2] - p.start[2]
    L = math.sqrt(dx*dx + dy*dy + dz*dz)
    if L < EPS:
        return (0.0, 0.0, 0.0)
    return (dx / L, dy / L, dz / L)


def _effective_path_accel(
    u: Tuple[float, float, float],
    default_accel: float,
    axis_accels: Optional[Tuple[float, float, float]] = None,
) -> float:
    """
    CNC-friendly: if you have per-axis accel limits, limit path accel so no axis exceeds its limit.
    Otherwise, use default_accel.
    """
    if axis_accels is None:
        return default_accel

    ax, ay, az = axis_accels
    ux, uy, uz = abs(u[0]), abs(u[1]), abs(u[2])
    candidates = []

    if ux > EPS:
        candidates.append(ax / ux)
    if uy > EPS:
        candidates.append(ay / uy)
    if uz > EPS:
        candidates.append(az / uz)

    if not candidates:
        return default_accel
    return min(candidates)


def _junction_v2(prev: _MoveInfo, cur: _MoveInfo, jd: float, accel: float) -> float:
    """
    Klipper-style JD junction speed limit between two linear segments.
    Returns v^2 (mm^2/s^2).
    """
    u1 = prev.u
    u2 = cur.u
    dot = _clamp(u1[0]*u2[0] + u1[1]*u2[1] + u1[2]*u2[2], -1.0, 1.0)

    # Klipper uses supplementary angle: junction_cos_theta = -dot
    junction_cos_theta = -dot

    sin_theta_d2 = math.sqrt(max(0.5 * (1.0 - junction_cos_theta), 0.0))
    cos_theta_d2 = math.sqrt(max(0.5 * (1.0 + junction_cos_theta), 0.0))

    one_minus_sin = 1.0 - sin_theta_d2
    if one_minus_sin <= EPS or cos_theta_d2 <= EPS:
        # Nearly straight / degenerate; no JD-based limit
        return float("inf")

    # Radius factor
    R = sin_theta_d2 / one_minus_sin

    # Core JD constraint (Klipper-style):
    # v^2 <= accel * (jd * R)
    v2_jd = accel * (jd * R)

    # Optional extra constraint: don't allow an implied blend that extends beyond short segments
    quarter_tan = 0.25 * sin_theta_d2 / cos_theta_d2
    v2_cent_cur = cur.delta_v2 * quarter_tan
    v2_cent_prev = prev.delta_v2 * quarter_tan

    return min(v2_jd, v2_cent_cur, v2_cent_prev)


def _plan_window(
    moves: List[_MoveInfo],
    *,
    junction_deviation: float,
    start_v2: float,
    stop_at_end: bool,
) -> List[PlannedPrimitive]:
    """
    Plan a list of moves with:
    - JD junction caps
    - accel reachability (backward + forward pass in v^2 space)
    - trapezoid/triangle timing per segment

    start_v2 is the known v^2 entering moves[0].
    stop_at_end forces final boundary speed to 0.
    """
    n = len(moves)
    if n == 0:
        return []

    # Boundary caps (v^2) at each junction between moves
    cap2 = [float("inf")] * (n + 1)

    # Known carry-in speed cap at start
    cap2[0] = max(0.0, start_v2)

    # End condition
    cap2[n] = 0.0 if stop_at_end else float("inf")

    # Cap boundaries by per-move vmax as well
    for i in range(n):
        vmax2 = moves[i].vmax * moves[i].vmax
        cap2[i] = min(cap2[i], vmax2)
        cap2[i + 1] = min(cap2[i + 1], vmax2)

    # Junction caps (between i-1 and i) go into cap2[i]
    for i in range(1, n):
        prev = moves[i - 1]
        cur = moves[i]
        a_junc = min(prev.accel, cur.accel)

        v2 = _junction_v2(prev, cur, junction_deviation, a_junc)
        v2 = min(v2, prev.vmax * prev.vmax, cur.vmax * cur.vmax)

        cap2[i] = min(cap2[i], v2)

    # Start with caps
    v2b = cap2[:]

    # Backward pass: ensure we can decelerate from boundary i to i+1 across move i
    # Skip i=0: we treat cap2[0] as given carry-in cap and do not increase it.
    for i in reversed(range(1, n)):
        a = moves[i].accel
        L = moves[i].L
        reachable2 = v2b[i + 1] + 2.0 * a * L
        v2b[i] = min(v2b[i], reachable2)

    # Move 0 also must be able to decelerate to boundary 1, but we can only reduce boundary 1,
    # not increase boundary 0 beyond its cap.
    # (Forward pass below will handle limiting boundary 1 based on boundary 0.)

    # Forward pass: ensure we can accelerate from boundary i to i+1 across move i
    for i in range(n):
        a = moves[i].accel
        L = moves[i].L
        reachable2 = v2b[i] + 2.0 * a * L
        v2b[i + 1] = min(v2b[i + 1], reachable2)

    # Build per-move trapezoids
    planned: List[PlannedPrimitive] = []
    for i in range(n):
        m = moves[i]
        L = m.L
        a = m.accel

        v_in2 = max(0.0, v2b[i])
        v_out2 = max(0.0, v2b[i + 1])
        vmax2 = m.vmax * m.vmax

        # Max reachable peak v^2 with accel constraints on both ends:
        # v_peak^2 <= a*L + 0.5*(v_in^2 + v_out^2)
        v_peak2 = min(vmax2, a * L + 0.5 * (v_in2 + v_out2))
        v_in = math.sqrt(v_in2)
        v_out = math.sqrt(v_out2)
        v_peak = math.sqrt(max(0.0, v_peak2))

        if a <= EPS:
            # Degenerate (shouldn't happen with real accel), treat as constant speed
            t_accel = t_decel = 0.0
            t_cruise = (L / v_peak) if v_peak > EPS else 0.0
            planned.append(PlannedPrimitive(
                primitive=m.p, v_entry=v_in, v_exit=v_out, v_peak=v_peak,
                accel=a, t_accel=t_accel, t_cruise=t_cruise, t_decel=t_decel
            ))
            continue

        # Distances for accel/decel to/from peak
        d_accel = (v_peak2 - v_in2) / (2.0 * a)
        d_decel = (v_peak2 - v_out2) / (2.0 * a)
        d_cruise = max(0.0, L - d_accel - d_decel)

        t_accel = (v_peak - v_in) / a
        t_decel = (v_peak - v_out) / a
        t_cruise = (d_cruise / v_peak) if v_peak > EPS else 0.0

        planned.append(PlannedPrimitive(
            primitive=m.p,
            v_entry=v_in,
            v_exit=v_out,
            v_peak=v_peak,
            accel=a,
            t_accel=t_accel,
            t_cruise=t_cruise,
            t_decel=t_decel,
        ))

    return planned


class CNCPlanner:
    """
    Streaming CNC planner:
    - Accepts MotionPrimitive objects incrementally
    - Maintains a raw lookahead window
    - Periodically replans the window and emits a committed prefix as PlannedPrimitive objects
    - Keeps a tail to preserve lookahead quality and maintain continuity

    Notes:
    - This planner plans *toolpath speed*; it does not split geometry further.
    - Your current Klipper executor ignores v_entry/v_exit/v_peak (Klipper will do its own planning).
      The structure here is still valuable for:
        * a non-Klipper backend
        * accurate accel-aware ETA / prescan
        * future integration where executor consumes these values
    """

    def __init__(
        self,
        *,
        max_velocity: float,             # mm/s
        max_accel: float,                # mm/s^2
        junction_deviation: float,        # mm
        buffer_time: float = 0.250,       # seconds of optimistic motion to keep in raw window
        keep_tail_moves: int = 2,         # minimum tail moves to keep uncommitted
        max_window_moves: int = 200,      # hard cap to prevent runaway buffering
        axis_accels: Optional[Tuple[float, float, float]] = None,
    ):
        self.max_velocity = float(max_velocity)
        self.max_accel = float(max_accel)
        self.junction_deviation = float(junction_deviation)

        self.buffer_time = float(buffer_time)
        self.keep_tail_moves = int(max(1, keep_tail_moves))
        self.max_window_moves = int(max(10, max_window_moves))

        self.axis_accels = axis_accels

        self._window: Deque[_MoveInfo] = deque()
        self._window_time = 0.0  # sum(min_time) of moves in window
        self._carry_in_v2 = 0.0  # v^2 entering first move of window

    def reset(self) -> None:
        self._window.clear()
        self._window_time = 0.0
        self._carry_in_v2 = 0.0

    def push(self, prim: MotionPrimitive) -> List[PlannedPrimitive]:
        """
        Add a raw MotionPrimitive. May return 0+ PlannedPrimitives that are committed (safe to send).
        """
        mi = self._make_moveinfo(prim)
        if mi is None:
            return []

        self._window.append(mi)
        self._window_time += mi.min_time

        # If window grows too large, force a flush (commit more aggressively)
        force = len(self._window) >= self.max_window_moves

        return self._flush_if_ready(force=force)

    def finish(self) -> List[PlannedPrimitive]:
        """
        End-of-stream: plan remaining moves and force final stop (v_exit=0).
        """
        if not self._window:
            return []

        planned = _plan_window(
            list(self._window),
            junction_deviation=self.junction_deviation,
            start_v2=self._carry_in_v2,
            stop_at_end=True,
        )
        self.reset()
        return planned

    # -------------------------
    # Internals
    # -------------------------

    def _make_moveinfo(self, prim: MotionPrimitive) -> Optional[_MoveInfo]:
        L = prim.length()
        if L < EPS:
            return None

        # Determine segment speed limit (mm/s)
        # feedrate in your codebase is mm/min
        if prim.motion == MotionType.RAPID:
            v_cmd = self.max_velocity
        else:
            if prim.feedrate is None or prim.feedrate <= 0.0:
                v_cmd = self.max_velocity
            else:
                v_cmd = min(prim.feedrate / 60.0, self.max_velocity)

        u = _unit_vec(prim)
        a = _effective_path_accel(u, self.max_accel, self.axis_accels)

        min_time = (L / v_cmd) if v_cmd > EPS else 0.0
        delta_v2 = 2.0 * L * a

        return _MoveInfo(
            p=prim,
            L=L,
            u=u,
            vmax=v_cmd,
            accel=a,
            min_time=min_time,
            delta_v2=delta_v2,
        )

    def _flush_if_ready(self, *, force: bool) -> List[PlannedPrimitive]:
        """
        Decide whether to plan+commit a prefix of the window.

        Strategy (MVP):
        - Keep at least `buffer_time` seconds of optimistic motion + `keep_tail_moves` in the window
        - When we have "enough" buffered (or forced), plan the whole window with stop_at_end=True
          and commit the oldest prefix so the remaining tail still has lookahead.

        Carry-in continuity:
        - After committing K moves, we set carry-in speed for the new first move to the planned
          v_entry of that move.
        """
        if len(self._window) <= self.keep_tail_moves:
            return []

        if not force and self._window_time < self.buffer_time:
            return []

        # Plan the full current window (force a stop at end of *window*)
        # The intent is: keep enough tail so that the forced stop lives in the tail and doesn't
        # pollute earlier motion.
        planned_all = _plan_window(
            list(self._window),
            junction_deviation=self.junction_deviation,
            start_v2=self._carry_in_v2,
            stop_at_end=True,
        )

        # Choose how many moves to commit:
        # Commit from the front until remaining_time <= buffer_time,
        # BUT always keep at least keep_tail_moves uncommitted.
        remaining_time = self._window_time
        flush_count = 0

        # We'll keep at least keep_tail_moves in the window no matter what
        max_flush = len(self._window) - self.keep_tail_moves

        while flush_count < max_flush:
            mi = self._window[flush_count]
            next_remaining = remaining_time - mi.min_time

            if not force and next_remaining < self.buffer_time:
                break

            flush_count += 1
            remaining_time = next_remaining

        if flush_count <= 0:
            return []

        committed = planned_all[:flush_count]

        # Update carry-in for the new head of window (if any)
        if flush_count < len(planned_all):
            new_head = planned_all[flush_count]
            self._carry_in_v2 = new_head.v_entry * new_head.v_entry
        else:
            self._carry_in_v2 = 0.0

        # Pop flushed moves from raw window and update time
        for _ in range(flush_count):
            mi0 = self._window.popleft()
            self._window_time -= mi0.min_time

        # Numerical safety
        if self._window_time < 0.0:
            self._window_time = 0.0

        return committed
