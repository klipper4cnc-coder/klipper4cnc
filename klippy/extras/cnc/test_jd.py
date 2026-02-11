#!/usr/bin/env python3
# klippy/extras/cnc/test_jd.py
#
# Run from repo root:
#   python3 klippy/extras/cnc/test_jd.py
#
# This is an ad-hoc functional test runner (no pytest dependency).

import os
import sys
import math
import tempfile
from dataclasses import dataclass

# Ensure local (non-package) imports inside klippy/extras/cnc work
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

from primitives import MotionPrimitive, MotionType
from planner import CNCPlanner
from controller import CNCController
from streamer import GCodeStreamer


# -------------------------
# Simple assert helpers
# -------------------------

def assert_true(cond, msg):
    if not cond:
        raise AssertionError(msg)

def assert_almost(a, b, tol=1e-3, msg=""):
    if abs(a - b) > tol:
        raise AssertionError(msg or f"Expected {a} ~= {b} (tol={tol})")

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# -------------------------
# Expected JD math (matches planner.py)
# -------------------------

def expected_junction_speed(u1, u2, jd_mm, accel_mm_s2, L1, L2, vmax):
    """
    Compute expected junction speed cap for 2 connected segments using the same
    Klipper-style JD model implemented in planner.py.

    Returns v_junc (mm/s).
    """
    dot = clamp(u1[0]*u2[0] + u1[1]*u2[1] + u1[2]*u2[2], -1.0, 1.0)
    junction_cos_theta = -dot
    sin_theta_d2 = math.sqrt(max(0.5 * (1.0 - junction_cos_theta), 0.0))
    cos_theta_d2 = math.sqrt(max(0.5 * (1.0 + junction_cos_theta), 0.0))

    one_minus_sin = 1.0 - sin_theta_d2
    if one_minus_sin <= 1e-12 or cos_theta_d2 <= 1e-12:
        return vmax

    R = sin_theta_d2 / one_minus_sin
    v2_jd = accel_mm_s2 * (jd_mm * R)

    quarter_tan = 0.25 * sin_theta_d2 / cos_theta_d2
    delta_v2_1 = 2.0 * L1 * accel_mm_s2
    delta_v2_2 = 2.0 * L2 * accel_mm_s2
    v2_cent_1 = delta_v2_1 * quarter_tan
    v2_cent_2 = delta_v2_2 * quarter_tan

    v2 = min(v2_jd, v2_cent_1, v2_cent_2, vmax*vmax)
    return math.sqrt(max(0.0, v2))


# -------------------------
# Test primitives builders
# -------------------------

def make_linear(x0, y0, z0, x1, y1, z1, feed_mm_min):
    return MotionPrimitive(
        motion=MotionType.LINEAR,
        start=(x0, y0, z0),
        end=(x1, y1, z1),
        feedrate=feed_mm_min,
    )

def unit_vec(p):
    dx = p.end[0] - p.start[0]
    dy = p.end[1] - p.start[1]
    dz = p.end[2] - p.start[2]
    L = math.sqrt(dx*dx + dy*dy + dz*dz)
    if L < 1e-12:
        return (0.0, 0.0, 0.0), 0.0
    return (dx/L, dy/L, dz/L), L

def make_arc_polyline(center, radius, start_deg, end_deg, segments, feed_mm_min):
    """
    Build a list of linear segments approximating an arc.
    """
    cx, cy = center
    pts = []
    for i in range(segments + 1):
        t = i / segments
        ang = math.radians(start_deg + (end_deg - start_deg) * t)
        x = cx + radius * math.cos(ang)
        y = cy + radius * math.sin(ang)
        pts.append((x, y, 0.0))
    prims = []
    for i in range(segments):
        prims.append(MotionPrimitive(
            motion=MotionType.LINEAR,
            start=pts[i],
            end=pts[i+1],
            feedrate=feed_mm_min,
        ))
    return prims


# -------------------------
# Planner tests
# -------------------------

def test_planner_corner_jd_numeric():
    """
    Two connected segments with a 90-degree corner. High commanded feedrate.
    Expect JD to cap junction speed around ~11 mm/s for the chosen parameters.
    """
    feed_mm_min = 3000.0         # 50 mm/s
    vmax = 200.0                 # planner max velocity mm/s
    accel = 1000.0               # mm/s^2
    jd = 0.05                    # mm

    p1 = make_linear(0, 0, 0, 100, 0, 0, feed_mm_min)
    p2 = make_linear(100, 0, 0, 100, 100, 0, feed_mm_min)

    u1, L1 = unit_vec(p1)
    u2, L2 = unit_vec(p2)

    planner = CNCPlanner(
        max_velocity=vmax,
        max_accel=accel,
        junction_deviation=jd,
        buffer_time=10.0,        # keep everything in window for this unit test
        keep_tail_moves=1,
    )

    out = []
    out.extend(planner.push(p1))
    out.extend(planner.push(p2))
    out.extend(planner.finish())
    assert_true(len(out) == 2, f"Expected 2 planned moves, got {len(out)}")

    v_junc_expected = expected_junction_speed(u1, u2, jd, accel, L1, L2, vmax=min(vmax, feed_mm_min/60.0))
    v_junc_planned = out[0].v_exit

    # Loose tolerance because floating math, and planner can also be influenced by stop_at_end
    assert_almost(v_junc_planned, v_junc_expected, tol=0.25,
                  msg=f"Corner JD junction speed mismatch: planned={v_junc_planned:.3f}, expected~{v_junc_expected:.3f}")

def test_planner_arc_segments_stay_fast():
    """
    Approximate a quarter-circle with many small segments.
    With JD, internal junction speeds should remain near commanded feedrate
    (no stop-and-go at each segment).
    """
    feed_mm_min = 3000.0  # 50 mm/s
    feed = feed_mm_min / 60.0
    vmax = 200.0
    accel = 1000.0
    jd = 0.05

    prims = make_arc_polyline(center=(10.0, 0.0), radius=10.0,
                              start_deg=180.0, end_deg=90.0,
                              segments=24, feed_mm_min=feed_mm_min)

    planner = CNCPlanner(
        max_velocity=vmax,
        max_accel=accel,
        junction_deviation=jd,
        buffer_time=10.0,
        keep_tail_moves=1,
    )

    planned = []
    for p in prims:
        planned.extend(planner.push(p))
    planned.extend(planner.finish())

    assert_true(len(planned) == len(prims), "Planned count mismatch")

    # Ignore first few / last few because start from 0 and end at 0 (stop_at_end=True).
    mid = planned[5:-5]
    assert_true(len(mid) > 0, "Not enough segments for mid-window check")

    min_mid_exit = min(pp.v_exit for pp in mid)
    # Expect most junctions allow ~full feedrate
    assert_true(min_mid_exit > 0.90 * feed,
                f"Arc segment junction speeds sag too much: min_mid_exit={min_mid_exit:.3f} mm/s, feed={feed:.3f} mm/s")


# -------------------------
# Controller test (planned mode)
# -------------------------

class RecordingPlanner(CNCPlanner):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.committed = []
        self.finish_called = 0

    def push(self, prim):
        out = super().push(prim)
        self.committed.extend(out)
        return out

    def finish(self):
        out = super().finish()
        self.finish_called += 1
        self.committed.extend(out)
        return out



class RecordingExecutor:
    def __init__(self):
        self.executed = []
        self.last_feedrate = None

    def execute(self, p):
        self.executed.append(p)
        self.last_feedrate = p.feedrate

    def flush(self):
        pass

class FakeInterpreter:
    """
    ignore parsed gcode; return pre-canned primitives per line
    """
    def __init__(self, per_line_prims):
        self.per_line_prims = list(per_line_prims)
        self.i = 0

    def interpret(self, parsed):
        if parsed is None:
            return []
        if self.i >= len(self.per_line_prims):
            return []
        prims = self.per_line_prims[self.i]
        self.i += 1
        return prims

def test_controller_run_stream_planned():
    feed_mm_min = 3000.0
    vmax = 200.0
    accel = 1000.0
    jd = 0.05

    # 3 moves with a corner
    prims = [
        [make_linear(0, 0, 0, 50, 0, 0, feed_mm_min)],
        [make_linear(50, 0, 0, 50, 50, 0, feed_mm_min)],
        [make_linear(50, 50, 0, 100, 50, 0, feed_mm_min)],
    ]

    # Create a temp gcode file with 3 non-empty lines (parser expects something)
    fd, path = tempfile.mkstemp(prefix="jd_", suffix=".nc", text=True)
    os.close(fd)
    with open(path, "w") as f:
        f.write("G1 X1\n")
        f.write("G1 X2\n")
        f.write("G1 X3\n")

    streamer = GCodeStreamer(path)
    interp = FakeInterpreter(prims)

    planner = CNCPlanner(
        max_velocity=vmax,
        max_accel=accel,
        junction_deviation=jd,
        buffer_time=0.01,      # small to force commits quickly
        keep_tail_moves=1,
        max_window_moves=50,
    )
    execu = RecordingExecutor()
    ctrl = CNCController(execu, program=None, planner=planner)

    # total length for progress (optional)
    total_len = sum(p[0].length() for p in prims)
    ctrl.set_total_length(total_len)

    ctrl.start()
    ctrl.run_stream(streamer, interp)
    ctrl.flush()

    assert_true(len(execu.executed) == 3, f"Expected 3 executed primitives, got {len(execu.executed)}")
    assert_almost(ctrl.completed_length, total_len, tol=1e-6, msg="Controller completed_length mismatch")
    assert_true(ctrl.eof is True, "Controller should hit EOF")
    os.remove(path)


# -------------------------
# CNCMode smoke test (expected to FAIL until cnc_mode.py is updated for streaming)
# -------------------------

@dataclass
class FakeGCmd:
    params: dict
    def get(self, key, default=None):
        return self.params.get(key, default)
    def error(self, msg):
        raise RuntimeError(msg)
    def respond_info(self, msg):
        print("[GCMD INFO]", msg)
    def respond_raw(self, msg):
        print("[GCMD RAW]", msg)

class FakeGCode:
    def __init__(self):
        self.commands = {}
    def register_command(self, name, fn, desc=None):
        self.commands[name] = fn

class FakeToolhead:
    def __init__(self):
        self.moves = []
    def move(self, pos, speed):
        self.moves.append((tuple(pos), speed))
    def wait_moves(self):
        pass

class FakePrinter:
    def __init__(self):
        self._objs = {
            "gcode": FakeGCode(),
            "toolhead": FakeToolhead(),
        }
    def lookup_object(self, name, default=None):
        return self._objs.get(name, default)
    def get_reactor(self):
        return None

class FakeConfig:
    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer

    # Klipper-like helpers (return defaults for tests)
    def getfloat(self, key, default=None, **kwargs):
        return default

    def getint(self, key, default=None, **kwargs):
        return default


def test_cnc_mode_smoke():
    """
    This is a wiring test: cnc_mode should create controller in streaming mode,
    and CNC_START should run controller.run_stream on a file.

    CURRENT REPO STATUS: cnc_mode.py still references self.program / CNCProgram,
    so this test will FAIL until you update cnc_mode to streaming.
    """
    # Create a temp gcode file
    fd, path = tempfile.mkstemp(prefix="jd_mode_", suffix=".nc", text=True)
    os.close(fd)
    with open(path, "w") as f:
        f.write("G1 X1\n")
        f.write("G1 X2\n")

    printer = FakePrinter()
    config = FakeConfig(printer)

    try:
        from cnc_mode import CNCMode
        mode = CNCMode(config)
        assert_true(mode.controller.program is None, "cnc_mode should not use CNCProgram")
        assert_true(mode.controller.planner is not None, "cnc_mode should wire CNCPlanner into CNCController")

        # Replace planner with recording planner (forces early commits for a tiny job)
        orig = mode.planner
        rec = RecordingPlanner(
            max_velocity=orig.max_velocity,
            max_accel=orig.max_accel,
            junction_deviation=orig.junction_deviation,
            buffer_time=0.0,  # make it commit immediately once it can
            keep_tail_moves=1,  # keep a 1-move tail
            max_window_moves=orig.max_window_moves,
        )
        mode.planner = rec
        mode.controller.planner = rec

        # Reset so controller/planner buffers are clean and using the new instance
        mode.controller.reset()

    except Exception as e:
        raise AssertionError(
            "CNCMode instantiation failed. This is expected until cnc_mode.py "
            "is updated to remove CNCProgram and use streaming.\n"
            f"Error: {e}"
        )

    per_line = [
        [make_linear(0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 3000.0)],  # +X, 0.02mm
        [make_linear(0.02, 0.0, 0.0, 0.02, 0.02, 0.0, 3000.0)],  # +Y, 0.02mm (90° corner)
    ]

    mode.interpreter = FakeInterpreter(per_line)


    # Call CNC_START FILE=...
    gcmd = FakeGCmd({"FILE": path})

    # This should execute 2 moves through KlipperMotionExecutor -> FakeToolhead
    mode.cmd_start(gcmd)



    # New: verify planner actually planned and short-seg guard behaved
    assert_true(len(rec.committed) == 2, f"Expected 2 planned moves, got {len(rec.committed)}")
    assert_true(rec.finish_called == 1, "Expected finish() to be called once at EOF")

    pp0 = rec.committed[0]
    pp1 = rec.committed[1]

    # For a 90° corner, the short-segment cap from your planner math is:
    # v^2 = (2*L*a)*0.25  => v = sqrt(0.5*L*a)
    L0 = pp0.primitive.length()
    a0 = pp0.accel
    v_short = math.sqrt(0.5 * L0 * a0)

    # Also compute the JD cap (using your helper) and take the minimum
    u1, L1 = unit_vec(pp0.primitive)
    u2, L2 = unit_vec(pp1.primitive)
    feed = 3000.0 / 60.0
    vmax = min(rec.max_velocity, feed)
    v_jd = expected_junction_speed(u1, u2, rec.junction_deviation, a0, L1, L2, vmax)

    expected = min(v_short, v_jd)
    assert_almost(pp0.v_exit, expected, tol=0.30,
                  msg=f"Expected corner v_exit≈{expected:.3f} (short={v_short:.3f}, jd={v_jd:.3f}), got {pp0.v_exit:.3f}")

    toolhead = printer.lookup_object("toolhead")
    assert_true(len(toolhead.moves) == 2, f"Expected 2 toolhead moves, got {len(toolhead.moves)}")
    os.remove(path)


def test_planner_near_straight_keeps_speed():
    """
    Two long segments with a tiny angle between them should *not* slow down much.
    This is the core “segmented arc stays smooth” property.
    """
    feed_mm_min = 3000.0
    feed = feed_mm_min / 60.0
    vmax = 200.0
    accel = 1000.0
    jd = 0.05

    # Segment 1: +X
    p1 = make_linear(0, 0, 0, 100, 0, 0, feed_mm_min)

    # Segment 2: small angle (2 degrees) from +X
    ang = math.radians(2.0)
    x2 = 100.0 + 100.0 * math.cos(ang)
    y2 = 0.0 + 100.0 * math.sin(ang)
    p2 = make_linear(100, 0, 0, x2, y2, 0, feed_mm_min)

    u1, L1 = unit_vec(p1)
    u2, L2 = unit_vec(p2)

    planner = CNCPlanner(
        max_velocity=vmax,
        max_accel=accel,
        junction_deviation=jd,
        buffer_time=10.0,
        keep_tail_moves=1,
    )

    out = []
    out.extend(planner.push(p1))
    out.extend(planner.push(p2))
    out.extend(planner.finish())
    assert_true(len(out) == 2, f"Expected 2 planned moves, got {len(out)}")

    v_expected = expected_junction_speed(u1, u2, jd, accel, L1, L2, vmax=min(vmax, feed))
    v_planned = out[0].v_exit

    # Should be very close to full feed for such a tiny angle.
    assert_true(v_planned > 0.95 * feed,
                f"Near-straight junction slowed too much: v_exit={v_planned:.3f} feed={feed:.3f}")

    # And should match the expected math fairly closely.
    assert_almost(v_planned, v_expected, tol=0.75,
                  msg=f"Near-straight v_exit mismatch: planned={v_planned:.3f}, expected~{v_expected:.3f}")


def test_planner_reversal_slows_to_zero():
    """
    180° reversal should force junction speed ~0 (can't blend a cusp at speed).
    """
    feed_mm_min = 3000.0
    vmax = 200.0
    accel = 1000.0
    jd = 0.05

    # Segment 1: +X
    p1 = make_linear(0, 0, 0, 100, 0, 0, feed_mm_min)
    # Segment 2: -X (reverse direction)
    p2 = make_linear(100, 0, 0, 0, 0, 0, feed_mm_min)

    planner = CNCPlanner(
        max_velocity=vmax,
        max_accel=accel,
        junction_deviation=jd,
        buffer_time=10.0,
        keep_tail_moves=1,
    )

    out = []
    out.extend(planner.push(p1))
    out.extend(planner.push(p2))
    out.extend(planner.finish())
    assert_true(len(out) == 2, f"Expected 2 planned moves, got {len(out)}")

    v_planned = out[0].v_exit
    assert_true(v_planned < 1e-3,
                f"Expected reversal junction speed near zero, got {v_planned:.6f} mm/s")


# -------------------------
# Main runner
# -------------------------

def main():
    tests = [
        ("planner corner JD numeric", test_planner_corner_jd_numeric),
        ("planner arc segments stay fast", test_planner_arc_segments_stay_fast),
        ("planner near-straight keeps speed", test_planner_near_straight_keeps_speed),
        ("planner reversal slows to zero", test_planner_reversal_slows_to_zero),
        ("controller run_stream planned", test_controller_run_stream_planned),
        ("cnc_mode smoke", test_cnc_mode_smoke),
    ]

    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"[PASS] {name}")
        except Exception as e:
            failed += 1
            print(f"[FAIL] {name}: {e}")

    if failed:
        print(f"\n{failed} test(s) failed.")
        sys.exit(1)

    print("\nAll tests passed.")
    sys.exit(0)

if __name__ == "__main__":
    main()
