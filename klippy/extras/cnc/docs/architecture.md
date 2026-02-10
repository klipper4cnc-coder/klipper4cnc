# Klipper CNC Architecture

This document describes the architecture of the CNC subsystem located in
klippy/extras/cnc.

The goal of this subsystem is to execute CNC-style G-code (G0/G1/G2/G3, modal
state, work offsets, planes, etc.) on top of Klipper’s motion system, while
keeping responsibilities cleanly separated and the execution model explicit.

This is not standard Klipper behavior. It is a self-contained CNC execution
pipeline layered on top of Klipper.

---

## High-level overview

At a high level, CNC execution follows this flow:

G-code source
  -> parser.py          (syntax -> structured words)
  -> interpreter.py     (modal state -> motion primitives)
  -> controller.py      (buffering, execution, progress, control)
  -> executor.py        (backend abstraction)
  -> klipper_executor.py (Klipper toolhead moves)

Two execution modes exist:

- Streaming mode (primary)
- Preloaded program mode (experimental / legacy)

---

## Core concepts

### Motion primitives

The lowest-level unit of motion in the CNC system is a MotionPrimitive
(primitives.py).

A primitive represents one fully-resolved motion segment:

- Absolute machine-space start and end positions
- Motion type (rapid / linear)
- Feedrate (already resolved and scaled)

Important invariants:

- Arcs never reach the executor as arcs
- All geometry is resolved before execution
- Executors do not interpret G-code or modal state

---

### Modal state

CNCModalState (modal_state.py) holds all CNC modal state that persists across
G-code lines:

- Units (G20 / G21)
- Distance mode (G90 / G91)
- Active plane (G17 / G18 / G19)
- Feedrate
- Work coordinate systems (G54–G59)
- Current position
- Arc tolerance

The interpreter is the only component allowed to mutate modal state.

---

## Parsing layer

### parser.py

parse_gcode_line() performs lightweight lexical parsing:

- Removes comments
- Extracts <LETTER><NUMBER> words
- Separates:
  - G-codes
  - M-codes
  - Other words (X, Y, Z, I, J, F, etc.)

It does not:
- Enforce ordering
- Apply modal logic
- Interpret motion semantics

---

## Interpreter layer

### interpreter.py

The interpreter converts parsed G-code into motion primitives.

Responsibilities:

1. Apply modal G-code updates (G0/G1/G2/G3, units, plane, distance mode)
2. Track and update modal state
3. Resolve targets into absolute machine-space coordinates
4. Segment motion into executable primitives

#### Linear motion

Linear and rapid moves are segmented using segment_linear():

- Segmentation is time-based, not distance-based
- Ensures no segment exceeds max_segment_time
- Produces straight-line XYZ segments

#### Arc motion

Arc motion (G2/G3):

- Supports all three planes (G17/G18/G19)
- Supports both IJK and R arc formats
- Supports full-circle arcs
- Supports helical arcs (simultaneous Z motion)

Arcs are:
- Segmented in the active plane
- Converted into linear motion primitives
- Executed as standard linear moves

---

## Controller layer

### controller.py

CNCController is the execution engine.

Responsibilities:

- Manage execution state (IDLE / RUNNING / HOLD / CANCELLED)
- Maintain a lookahead buffer
- Step execution one primitive at a time
- Handle feed hold, resume, reset, and cancel
- Track progress and ETA

Execution is pull-based:

- The controller calls step()
- Each step executes exactly one primitive

---

## Streaming execution

### streamer.py

GCodeStreamer provides incremental access to a G-code file:

- Reads line-by-line
- Skips blank lines and full-line comments
- Tracks line number for resume support
- Signals EOF explicitly

Streaming avoids loading entire files into memory.

---

## Executor abstraction

### executor.py

MotionExecutor is an abstract interface with:

- execute(primitive)
- flush()

Executors:
- Do not interpret G-code
- Do not manage modal state
- Do not perform lookahead

---

### klipper_executor.py

KlipperMotionExecutor:
- Converts feedrate mm/min -> mm/s
- Sends absolute moves to Klipper toolhead
- Flushes motion using wait_moves()

---

## Soft limits

SoftLimits provide software-enforced travel limits:

- Axis-specific bounds
- Checked before execution
- Raise explicit exceptions on violation

These are not real-time safety limits.

---

## Progress and ETA

Progress tracking is distance-based:

- Total motion length is precomputed
- Completed length is accumulated during execution
- ETA is computed from feedrate

---

## Test files as documentation

The following files act as executable documentation:

- test_file.py
- test_interpreter.py
- test_modal.py
- test.nc

---

## Design principles

- Explicit execution flow
- Clear separation of responsibilities
- Modal state lives in one place
- Executors are dumb and predictable
- Arcs are resolved early
- Streaming is the default

---

## Known limitations / future work

- Tool changes (M6)
- Spindle / coolant M-codes
- Real-time soft limits
- CNCProgram vs streaming consolidation
- Work offset setting (G10)

---

## Summary

If you understand:
- modal_state.py
- interpreter.py
- controller.py

you understand the system.
