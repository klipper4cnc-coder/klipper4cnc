# CNC Execution Flow

This document describes the runtime execution flow of the CNC subsystem.
It follows a single CNC job from G-code input to completed motion.

This is a procedural view of the system. For design rationale, see
design_notes.md. For architecture, see architecture.md.

---

## Execution modes

Two execution modes exist:

1. Streaming execution (primary)
2. Preloaded program execution (secondary / experimental)

This document focuses on streaming execution, as it is the intended default.

---

## High-level streaming flow

The streaming execution pipeline is:

1. Prescan (optional, for progress estimation)
2. Runtime setup
3. Streaming interpretation
4. Lookahead buffering
5. Primitive execution
6. Completion / shutdown

Each phase is described in detail below.

---

## 1. Prescan phase (optional)

Purpose:
- Compute total motion length
- Enable progress percentage and ETA reporting

Steps:

1. Create a fresh CNCModalState
2. Set arc tolerance
3. Create a CNCInterpreter using that state
4. Load the G-code file using GCodeFileLoader
5. For each line:
   - Parse the line
   - Interpret it into motion primitives
   - Sum primitive lengths
6. Discard all generated primitives
7. Return total_length

Important properties:
- No execution occurs
- Modal state is isolated from runtime
- Geometry resolution matches runtime behavior

---

## 2. Runtime setup

Before execution begins:

1. Create a new CNCModalState
2. Apply the same arc tolerance used during prescan
3. Create a CNCInterpreter using the runtime state
4. Create a MotionExecutor (KlipperMotionExecutor)
5. Create a CNCController
6. Set total_length on the controller (if prescan was performed)
7. Create a GCodeStreamer for the input file

At this point:
- No motion has been executed
- No G-code has been interpreted
- Controller state is IDLE

---

## 3. Controller start

Execution begins when:

controller.start()

This transitions the controller state from:
- IDLE -> RUNNING

No motion is executed at this step.
It only enables execution.

---

## 4. Streaming loop entry

Execution enters:

controller.run_stream(streamer, interpreter)

The controller:

1. Opens the streamer
2. Enters the main execution loop
3. Continues until:
   - Job completes
   - Execution is cancelled
   - An error is raised

---

## 5. Lookahead buffer fill

Inside the main loop, the controller attempts to fill the lookahead buffer.

For each iteration:

1. Request next line from the streamer
2. If EOF:
   - Mark eof = True
   - Stop filling
3. Otherwise:
   - Parse the G-code line
   - Interpret it into motion primitives
   - Append primitives to the lookahead buffer

Notes:
- Modal state is updated during interpretation
- Buffer size is capped
- Interpretation and execution are decoupled

---

## 6. Execution gating (HOLD / CANCEL)

Before executing motion, the controller checks state:

- If CANCELLED:
  - Exit immediately
- If not RUNNING (e.g. HOLD):
  - Skip execution
  - Continue buffer management only

This ensures:
- Feed hold pauses motion instantly
- Interpretation does not accidentally advance execution

---

## 7. Single-step execution

When execution is allowed, the controller calls:

step()

step() performs:

1. Check controller state (must be RUNNING)
2. Check lookahead buffer is not empty
3. Pop one MotionPrimitive
4. Validate feedrate resolution
5. Send primitive to executor.execute()
6. Update completed_length
7. Report progress if threshold reached
8. Return True if a step was executed

Exactly one primitive is executed per step.

---

## 8. Progress and ETA reporting

Progress is reported based on distance traveled.

Mechanism:

- completed_length accumulates primitive lengths
- total_length was computed during prescan
- Progress is reported every fixed distance increment
- ETA is computed using:
  remaining_length / last_feedrate

If feedrate is unknown, ETA is reported as "?".

---

## 9. End-of-job detection

Execution completes when:

- eof == True
- AND lookahead buffer is empty
- AND step() returns False

At this point:
- All motion primitives have been executed
- No more G-code remains

The controller exits the streaming loop.

---

## 10. Shutdown and flush

Finally:

1. The streamer is closed
2. The executor is flushed
3. Controller remains in RUNNING or IDLE state
4. No further motion occurs

The job is complete.

---

## Feed hold and resume flow

Feed hold can occur at any time:

controller.feed_hold()

This transitions:
- RUNNING -> HOLD

Effects:
- Execution pauses immediately
- No further primitives are executed
- Lookahead buffering may continue

Resume is performed by:

controller.resume()

This transitions:
- HOLD -> RUNNING

Execution continues exactly where it left off.

---

## Cancel flow

Cancellation is terminal:

controller.cancel()

Effects:
- Controller state becomes CANCELLED
- Streaming loop exits
- Execution cannot resume
- reset() is required to continue

This prevents ambiguous partial execution states.

---

## Error handling

Errors may occur during:

- Parsing
- Interpretation
- Limit checking
- Execution

Errors propagate upward and terminate execution.

The controller does not attempt to recover from:
- Geometry errors
- Limit violations
- Unresolved feedrates

Fail-fast behavior is intentional.

---

## Summary

The CNC execution flow is:

- Deterministic
- Explicit
- Single-step driven
- Streaming-first
- Geometry-resolved before execution

Every executed motion has a clear origin and context.

If you can trace:
- parse -> interpret -> buffer -> step -> execute

you understand the runtime behavior of the system.
