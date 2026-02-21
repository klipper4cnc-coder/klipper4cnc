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
3. Controller start
4. Job runner entry (reactor timer in Klipper)
5. Lookahead buffering
6. Primitive execution
7. Completion / shutdown

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
5. Create a CNCPlanner (if using planned mode)
6. Create a CNCController
7. Set total_length on the controller (if prescan was performed)
8. Create a GCodeStreamer for the input file

In Klipper runtime (cnc_mode), an additional step occurs:

9. Register a reactor timer that will drive execution incrementally via
   controller.pump(...). CNC_START schedules the timer and returns immediately
   so Klipper can continue to process commands (HOLD/RESUME/CANCEL).

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

## 4. Job runner entry (reactor timer)

### Klipper runtime (non-blocking)

In real Klipper, execution is driven by cnc_mode via a reactor timer callback.

1. CNC_START initializes runtime state (streamer/interpreter/planner/controller)
2. CNC_START calls controller.start() and schedules a reactor timer, then returns
3. Each timer callback:
   - Reads a small batch of lines from the streamer
   - Parses + interprets into MotionPrimitives
   - Pushes primitives into the planner and fills the ready queue
   - Executes a small number of primitives via controller.step()
   - Reschedules itself

This keeps the host reactor responsive so CNC_FEED_HOLD / CNC_RESUME / CNC_CANCEL
can take effect promptly.

### Offline / test tools (blocking)

In offline scripts and unit tests, it is acceptable to drive the controller in a
blocking loop (e.g. controller.run_stream(...)) because there is no reactor
responsiveness requirement.

---

## 5. Lookahead buffer fill

Inside the job runner, the controller attempts to fill the lookahead buffer.

For each iteration:

1. Request next line from the streamer
2. If EOF:
   - Mark eof = True
   - Stop filling (and finalize planner if applicable)
3. Otherwise:
   - Parse the G-code line
   - Interpret it into motion primitives
   - Append primitives to the lookahead buffer (legacy mode) OR
     push primitives into the planner (planned mode) and append committed
     PlannedPrimitives into the ready queue

Notes:
- Modal state is updated during interpretation
- Buffer size is capped
- Interpretation and execution are decoupled

In Klipper runtime, cnc_mode also applies backpressure based on toolhead queue
buffering (avoid queueing too far ahead) so HOLD/CANCEL remain responsive.

---

## 6. Execution gating (HOLD / CANCEL)

Before executing motion, the controller checks state:

- If CANCELLED:
  - Exit immediately
- If not RUNNING (e.g. HOLD):
  - Skip execution
  - Continue buffer management only (optional, depending on runner policy)

This ensures:
- Feed hold pauses motion promptly
- A job can be cancelled without waiting for a blocking loop to finish

---

## 7. Single-step execution

When execution is allowed, the controller calls:

step()

step() performs:

1. Check controller state (must be RUNNING)
2. Check lookahead/ready queue is not empty
3. Pop one MotionPrimitive (or unwrap a PlannedPrimitive)
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

The controller considers internal execution complete when:

- eof == True
- AND the lookahead buffer / ready queue is empty
- AND step() returns False

In Klipper runtime, cnc_mode typically adds a "drain" phase:

- When the controller reports EOF and internal queues are empty, stop feeding new moves
- Wait for the toolhead queue to drain (queued motion time approaches 0)
- Then mark the job complete and stop the reactor timer

At this point:
- All motion primitives have been executed (from the controller’s perspective)
- No more G-code remains

---

## 10. Shutdown and flush

Finally:

1. The streamer is closed
2. The executor may be flushed (if running in blocking/offline mode)
3. The controller remains in RUNNING or IDLE state depending on policy
4. No further motion occurs

In Klipper runtime, avoid blocking waits in the reactor; draining is performed
by polling toolhead queue state instead of calling wait_moves() inside the timer.

The job is complete.

---

## Feed hold and resume flow

Feed hold can occur at any time:

controller.feed_hold()

This transitions:
- RUNNING -> HOLD

Effects:
- Execution pauses promptly
- No further primitives are executed
- Lookahead buffering may continue (runner-dependent)

Resume is performed by:

controller.resume()

This transitions:
- HOLD -> RUNNING

Execution continues where it left off.

---

## Cancel flow

Cancellation is terminal:

controller.cancel()

Effects:
- Controller state becomes CANCELLED
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

parse -> interpret -> buffer/plan -> step -> execute

you understand the runtime behavior of the system.