# klipper4cnc

A CNC-capable fork of Klipper with a geometry-first motion pipeline - Very much incomplete but I will be actively developing it. I'm sure I have a few more weeks of coding before testing with an actual cnc.

**klipper4cnc** is a fork of [Klipper](https://github.com/Klipper3d/klipper) that adds a true CNC execution mode while preserving full compatibility with traditional 3D printer usage.

The goal is **not** to turn Klipper into “kind-of CNC”, but to allow Klipper to operate in **two distinct motion personalities**:

- **Printer mode** – behaves like upstream Klipper
- **CNC mode** – enables CNC-native G-code semantics, geometry-aware motion, and machine-oriented execution

This project is focused on correctness, clarity, and long-term maintainability rather than quick hacks or configuration-only solutions.

---

## Why klipper4cnc exists

Klipper is an excellent motion control system, but it is fundamentally **printer-centric**:

- Motion is optimized around extrusion timing
- G-code lines are treated as execution units
- Arcs, feeds, and progress reporting are secondary concerns
- CNC concepts (work offsets, tool compensation, geometry preservation) are not first-class

CNC machines have very different requirements:

- Geometry must be preserved exactly
- Feedrate is the primary control variable
- Arcs and helical motion are common and fundamental
- Progress and ETA must be distance-based
- Motion planning must be geometry-aware, not line-based

klipper4cnc exists to address these differences **without discarding Klipper’s strengths**:
its real-time MCU scheduling, host/MCU split, and robust infrastructure.

---

## Design philosophy

### 1. CNC is a mode, not a hack
CNC support is implemented as an **opt-in execution mode**, not as patches scattered throughout Klipper core.

When CNC mode is disabled:
- Behavior matches upstream Klipper as closely as possible

When CNC mode is enabled:
- Motion semantics change
- G-code is interpreted into geometry
- CNC-specific rules apply

---

### 2. Geometry-first motion
In CNC mode, G-code is **not executed line-by-line**.

Instead, the pipeline is:
G-code
- parsed commands
- modal interpretation
- geometric motion primitives
- time-parameterized execution


This allows:
- Correct arc handling (G2/G3)
- Feedrate preservation across segmented motion
- Distance-based progress and ETA
- A foundation for cutter compensation and offsets

---

### 3. Separation of concerns
The CNC implementation is intentionally layered:

- Parsing ≠ interpretation
- Interpretation ≠ geometry
- Geometry ≠ execution
- Execution ≠ hardware

This makes the system:
- Easier to reason about
- Easier to test
- Safer to extend

---

## What klipper4cnc changes (high level)

### New CNC subsystem
A new CNC-specific subsystem lives under:
- klippy/extras/cnc

This subsystem includes:

- A CNC-aware G-code parser and interpreter
- Modal state tracking (feeds, planes, distance modes, etc.)
- Geometry-aware motion primitives (linear, arc)
- A streaming execution pipeline
- Abstract executors for real hardware and testing
- A mock executor and test harness

All CNC-specific logic is isolated here to minimize impact on printer mode.

---

### Motion primitives instead of “moves”
Instead of treating each G-code line as a move, CNC mode produces **motion primitives** that represent actual toolpath geometry:

- Linear segments
- Circular arcs
- (future) helical motion, probing, etc.

Each primitive is:
- Length-aware
- Feedrate-aware
- Time-consistent

---

### Streaming, not batching
CNC programs can be large and arc-heavy.

klipper4cnc uses a **streaming execution model**:
- Bounded lookahead
- Safe cancellation
- End-of-file–safe draining
- Continuous progress tracking

---

## Current status

klipper4cnc is **under active development** and is not yet a drop-in CNC controller.

What exists today:

- CNC G-code parsing and interpretation
- Modal state tracking
- Linear and arc geometry handling
- Geometry-to-motion primitive conversion
- Streaming execution pipeline
- Abstract executor interface
- Mock executor and automated tests

What is in progress / planned:

- Work coordinate systems (G54–G59)
- Tool length offsets (G43 / G49)
- Cutter radius compensation (G41 / G42)
- Improved lookahead and junction blending
- Homing and limit semantics appropriate for CNC
- Feed hold / resume safety paths

---

## What klipper4cnc is *not*

- Not a configuration-only CNC “profile”
- Not a macro pack
- Not a minimal patch to upstream Klipper
- Not intended to replace LinuxCNC

The aim is a **clean, understandable CNC execution model** built on Klipper’s proven infrastructure.

---

## Who this is for

klipper4cnc may be interesting to you if you:

- Want to run a CNC router, mill, plasma cutter, or laser on Klipper
- Care about geometric correctness and feedrate fidelity
- Want to experiment with CNC motion control without rewriting everything from scratch
- Are comfortable with software still in active development

---

## Relationship to upstream Klipper

klipper4cnc is a fork and tracks upstream Klipper where practical.

The long-term intent is:
- Minimal divergence in printer mode
- Clearly defined divergence in CNC mode
- Explicit boundaries between shared infrastructure and CNC semantics

---

## License

klipper4cnc is licensed under the same terms as Klipper (GNU GPL v3).  
See the `COPYING` file for details.

---

## Disclaimer

This project can move real hardware. (Not yet, but it will soon)

**Use at your own risk.**  
Do not run on machines without proper safety measures.

---

## Contributing

Contributions, discussion, and design review are welcome — especially around:

- CNC motion planning
- G-code semantics
- Safety and limits
- Testing strategies

This project values **clarity and correctness over speed**.
