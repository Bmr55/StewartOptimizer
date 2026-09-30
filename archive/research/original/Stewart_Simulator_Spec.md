> Historical simulator requirements. This is not a description of the current root optimizer. See the [current README](../../../README.md) and [archived simulator instructions](../../simulator/README.md).

# Stewart Platform Simulator — Specification for CODEX

## 1. Goal
Build a new simulator from the ground up that:
- Visualizes and simulates Stewart platforms with customizable geometry.
- Generalizes the math from the old simulator (currently limited to hex/circle).
- Provides an open-ended UI for exploring *all* mechanical variables (with no enforced limits).
- Supports workspace and performance simulation under load (torque, frequency, payloads).

## 2. Generalization of Math
The current simulator hard-codes hexagonal or circular anchor placement with fixed geometry. We need to generalize:

- Anchor positions: allow explicit arrays of coordinates (6 anchors for base, 6 for platform).
- Servo axis orientations: generalize to accept beta angles (array of 6 values).
- Horn and rod lengths: parameters, not constants.
- IK Math: keep quaternion-based math but adapt for arbitrary anchor placements and servo axes.

## 3. Simulation Functions
Support simulation across all mechanical variables. Every slider in the “Mechanical” section of the old UI becomes a free variable:

- Base radius, platform radius, shaft distance, anchor distance
- Z offset, horn length, rod length
- Servo ranges (min/max angles)
- Anchor placements (direct input)
- Servo axis orientations (beta)

Instruction: Treat all as idealized defaults for symmetrical layouts, but remove artificial upper and lower bounds.

## 4. UI Requirements
- Left panel with collapsible sections.
- Mechanical Section: sliders + input boxes for all parameters, no min/max limits.
- Simulation Section: workspace sweeps, performance under load.
- Export Section: export base/platform anchors and results as CSV/JSON.

## 5. Rendering Requirements
- Use Canvas or WebGL.
- Draw anchors, servo horns, rods, and outlines.
- Update live when parameters change.
- Color rods when servo angle exceeds limit.

## 6. New Features
- Support linear layouts (aligned with X or Z).
- Support custom JSON imports (anchors, horn/rod lengths).
- Allow batch simulations (sweep variables, plot results).

## 7. Instruction for CODEX
1. Examine how anchors are generated in the old simulator.
2. Replace fixed formulas with arbitrary user-defined arrays.
3. Generalize IK math for arbitrary anchors.
4. Expand UI sliders to include all mechanical parameters, no bounds.
5. Add simulation for workspace sweeps and performance under load.
6. Ensure default presets are symmetrical designs, but allow overrides.
7. Add support for import/export of layouts and simulation results (JSON/CSV).
