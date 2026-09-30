# Active simulator

The simulator uses the same internal layout and `evaluatePose` function as the
optimizer. Lengths are millimetres, pose rotations and `betaAngles` are radians,
and `servoRangeRad` is radians. Downloaded layouts convert the servo range to
degrees through the shared layout export contract.

## Controller boundary

`createSimulatorController()` in `src/simulator/controller.js` owns a copy of the
active layout. `loadLayout(layout, { source, options })` resets pose state and
checks home. `source` identifies a candidate, import, or editable copy; candidate
sources carry `candidateId`. The controller never edits the retained optimizer
result. `getReferenceLayout()` returns another independent copy for optimizer
reference input or JSON transfer.

`requestPose(pose, { source })` always records six finite requested coordinates
and calls `evaluatePose` with `recordLegData: true`. `getState()` returns the
requested pose, evaluator `assessment`, last valid `accepted` pose, and
`acceptedAssessment`. A failed request keeps its leg-specific diagnostics but
does not change `accepted`; if home fails, `accepted` remains `null`. Rendering
uses the accepted pose only. `setOptions(patch)` reevaluates both the accepted
pose and current request. `subscribe(listener)` provides a snapshot immediately
and after every change; its returned function unsubscribes. Geometry controls
and diagnostics should use these calls rather than duplicating the evaluator.

`setAnimation(pattern, playing, settings)` and `tick(deltaSeconds, settings)`
support wobble, ping-pong, rotation, tilt, and helical motion. An invalid
animation frame pauses playback and leaves its request and failure visible.
Markers and bounded accepted-position traces are controlled by `setMarkers`,
`setTraces`, and `clearTrace`.

## Mechanical geometry controls

`createGeometryControls({ document, container, controller })` mounts the active
geometry editor in `#simGeometryControls`. Its editor copies the controller's
layout before every edit and reloads that copy through `loadLayout`, which
reevaluates the home pose and publishes fresh diagnostics. A selected optimizer
candidate becomes an `editable` simulator source carrying its `candidateId`;
the retained optimizer candidate stays unchanged. Reset restores the geometry
and source last loaded from outside the editor while keeping current evaluator
options.

Circular, C3 paired, and rectangular paired layouts start in parametric mode.
Radius, C3 anchor pair gap, rectangular aspect, base/platform turn, and horn
direction offset use the shared `topologyGeometry` generator. Turn and horn
direction controls display degrees but store radians. The generator keeps the
selected topology's anchor and beta-angle invariants. Horn/rod length, home
height, and servo range are separate scalar controls; the servo range displays
degrees and stores radians.

**Edit anchors explicitly** preserves the current anchors and beta angles
exactly while switching topology metadata to `free`. Explicit mode offers each
base/platform XYZ coordinate and each horn direction. **Generate selected
topology** is the deliberate reverse mode switch: it replaces explicit anchors
with coordinates from the chosen shared generator. Changing unrelated lengths,
height, or servo bounds never regenerates explicit anchors. Sliders and numeric
fields show the same current value; invalid edits leave the last valid layout
in place and report a field-specific error.
