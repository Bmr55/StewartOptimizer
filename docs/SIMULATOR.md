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

Circular and Rectangular also expose an alternating horn offset. Newly generated
layouts start at 30°; legacy imports without this parameter display 0° and keep
their original directions until explicitly edited.

**Edit anchors explicitly** preserves the current anchors and beta angles
exactly while switching topology metadata to `free`. Explicit mode offers each
base/platform XYZ coordinate and each horn direction. **Generate selected
topology** is the deliberate reverse mode switch: it replaces explicit anchors
with coordinates from the chosen shared generator. Changing unrelated lengths,
height, or servo bounds never regenerates explicit anchors. Sliders and numeric
fields show the same current value; invalid edits leave the last valid layout
in place and report a field-specific error.
## Browser workflow

The Optimize and Simulate tabs share the retained candidate selection. The
simulator loads the selected candidate automatically after a run and when the
chart or either candidate list changes. Switching tabs leaves the current pose,
camera, input mode and animation state in memory. A new run clears the previous
candidate selection.

The native WebGL2 renderer reads only the accepted evaluator geometry: base
anchors, solved horn tips, actual rods, platform points, servo orientation
angles and platform frame. Orbit camera is the initial mouse mode. Moving the
platform by pointer requires selecting **Move platform**. Numeric controls and
sliders request six-axis poses; keyboard arrows move X/Y, Page Up/Down moves Z,
W/S and A/D tilt, and Q/E rotates about Z. An optional gamepad maps analog
sticks to translation/tilt and shoulder/trigger buttons to Z/yaw. Pose requests
do not change anchor coordinates.

**Use geometry as optimizer reference** copies the active layout and a replay
snapshot to the optimizer's reference JSON field. **Load optimizer reference**
can restore that layout and saved requested/accepted poses. **Download simulator
JSON** saves the same geometry plus run settings, simulator options, pose,
camera, animation, markers, traces and input mode. The shared importer validates
the layout and ignores old scores; every pose is checked again by the current
evaluator. If WebGL2 is unavailable, the simulator names the missing capability
and suggests enabling hardware acceleration or a modern desktop browser;
optimization remains usable.

Loaded animations remain paused. A saved idle (`none`) or unrecognized pattern
selects Wobble as the playable default, so Play works after a save/load before
the first animation has been started.

## Live diagnostics

The diagnostics panel subscribes to the same controller as the renderer. It reports the six-axis **requested** pose and last valid **rendered accepted** pose separately. A rejected request remains visible with its evaluator failures while the renderer holds the last accepted geometry. Red leg rows and matching WebGL leg colors mark failures in the requested pose; a whole-platform conditioning failure has a separate message and color. Highlighting does not imply that the rejected pose was rendered.

Each leg shows lower and upper socket deflection against their effective limits, maximum deflection, rod-length deviation against the editable tolerance, and servo angle. Unavailable measurements show a dash because the evaluator may stop at the first structural failure. Lower and upper joint limits and rod tolerance can be edited in the panel; changes reevaluate the current request and last accepted pose. The mandatory reciprocal conditioning cutoff and optional engineering condition limit are displayed. Simulator JSON saves these effective options with the requested and accepted poses, and loading that JSON restores them.
