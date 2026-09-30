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
Markers and bounded accepted-position traces (the most recent 300 accepted
platform-origin positions) are controlled by `setMarkers`, `setTraces`, and
`clearTrace`. `setOverlays(patch)` switches named scene overlays on or off;
names left out of the patch keep their state, and an unknown name or a
non-boolean value throws without changing anything. Animation patterns advance at most 0.1 s of simulated time per
frame, scaled by the speed multiplier (0.1 to 5 in the UI).

## Mechanical geometry controls

`createGeometryControls({ document, container, controller })` mounts the active
geometry editor in `#simGeometryControls`. Its editor copies the controller's
layout before every edit and reloads that copy through `loadLayout`, which
reevaluates the home pose and publishes fresh diagnostics. A selected optimizer
candidate becomes an `editable` simulator source carrying its `candidateId`;
the retained optimizer candidate stays unchanged. Every edit rederives the
layout's `derived` mounting directions against the new home geometry and keeps
`supplied` ones, so the exported `mounting` block matches the evaluator's.
Reset restores the geometry and source last loaded from outside the editor
while keeping current evaluator options.

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
fields show the same current value; invalid edits, including an empty or
non-numeric field (never read as 0), leave the last valid layout in place,
restore the field and report a field-specific error. A focused field whose
text the user has changed is not rewritten by animation frames, so geometry
can be edited while a pattern plays; a rejected edit still restores the stored
value. A focused field that has not been edited follows every layout change,
so Reset geometry, a candidate or reference load and a browser-save restore
update it even in browsers where clicking a button leaves focus in the field.
A committed entry counts as untouched even when the stored value prints
differently from the typed text (`77.0` for 77), and values print with at most
12 significant digits so a degree that round-trips through radians shows 30
rather than 29.999999999999996.
## Browser workflow

The Optimize and Simulate tabs share the retained candidate selection. The
simulator loads the selected candidate automatically after a run and when the
chart or either candidate list changes. Switching tabs leaves the current pose,
camera, input mode and animation state in memory. A new run clears the previous
candidate selection. After **Load optimizer reference** the candidate select
gains an **Imported reference** entry, is enabled even without a run, and
choosing that entry again reloads the imported document after a candidate was
shown.

The native WebGL2 renderer reads only the accepted evaluator geometry: base
anchors, solved horn tips, actual rods, platform points, servo orientation
angles and platform frame. Orbit camera is the initial mouse mode: dragging with the primary button or
first touch changes yaw and pitch (a right click or second finger does not
reset the drag), the
wheel zooms between 80 and 2,500 units, and **Reset camera** restores the
default view. The projected depth range follows the camera distance so a
zoomed-out view keeps near/far line ordering instead of saturating the depth
buffer. Moving the platform by pointer requires selecting **Move
platform**, which converts drag distance to X/Y requests. Numeric controls and
sliders request six-axis poses; sliders span ±50 mm and ±30°, while the numeric
fields accept any finite value. As with the geometry and diagnostics fields, a
pose field the user is typing into keeps its text while an animation plays, an
untouched focused field follows every request, a committed entry counts as
untouched, and an invalid entry restores the requested value. Keyboard arrows move X/Y, Page Up/Down moves Z,
W/S tilts about X, A/D tilts about Y, and Q/E rotates about Z, each by 1 mm or
1° per press (Shift doubles the step); keys are ignored while a field has focus
or the Simulate tab is hidden. An optional gamepad maps the left stick to X/Y,
the right stick to tilt, the triggers to Z and the shoulder buttons to yaw with
a 0.15 dead zone. Pose requests do not change anchor coordinates.

**Use geometry as optimizer reference** copies the active layout and a replay
snapshot to the optimizer's reference JSON field. **Load optimizer reference**
can restore that layout and saved requested/accepted poses. **Download simulator
JSON** saves the same geometry plus run settings, simulator options, pose,
camera, animation, markers, traces, overlay toggles and input mode. The shared importer validates
the layout and ignores old scores; every pose is checked again by the current
evaluator. The `simulator` block is validated as a whole before anything is
applied: `options` (only the known keys are kept; limits, servo bounds,
tolerance, condition limit and the clamp flag must have the right type),
`requested` and `accepted` poses, `camera` (finite yaw, pitch within ±1.4,
distance 80 to 2,500, three-coordinate target), `animation.speed` (positive),
`markers`, `tracesEnabled`, `overlays` (each known overlay name true or false;
unknown names are dropped, and a file without the block keeps the current
toggles) and `pointerMode` (`orbit` or `platform`). A rejected
file or browser save reports the offending `simulator.` field and leaves the
current layout, pose, camera, animation and, for a browser save, the optimizer
inputs untouched. If WebGL2 is unavailable, the simulator names the missing capability
and suggests enabling hardware acceleration or a modern desktop browser;
optimization remains usable. If the browser loses the WebGL2 context (GPU reset,
driver crash, backgrounded mobile tab, too many contexts on one page), the pose
status reports it, the renderer rebuilds its program and buffers when the
context is restored and redraws the accepted pose; pose requests are still
evaluated while the context is lost.

Loaded animations remain paused. A saved idle (`none`) or unrecognized pattern
selects Wobble as the playable default, so Play works after a save/load before
the first animation has been started.

## Scene builders and overlays

`buildSceneGeometry(state)` in `src/simulator/scene.js` (re-exported by the
renderer) concatenates the output of an ordered list of builders,
`SCENE_BUILDERS`. Each builder is `(state, layout, solved) => { lines, points }`,
where `solved` is the accepted assessment or `null`, so builders only draw data
the evaluator already produced. The list order is the draw order: `base`
(base polygon, servo direction stubs and base markers), `platform` (platform
polygon), `legs` (horns, rods and their markers), `servoArcs`, `platformAxes`,
`worldAxes` and `trace`. Markers and traces keep their own controls.

A builder with an `overlay` key is drawn only when that key is on in
`state.overlays`. `OVERLAY_DEFAULTS` lists every toggleable overlay and its
default; today these are **Servo arcs** (`servoArcs`), **Platform axes**
(`platformAxes`) and **World axes** (`worldAxes`), all on. With only the two
axis overlays on, the scene matches the original single-function renderer line
for line (a frozen fixture in `tests/fixtures/scene-geometry.json` checks this). The Simulate tab shows one checkbox per overlay in the
**Overlays** group, with the id `simOverlay` plus the capitalised name (for
example `simOverlayWorldAxes`). A new overlay adds one builder, one default,
one checkbox and a paragraph here; new overlays default off unless their
issue says otherwise.

**Servo arcs** (on by default) draw each servo's allowed travel as an arc of
horn-length radius about its base anchor, from the effective minimum to the
effective maximum servo angle (the simulator's `servoRangeRad` option, else the
layout's range, else ±90°, as `effectiveServoRange` in `src/model/pose.js`
resolves it for the evaluator). The arc uses the evaluator's horn frame, so its
ends are exactly the horn tips at the two stops. Short ticks mark both stops and
a marker crosses the arc at the accepted horn angle; with no accepted pose the
marker is omitted. The arc and its ticks are grey, the marker is horn orange,
and all three turn yellow when the accepted angle is within
`NEAR_LIMIT_MARGIN_RAD` (5°) of a stop, or red when the requested pose fails
that servo's range (`servoLimit`). Other failures leave the arc colours alone;
the leg itself is still coloured as described under Live diagnostics. The margin
constant lives in `src/simulator/scene.js` for all limit overlays.

## Live diagnostics

The diagnostics panel subscribes to the same controller as the renderer. It reports the six-axis **requested** pose and last valid **rendered accepted** pose separately. A rejected request remains visible with its evaluator failures while the renderer holds the last accepted geometry. Red leg rows and matching WebGL leg colors mark failures in the requested pose; a whole-platform conditioning failure has a separate message and color. Highlighting does not imply that the rejected pose was rendered.

Each leg shows lower and upper socket deflection against their effective limits, maximum deflection, rod-length deviation against the editable tolerance, and servo angle. Unavailable measurements show a dash because the evaluator may stop at the first structural failure. Lower and upper joint limits and rod tolerance can be edited in the panel; changes reevaluate the current request and last accepted pose, a focused field keeps text the user has typed while an animation plays, an untouched focused field still follows loads and settings changes, and a rejected value is restored. The mandatory reciprocal conditioning cutoff and optional engineering condition limit are displayed. Simulator JSON saves these effective options with the requested and accepted poses, and loading that JSON restores them.
