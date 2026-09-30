# Ball-joint mounting model

Each leg has two ball sockets. The lower socket is attached to its moving servo horn; the upper socket is attached to the moving platform. A socket direction is a unit vector toward the **other endpoint of the same rod**. The lower direction points from horn tip toward platform anchor. The upper direction points from platform anchor toward horn tip. Their deflections are evaluated separately against the actual rod at every pose.

## Frames and signs

All internal lengths are millimeters and angles are radians. For leg `i`, servo-plane azimuth `betaAngles[i]` is measured from base +X toward +Y. Positive servo angle `alpha` raises the horn. The lower socket direction is stored in a right-handed horn frame whose local X follows the horn, local Y is tangential to its servo plane, and local Z completes the basis:

- X = `(cos(alpha) cos(beta), cos(alpha) sin(beta), sin(alpha))`
- Y = `(-sin(beta), cos(beta), 0)`
- Z = `(-sin(alpha) cos(beta), -sin(alpha) sin(beta), cos(alpha))`

The positive servo rotation axis is **minus** the horn-frame Y axis: `dX/dalpha = Z` and `X × Z = −Y`, so increasing `alpha` rotates the horn about `−Y` (equivalently, a right-handed rotation about `+Y` lowers the horn). Keep this in mind when checking servo torque signs by hand; the code is consistent with it.

The upper socket direction is stored in the platform's local frame. Platform Euler orientation uses `Rz(rz) Ry(ry) Rx(rx)`, as in `rotationMatrixFromEuler`. At a pose, lower local direction is transformed by the horn frame and upper local direction by the platform rotation. Deflection is the angle between each transformed socket direction and its oriented rod direction. For the upper socket, the oriented rod is reversed. `socketNormalsInWorld(beta, alpha, rotationMatrix, { lower, upper })` in `src/model/pose.js` performs this transform; the evaluator and the simulator's joint-cone overlay both call it.

## Directions, limits, and diagnostics

With no override, both socket directions are derived from inverse kinematics at the home pose. A valid home pose therefore has zero deflection at both sockets. Overrides can be supplied per leg and per joint through `layout.mounting`:

```js
layout.mounting = {
  lower: [null, { direction: [0, 0, 1], source: 'supplied' }, null, null, null, null],
  upper: [null, null, null, null, null, null],
};
```

A missing joint array or `null` entry means `derived`; a vector entry is shorthand for `{ direction: vector, source: 'supplied' }`. Supplied directions must be finite and nonzero and are normalized. Each effective entry is `{ direction: [x,y,z], source: 'derived'|'supplied' }`. Derived directions are recalculated when geometry or home height changes; supplied directions remain fixed in their mounting frames. `resolveMounting(layout)` returns these effective entries without changing the layout. `resolveMounting(layout, { imported: true })` also returns a `migration` record when an import has no mounting data: `{ upgraded, fromModelVersion, toModelVersion, note }`. `upgraded` is `true` with the legacy-upgrade note only when the import's `model_version` is below the current `MODEL_VERSION`; a current-version layout that merely omits `mounting` reports `upgraded: false` with a neutral note that the sockets were derived from the home geometry. Importers must discard old computed metrics and reevaluate the unchanged geometry under the new model.

`ballJointLimitDeg` applies to both sockets. The optional `lowerBallJointLimitDeg` and `upperBallJointLimitDeg` override that shared limit independently in the pose, workspace, cycle, and optimizer evaluators. For optimization they are headless constructor options only: the Optimize tab exposes a single shared **Ball Joint Limit** and the requirements JSON only has `ball_joint_max_deg`. The simulator diagnostics panel can edit the lower and upper limits separately for pose checks, and simulator JSON saves them. The limits, `servoRangeRad` and `rodLengthTolerance` must be numbers of the right kind (`servoRangeRad` two finite radians with max >= min); a `null`, string, boolean or array is rejected with an error naming the option rather than coerced, so hand-edited simulator JSON cannot turn a missing limit into 0° or silently disable the servo check. A joint at its limit passes; a joint beyond it produces a `ballJoint` violation with `leg`, `joint`, `value`, and `limit` (radians). A pose reports `jointAngles.lower` and `jointAngles.upper` separately; `ballJointAngles[i]` retains the worst angle of leg `i` for existing aggregate consumers. Workspace statistics include per-joint maxima and violation counts. Strict feasible coverage excludes either joint failure. Soft mode reports a separate relaxed exploration coverage; it does not turn a violating pose into a mechanically feasible pose. Cycle checks remain strict and report the failed sample, pose, and joint-specific violations.

The cycle torque and speed model remains the actual-rod force balance and horn-angle derivative calculation described in [CYCLE_MODEL.md](./CYCLE_MODEL.md). Socket deflection only changes mechanical validity; it does not substitute a nominal leg direction into that calculation.
