# Current optimizer architecture and reproduction guide

This guide describes the implemented root application. The supplemental v3/v4 specifications and earlier research guides are historical design proposals. For launch and input instructions, start with [README.md](README.md) and [requirements](docs/REQUIREMENTS.md).

## Runtime

Serve the repository with `python -m http.server 8000 --bind 127.0.0.1` and open the served root. The HTML statically imports native JavaScript modules. It fetches the bundled sample JSON; there is no network service, database, bundler or third-party math dependency in the root app. Node.js >=22 is used for `npm test`, not required to serve the app.

The p5/quaternion/Stewart scripts belong only to the [archived simulator](Raw%20Information/Stuff%20from%20Old%20Project/README.md). Its prebuilt bundle includes separate third-party dependencies and has its own locked build command.

## Data flow and geometry

1. `parseRequirements(text)` normalizes flat/nested input, validates it, and returns `{ normalized, workspace }`. Every nonzero axis initially gets min/midpoint/max samples. Bounds are inclusive when a step lands on the maximum; arbitrary steps do not force an extra endpoint.
2. The UI fills defaults, retaining explicit overrides at Run. An explicit optimizer joint-limit option wins over normalized requirements. Loading the sample resets the controls.
3. `Optimizer` generates six approximately circular base anchors, six offset platform anchors, and perturbed beta angles. All legs share a horn length, rod length and fixed servo range. Initial radii are 90-160 mm for the base and 40-120 mm for the platform (clamped below the base radius). Mutations move anchors and orientations; these initial radii are not hard bounds on later anchor mutations. Platform anchor Z stays zero.
4. `finalizeLayout` clamps shared lengths and derives home height from the average square-root rod/horn geometry term. The initially sampled platform-height field is overwritten by this derived value; it is not an independent final design variable.
5. `evaluateLayout` sweeps workspace poses, evaluates home geometry, evaluates the required cycle, and assembles objectives. None of these loops are stubs.

Direct `new Optimizer()` defaults are population 12, generations 5, mutation rate 0.35, joint limit 52 degrees, horn bounds [30,120] mm and rod bounds [160,420] mm. The UI passes parser-normalized requirements: omitted JSON limits instead use 45 degrees, [30,110] mm and [160,420] mm. The bundled sample explicitly specifies 52 degrees, [40,110] mm and [180,380] mm. Both entry paths use [-120,120] degrees servo travel unless overridden. Prefer parsing requirements rather than constructing incomplete data by hand.

## Pose evaluator

Internal geometry uses mm and angles use radians. R = Rz * Ry * Rx rotates a local platform anchor. Add translation and the home-height Z offset to obtain q; l = q - base. With horn length h, rod length d and orientation beta:

```text
e = 2 h lz
f = 2 h (cos(beta) lx + sin(beta) ly)
g = dot(l,l) - (d*d - h*h)
alpha = asin(g / sqrt(e*e + f*f)) - atan2(f,e)
```

Reject degenerate/invalid geometry or a servo angle outside its range. Reconstruct the horn tip, check rod length within 0.5 mm, and check the modeled horn-to-rod ball angle. Only one inverse-kinematics branch is evaluated. `reachable` always excludes joint violations; optional soft exploration can mark an otherwise valid pose `relaxedReachable`. No geometry is clamped.

[Results documentation](docs/RESULTS.md) describes the geometric Jacobian proxies and their limitations. [Cycle documentation](docs/CYCLE_MODEL.md) describes the separate actual-rod equilibrium, gravity direction and torque/speed formulas. The old equal-load harmonic torque approximation is no longer used.

## Evolution and execution

The initial population and each generation's offspring are evaluated. Non-dominated sorting, crowding distance, tournament selection, crossover and probabilistic mutation form an NSGA-II search. Ten objective slots maximize feasible coverage, search coverage (relaxed only in soft mode), dexterity, stiffness proxy, load-balance proxy, isotropy and limit margin, while minimizing cycle torque, cycle speed and fatigue proxy. Invalid cycle demand receives the worst demand objective. No random seed is exposed.

Preflight counts samples arithmetically before allocating range arrays. It rejects more than 100,000 workspace poses per layout or 1,000,000 total budgeted pose evaluations per run. Total work is:

```text
population * (generations + 1) * (workspace poses + cycle phases + 1 home pose)
```

Cycle phases are 64 for moving cycles and 1 for stationary cycles. The default sample budgets 72 * (729 + 64 + 1) = 57,168 checks; early cycle failure can perform fewer actual checks. Progress accounts for the candidate budget, not wall-clock time.

Workspace sweeps yield to the event loop before starting and every 256 poses. Statistics use running means; reservoir samples cap retained example poses at 200/class. A run's AbortController is checked during evaluation and after each yield. Cycle checks are bounded and observe the same signal. There is no worker thread or server compute service.

`await optimizer.start()` or `await optimizer.run()` returns a completed/cancelled outcome. `optimizer.stop()` requests cancellation. Overlapping runs reject. Errors propagate; they are not reported as successful completion. Cancellation retains the most recent fully evaluated population, not an unfinished sweep/generation. UI and download mark retained results partial.

## Verification and reproduction

Run `npm test` for the checked-in regression suite. It covers normalization, controls, feasible/relaxed accounting, export, work limits, real event-loop yielding, cancellation, cycle force balance and finite-difference speeds, and the archived bundle's custom-layout API. Browser smoke checks are manual: complete the sample, observe progress, cancel a longer run, inspect the result, and verify the archived canvas/animation separately.

The search is a heuristic. Reproduction of numerical runs requires preserving a layout and settings or adding controlled random seeding; the public UI currently has no seed control. Exported geometry and metrics are a starting point for independent engineering analysis, not a manufacturing specification.
