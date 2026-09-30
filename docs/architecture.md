# Current optimizer architecture and reproduction guide

This guide describes the implemented root application. The supplemental v3/v4 specifications and earlier research guides are historical design proposals. For launch and input instructions, start with [README.md](../README.md) and [requirements](./REQUIREMENTS.md).

## Runtime

Run `npm run dev` with Node.js >=22 and open the printed URL. This local static server binds to 127.0.0.1; `-- --port 0` chooses a free port. Any other static server also works. The HTML loads a stylesheet and `src/main.js`, which initializes native JavaScript modules. The sample loader resolves its JSON URL relative to its module. There is no backend, database, bundler or third-party math dependency in the active app.

The p5/quaternion/Stewart scripts belong only to the [archived simulator](../archive/simulator/README.md). Its prebuilt bundle includes separate third-party dependencies and has its own locked build command.

## Module boundaries

| Module | Owns |
| --- | --- |
| `src/ui/app.js` | Event handlers, status display and run/cancel control |
| `src/ui/controls.js` | Workspace inputs and preservation of explicit overrides |
| `src/ui/tooltips.js`, `download.js` | Browser-only interactions |
| `src/model/requirements.js` | Parsing, normalization and physical input validation |
| `src/model/pose.js` | Single-pose inverse kinematics and constraints |
| `src/model/cycle.js` | Trajectory sampling and actual-rod force balance |
| `src/workspace/sweep.js` | Grid iteration, yields, abort checks and sweep progress |
| `src/workspace/statistics.js` | Running metrics, counters and reservoir samples |
| `src/optimization/optimizer.js` | Run state and population lifecycle |
| `src/optimization/layout-operators.js` | Generation, finalization, mutation and crossover |
| `src/optimization/topology.js` | Symmetric anchor generators and declared-topology validation |
| `src/optimization/nsga2.js` | Ranking, crowding, tournament and survivor selection |
| `src/optimization/evaluate-layout.js` | Workspace/home/cycle scoring and objectives |
| `src/optimization/budget.js` | Run workload estimate and limits |
| `src/io/results.js` | Best-candidate selection and both output formats |
| `src/io/sample-requirements.js` | Fetching the bundled example |
| `src/math.js` | Shared numerical primitives |

The UI depends on the optimizer; the optimizer composes search and evaluation functions. Both cycle and workspace evaluation depend on the pose evaluator. Numerical modules never import the UI or manipulate the DOM. Abort signals and progress callbacks cross these boundaries explicitly. NSGA-II intentionally updates evaluation rank/crowding fields; layout mutation and crossover clone their inputs.

## API compatibility

Root `math.js`, `workspace.js`, `cycle.js` and `requirements.js` retain their original exports through compatibility modules. Root `optimizer.js` preserves the original `Optimizer.exportBest()` browser download and overridable `download()` method.

New headless callers use the core directly:

```js
import { Optimizer } from './src/optimization/optimizer.js';
import { parseRequirements } from './src/model/requirements.js';

const { normalized, workspace } = parseRequirements(requirementsText);
const optimizer = new Optimizer(normalized, { ranges: workspace });
const outcome = await optimizer.start();
const json = optimizer.exportBest(); // JSON string, no browser side effects
```

These paths are relative to a caller at the repository root. The core keeps `start`, `run`, `stop`, progress and run-result semantics; its `exportBest()` returns a string when results exist. The UI passes that string to `src/ui/download.js`. Display and download retain their distinct documented JSON shapes while sharing field conversion and selection. Tests import `createApp` directly and await its `ready` promise for sample initialization.

## Data flow and geometry

1. `parseRequirements(text)` normalizes flat/nested input, validates it, and returns `{ normalized, workspace }`. Every nonzero axis initially gets min/midpoint/max samples. Bounds are inclusive when a step lands on the maximum; arbitrary steps do not force an extra endpoint.
2. The UI fills defaults, retaining explicit overrides at Run. An explicit optimizer joint-limit option wins over normalized requirements. Loading the sample resets the controls.
3. `Optimizer` defaults to the C3 paired topology. Circular, C3 paired, and rectangular paired layouts are regenerated from symmetry-preserving parameters during mutation and crossover; Free layouts evolve individual anchors. See [layout topologies](./TOPOLOGIES.md) for exact invariants, parameter names, and bounds.
4. `finalizeLayout` clamps shared lengths and the independently chosen home height to effective bounds. It preserves the topology's geometry; it never derives a replacement height from rods and horns.
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

[Results documentation](./RESULTS.md) describes the geometric Jacobian proxies and their limitations. [Cycle documentation](./CYCLE_MODEL.md) describes the separate actual-rod equilibrium, gravity direction and torque/speed formulas. The old equal-load harmonic torque approximation is no longer used.

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

Run `npm test` for the checked-in regression suite, organized under `tests/model`, `workspace`, `optimization`, `ui`, `io`, `tooling` and `archive`. It covers normalization, controls, feasible/relaxed accounting, export, work limits, real event-loop yielding, cancellation, selection, cycle force balance and finite-difference speeds, and the archived bundle's custom-layout API. The workspace reference fixture was captured before the refactor at commit `97b7360`; it includes strict/soft/reachable results and capped samples. During the optimizer extraction, six seeded old/new runs across x/y/z cycles in strict/soft modes matched populations, scores, fronts, progress and JSON results exactly.

Run `npm run smoke` for the complete default sample and export checks. Browser smoke checks are manual: complete the sample, observe progress, cancel a longer run, inspect/download the result, and verify the archived canvas/animation separately. The local development server has HTTP checks for entry points, content types and path restrictions.

The search is a heuristic. Reproduction of numerical runs requires preserving a layout and settings or adding controlled random seeding; the public UI currently has no seed control. Exported geometry and metrics are a starting point for independent engineering analysis, not a manufacturing specification.
