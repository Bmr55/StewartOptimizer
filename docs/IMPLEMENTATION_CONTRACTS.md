# Shared implementation contracts for issue #36

> **Status:** planning baseline written before the #21–#34 workstreams landed.
> Later work (#54–#59) added trajectories, adaptive cycle sampling, servo
> envelopes, load sharing, compliance and payload support beyond this list. For
> the implemented behavior, read [FEATURES.md](./FEATURES.md) and
> [architecture.md](./architecture.md); this file is kept as the record of the
> shared names and boundaries those streams agreed on.

These contracts are the integration baseline for children #21–#34. Extend the
existing modules and fields. `src/contracts.js` owns version numbers, topology
names, metric names, directions, units, and failure categories. Changes to these
names need a coordinated migration. Issue #35 remains outside this plan.

## Layout, identity, and replay

- Internal geometry keeps `baseAnchors`, `platformAnchors`, `betaAngles`,
  `hornLength`, `rodLength`, `servoRangeRad`, and `homeHeight`. Lengths are mm;
  angles are radians. Export retains `base_anchors`, `platform_anchors`,
  `beta_angles`, `horn_length`, `rod_length`, `servo_range` in **degrees**, and
  `home_height`. Add `schema_version: 2`, `model_version: 2`, `topology`,
  `topology_parameters`, and `mounting` to the exported layout. The version
  fields identify format and effective physical model separately.
- A candidate has a stable `id` within a run. The selected candidate ID is
  shared by chart, simulator, JSON, and CAD. A new run invalidates the old
  selection. Geometry and view/pose changes do not silently mutate the retained
  optimizer candidate. The simulator may create an explicit editable copy.
- `topology` is one of `circular`, `c3_paired`, `rectangular_paired`, or `free`.
  New searches default to `c3_paired`. `topology_parameters` holds the
  generator's invariant parameters. A declared topology must match its anchor
  geometry; malformed metadata is an error. An import with no declared topology
  is `free`; do not infer or reshape its anchors.
- Accept a plain layout, a downloaded layout with `metadata`, or a displayed
  result wrapper with `layout`. Import validates all geometry before work,
  preserves well-formed numeric geometry exactly, and treats imported metrics
  as stale until reevaluation. An older layout without mounting data upgrades
  to the home-aligned socket model; return a visible migration note.
- `mounting` contains lower and upper direction overrides by leg. Lower
  directions live in each moving horn frame and point toward the platform rod
  endpoint. Upper directions live in the platform frame and point toward the
  horn tip. Missing directions derive from the home pose. Export effective
  directions and whether each was `derived` or `supplied`.
- JSON results include `run` with `id`, `status`, `partial`, and
  `effective_settings`. Settings include normalized requirements, effective
  bounds and limits, topology, seed, sampling strategy/count or grid steps,
  population, generations, mutation rate, objective set, and servo rating
  policy. This is enough to replay a run; report options actually used rather
  than defaults that were overridden.

## Evaluation and ranking

- Use the metric keys in `src/contracts.js` internally and their listed JSON
  names in `metadata`. Missing or nonfinite demand is unavailable (`null` in
  JSON), never zero. Percent coverage refers to sampled poses. Strict feasible
  coverage remains distinct from relaxed exploration. Maintain existing
  actual-rod cycle force balance and derivative-based servo speed.
- A candidate has `feasibility` with `homePoseSatisfied`,
  `sampledWorkspaceSatisfied`, `cycleSatisfied`, `passing`, and a set of failed
  categories from `FAILURE_CATEGORIES`. Keep leg/pose-specific violations as
  detail. Optional engineering condition and servo rating policies join these
  tests when supplied/enforced; mandatory numerical singularity rejection is
  always active. Preserve diagnostic candidates and their explicit failures.
- Apply the dimensionless rotary-actuator Jacobian in #24:
  `J_i = [L*u_i, r_i × u_i] / (u_i · h'_i)`, with actual rod direction `u_i`,
  anchor offset `r_i` about the current platform-anchor centroid, RMS anchor
  radius `L`, and horn-tip angle derivative `h'_i`. Reject rank deficiency,
  equivalent degenerate leverage, and `sigmaMin/sigmaMax <= 1e-10` at home,
  workspace, and cycle. A finite engineering condition limit is optional and
  applies at all three. Compact conditioning quality is the worst reciprocal
  condition among valid home/workspace samples; failed samples remain counted.
- Passing candidates precede failures. For failures, sort by fewer failed
  categories, then higher strict feasible coverage, then lower available
  demand. Initial passing selection uses lower torque, then lower speed, then
  higher conditioning quality. If none pass, initially select the first
  diagnostic candidate. Display/export a diagnostic label and failure details.
  CAD is available only when the selected candidate has valid home geometry.

## Execution and simulator boundaries

- The headless `Optimizer` remains the numerical owner. Browser execution uses
  one module worker. Main/worker messages are serializable and carry `runId`:
  `start` (effective options), `cancel`, `progress` (bounded snapshot),
  `checkpoint` (completed population), `result` (completed or partial), and
  `error`. The UI ignores any message with a stale ID. A failed worker startup
  offers an explicit yielding main-thread fallback. Checkpoints never include
  an incomplete candidate; cancellation retains the last completed population.
- Progress snapshots contain elapsed time, completed/total candidates,
  generation, front size, best candidate summary, actual completed pose work,
  and budgeted pose work. Publish to the UI at most 10 Hz. ETA is unavailable
  before three completed candidates and is always marked approximate.
- The active simulator separates a WebGL2 renderer, input/pose controller, and
  the existing active pose evaluator. A pose request returns both `requested`
  and last valid `accepted` pose plus evaluator diagnostics. The rendered
  mechanism is the accepted pose; a rejected request appears only as a dimmed
  ghost overlay. Invalid requests hold the last valid pose and pause an
  animation. Camera orbit is the default mouse mode; platform manipulation is
  explicit. Geometry controls (#33) and diagnostics (#34) consume this boundary
  in separate modules. Unsupported WebGL2 reports an actionable error while
  optimization remains available. The optional reachability cloud sweeps
  sampled translations at the requested rotation through the same evaluator
  in yielding, abortable chunks and reports evaluated samples only. The
  optional conditioning ellipsoid draws the singular vectors of the accepted
  pose's Jacobian translation block and evaluates no pose of its own. The
  optional loads overlay draws rod forces and servo torque utilisation that
  the controller solves for the accepted pose with the cycle model's
  `dynamicsAtPose`, statically or from the animation's analytic derivatives.
