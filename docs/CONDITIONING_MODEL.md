# Actuator conditioning model

Every complete pose uses the same dimensionless rotary-actuator Jacobian for numerical singularity and optional engineering condition checks. For leg `i`:

`J_i = [L u_i, r_i × u_i] / (u_i · h'_i)`

`u_i` is the unit **actual rod direction** from horn tip to platform anchor. `r_i` is that anchor's offset from the centroid of all six platform anchors at the evaluated pose. `L` is the root-mean-square anchor radius about that centroid. `h'_i` is horn-tip displacement per radian of servo rotation, in millimeters:

`h'_i = H (-sin(alpha_i) cos(beta_i), -sin(alpha_i) sin(beta_i), cos(alpha_i))`

The first three columns use translation divided by `L`; the last three use rotation about the anchor centroid. Every numerator and the leverage denominator have millimeter units, so Jacobian entries and the singular-value ratio are dimensionless. `J_i` maps a centroid twist with normalized translation to servo angle change. It is independent of base-frame origin and uniform length scale. The pose evaluator returns its six rows as `jacobianRows` and records the centroid, radius, singular values, condition, and reciprocal in `conditioning`.

## Feasibility policy

One-sided Jacobi SVD evaluates the matrix directly. Forming `JᵀJ` would square the smallest singular value and lose accuracy near the declared boundary. No zero singular value is filtered out.

The mandatory numerical test rejects rank deficiency, nonfinite/unavailable matrices, leverage with `|u_i · h'_i|/H ≤ 10⁻¹⁰`, and `σmin/σmax ≤ 10⁻¹⁰` **including equality**. An unavailable anchor radius or rod direction is also invalid. Numerical failures produce a `numericalSingularity` violation with a reason and any available condition values. An invalid geometry pose remains a geometry failure; no condition pass is inferred from incomplete kinematics.

`conditionLimit` is optional and is set only through the headless `Optimizer` constructor (or `evaluatePose`/`computeWorkspace` options); the browser UI has no control for it and the requirements JSON has no field for it, so browser runs always use `null`. The simulator diagnostics panel displays whichever limit its loaded options carry. When present, it must be finite and at least 1. A pose passes the engineering check when its condition is **at most** this limit; equality passes. Exceeding it produces a distinct `conditionLimit` violation. The same limit is applied to home, every workspace sample, and every cycle phase. Cycle demand still uses the actual-rod force balance and servo-angle derivative, with its separate equilibrium and transmission singularity checks.

`geometricallyReachable` reports completion of inverse kinematics, servo travel, rod length, and socket availability. `mechanicallyReachable` additionally requires both socket deflections to pass. `reachable` also requires numerical and optional engineering conditioning. Relaxed exploration can ignore only ball-joint excess; it never ignores conditioning. `feasibility.conditionSatisfied`, sampled coverage, failure categories, workspace `conditioningCounts`, cycle violations, and the exported `constraint_policy` retain failures independently.

## Metrics

`conditioningQuality` is the **lowest reciprocal condition** among valid home and sampled workspace poses. It is `null` when none qualifies; failed samples remain in workspace failure counts and do not silently count as passing. `dexterity` is the valid home reciprocal condition, `isotropy` is the mean reciprocal among valid workspace samples, and `stiffness` is a dimensionless minimum-singular-value proxy. These are kinematic metrics, not measured physical stiffness. `condition` reports the home condition when available. Export metadata maps `conditioningQuality` to `conditioning_quality`; effective limits, condition summaries, and violations are retained with the result.

The historical symmetric joint fixture is actuator-rank deficient at home. Its old geometric proxy dropped zero modes. With the mandatory check, the home pose fails and eight of its nine sampled poses pass at the 52-degree socket limit. This is an intentional physical-model change; the preserved fixture still verifies unchanged inverse kinematics, horn tips, and actual rod vectors.

## Translation conditioning ellipsoid

The simulator's **Conditioning ellipsoid** overlay draws the translation part of `J` as a shape. `translationSingularSystem(rows)` in `src/model/conditioning.js` takes the first three columns of the six `jacobianRows` (the 6x3 translation block `J_t`) and runs the same one-sided Jacobi rotations on it while accumulating them, returning the three singular values, largest first, each with its right singular vector: a unit world translation direction `v_k` with `J_t v_k` of length `σ_k`. The vectors are the eigenvectors of the upper-left 3x3 block of `JᵀJ`, but the block is never formed. It returns `null` for missing, non-6x6, nonfinite, all-zero-translation or nonconvergent rows.

A long axis is a direction in which a small platform translation moves the servos a lot, so the servos drive and hold it firmly; a short axis is one they barely affect, so motion along it is poorly controlled and weakly resisted. As a pose approaches a translational singularity the smallest `σ` falls toward zero and the ellipsoid flattens. The overlay scales each half-axis by `σ_k / σ_1`, so only proportions are shown. For a D3-symmetric layout at home the two horizontal values are equal and the vertical one is along Z; with near-vertical rods it is the largest, so the ellipsoid is not a sphere even for a well-conditioned layout.

The helper only reads the rows. `singularValuesOneSided`, `assessJacobian` and every condition, reciprocal, violation and metric above are unchanged, so optimizer results and seeded runs do not move. The translation block alone cannot show rotational singularities; the overlay's colour therefore follows the full six-axis reciprocal condition. See [SIMULATOR.md](./SIMULATOR.md#scene-builders-and-overlays) for the drawing and colour scale.
