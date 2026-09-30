# Cycle demand model

Model identity: `cycle-newton-euler-v1` (`cycle.modelVersion`), independent of the layout/joint `model_version`.

## Trajectories

A cycle is a periodic trajectory about the home pose. Two inputs are accepted:

- **Legacy single-axis cycle** (`cycle_mm`, `frequency_hz`, `cycle_axis`): peak-to-peak translation along x, y or z. It is exactly the sinusoid with amplitude `cycle_mm / 2`, zero phase. Results report `trajectorySource: "legacy-cycle"` and keep `axis`.
- **Supplied trajectory** (`payload.trajectory`): a common-frequency multi-axis sinusoid. Each component names an axis (`x`, `y`, `z` with `amplitude_mm`; `rx`, `ry`, `rz` with `amplitude_deg`) and an optional `phase_deg`. Component value is `A sin(2 pi f t + phase)`. It replaces the legacy fields; supplying both is an error. Results report `trajectorySource: "supplied"` and `axis: null`.

`trajectoryId` is a canonical text identity, for example `sinusoid-v1:f=2Hz;z:15mm@0deg`, recorded in the result and in `run.effective_settings.cycleModel`. Zero frequency or all-zero amplitudes evaluate one stationary home pose (`sampling.status: "stationary"`).

## Sampling and convergence

`run.effective_settings.cycleSampling` selects the time schedule over one period [0, T):

- `{ "strategy": "uniform", "samples": 64 }` evaluates equally spaced times from t = 0. This is the original schedule; it reports `status: "fixed"` and makes no convergence claim. Replays of runs saved without `cycleSampling` use it.
- `{ "strategy": "adaptive", "initialSamples": 32, "maxSamples": 256, "tolerance": 0.005, "inconclusivePolicy": "enforced" }` is the default for new runs. It evaluates `initialSamples` equally spaced times and then the midpoint of every initial interval, so the first 64 samples are exactly the legacy grid. For each bisected interval, the unresolved variation of each monitored quantity is estimated as one quarter of the midpoint's departure from linear interpolation. The monitored quantities are signed servo torque and speed (scaled by the current peak across servos), reciprocal conditioning (scaled by the worst sampled value, floored at 1e-3), lower and upper socket angles (scaled by their limits), and servo angles (scaled by the travel span). The interval with the largest normalized estimate is bisected next; ties go to the earliest interval. Refinement stops as `converged` when every estimate is at most `tolerance`, or as `budget-limited` when `maxSamples` is reached first.

The result's `sampling` object records the policy, `status`, `evaluatedSamples`, `converged` (true, false for budget-limited, null when not assessed) and `maxUnresolved`. A failed cycle reports `violated` for an observed modeled constraint (joint, travel, conditioning) and `unavailable` for a singular or nonfinite calculation. A budget-limited cycle is inconclusive: with the default enforced policy it adds the `cycle_convergence` failure category; with `inconclusivePolicy: "advisory"` it stays visible without disqualifying the candidate. The UI offers the default, a finer preset (1,024 samples, 0.1%) and the fixed 64-phase schedule.

Sampled convergence is a numerical estimate, not a mathematical guarantee of continuous feasibility: a feature narrower than the inspected intervals can still be missed. No interval-bound method is applied. Nonuniform samples carry trapezoidal periodic time weights (`periodicSampleWeights`) for time-averaged metrics.

### Frames and units

The base frame has +Z up; gravity is `[0, 0, -9.81]` m/s^2. The moving origin is the platform-frame origin (the frame of `platform_anchors`), located at `[x, y, z + home_height]`. Orientation uses the pose evaluator's Euler convention `R = Rz(rz) Ry(ry) Rx(rx)`. Internally, positions are converted from mm to m and angles to radians; derivatives are SI and expressed in base-frame components at the moving origin.

Euler-angle rates are not the physical angular velocity. With Euler rates `(a', b', c')` for `(rx, ry, rz)`, the base-frame angular velocity is `omega = a' Rz Ry e_x + b' Rz e_y + c' e_z`, and the angular acceleration is its analytic time derivative. Tests check both against finite differences of `R(t)`.

## Mass properties

`mass_kg` is the combined moving platform and payload mass. Optional payload fields define a rigid body:

| Field | Meaning |
| --- | --- |
| `center_of_mass_mm` | Center of mass in the platform frame, relative to the moving origin, mm |
| `inertia_kg_m2` | Inertia tensor about the center of mass in platform axes, kg m^2. A 3x3 matrix or `{ixx, iyy, izz, ixy, ixz, iyz}`; off-diagonal values are tensor entries (I_xy = -integral of x y dm) |
| `external_force_n` | Base-frame force applied to the moving body at the center of mass, N |
| `external_moment_nm` | Base-frame moment applied to the moving body, N m |

The tensor must be finite, symmetric, positive semidefinite, and satisfy the principal-moment triangle inequality; a nonzero tensor requires positive mass. With none of these fields, the result reports `massModel: "legacy-point"`: a point mass at the moving origin, which reproduces the earlier model exactly. Any supplied field selects `massModel: "rigid-body"`.

## Newton-Euler balance

With `c = R c_local`, `a_c = a + alpha x c + omega x (omega x c)` and `I_w = R I R^T`, the rods must supply the wrench about the moving origin

- `F = m (a_c - g) - F_ext`
- `M = I_w alpha + omega x (I_w omega) - M_ext + c x F`

At each sample, inverse kinematics gives horn angles and actual rod directions `u_i` (horn tip toward platform). The six equilibrium equations `sum f_i [u_i; r_i x u_i] = [F; M]`, with `r_i = R p_i` in meters, are solved for rod axial forces `f_i`. A positive rod force pushes the platform along `u_i`.

## Servo motion and torque

The physical rotary mapping is `G = D^-1 A`, with `A_i = [u_i^T, (r_i x u_i)^T]`, `D_ii = u_i . h'_i` and `h'_i = dh/dalpha` (m/rad). For the moving-origin twist `V = [v; omega]`, the signed servo rate is `alphaDot = G V`. Signed servo acceleration `alphaDDot = G VDot + GDot V` is evaluated from the second derivative of the rod-length constraint, `alphaDDot_i = (|rhoDot|^2 / L + u . qDDot + (u . h) alphaDot^2) / (u . h')`, using `h'' = -h`. Signed servo torque is `tau_i = f_i D_ii` in the +alpha direction, so `sum tau_i alphaDot_i = F . v + M . omega` (virtual power). `G` is unnormalized and SI; the dimensionless centroid-referenced Jacobian in [CONDITIONING_MODEL.md](./CONDITIONING_MODEL.md) remains a separate conditioning measure.

Results report the largest sampled absolute torque (`torqueNm`, N m), speed (`speedRadPerSec`, rad/s) and acceleration (`accelerationRadPerSec2`, rad/s^2), per-servo peaks, and `limiting` entries giving the servo, sample index, time and signed value of each peak. These sampled maxima need not bound the continuous trajectory.

## Assumptions and omissions

Rods and horns are ideal, rigid and massless; joints are ideal and frictionless. The moving body is one rigid body with the supplied mass properties. Omitted: rod and horn inertia, actuator rotor inertia, gearbox and friction, motor torque-speed limits and dynamics, compliance, collision, thermal and material-strength checks. This is not a complete multibody model. The workspace stiffness/fatigue proxies are separate from this force balance. Near-singular designs can have very large demand; the solver's numerical singularity check is not an engineering margin.

Cycle evaluation always enforces the modeled mechanical limits, including both socket deflections in their moving mounting frames, the mandatory actuator numerical-singularity check, and any supplied engineering condition limit, even during soft workspace exploration. An invalid pose reports its failed sample, time and specific violations. An invalid pose or singular equilibrium/transmission yields `valid: false`, a reason, and null demand values; it must not be interpreted as zero demand. Work estimates reserve the policy's maximum cycle samples (`maxSamples`, or `samples` for a uniform schedule) and one home-pose evaluation per candidate; convergence or early failure can perform fewer actual checks, and progress counts the samples actually evaluated. Cancellation is checked before every sample, including during refinement.

Run `npm test` for adaptive-versus-dense agreement, a between-sample violation missed by the 64-phase grid, legacy reproduction, equilibrium, offset-gravity, pure-rotation and combined-motion Newton-Euler, finite-difference rate/acceleration and virtual-power checks. Hardware validation is still required before using these estimates for component selection.
