# Cycle demand model

`cycle_mm` is peak-to-peak translation about the home pose along `cycle_axis` (x, y or z). The evaluator samples 64 equally spaced phases of a sinusoid, including displacement extrema. Zero frequency or zero stroke evaluates one stationary home pose. Exported `torqueNm` is the largest sampled absolute torque across the six servos (N m); `speedRadPerSec` is the largest sampled absolute servo angular velocity (rad/s). Per-servo peaks are also exported. These sampled maxima need not bound the continuous trajectory.

For amplitude A in meters and angular frequency w = 2 pi f, displacement is A sin(phase), velocity is w A cos(phase), and acceleration is -w^2 A sin(phase). Gravity acts along -Z regardless of cycle axis. The required supporting force is m times (acceleration + [0, 0, 9.81]).

At each pose, inverse kinematics gives horn angles and actual rod directions u. Solve the six force/moment equilibrium equations for rod axial forces, using moments about the moving platform origin and zero external payload moment. With horn-tip derivative h-prime = dh/dalpha in meters, servo torque is abs(rod force * dot(u, h-prime)); servo speed is abs(dot(u, platform velocity) / dot(u, h-prime)).

The assumptions are rigid massless rods, a point payload centered at the moving origin, ideal joints and no actuator/platform/rod inertia or friction. There is no motor capacity, collision, compliance, thermal or material-strength check. The workspace stiffness/fatigue proxies are separate from this force-balance model. Near-singular designs can have very large demand; the solver's numerical singularity check is not an engineering margin.

Cycle evaluation always enforces the modeled mechanical limits, including both socket deflections in their moving mounting frames, the mandatory actuator numerical-singularity check, and any supplied engineering condition limit, even during soft workspace exploration. An invalid pose reports its failed sample and specific violations. An invalid pose or singular equilibrium/transmission yields `valid: false`, a reason, and null demand values. It must not be interpreted as zero demand. Work estimates include the maximum cycle samples and one home-pose evaluation per candidate; early failure can perform fewer actual checks.

Run `npm test` for equilibrium, axis-dependence, static-gravity and finite-difference speed checks. Hardware validation is still required before using these estimates for component selection.
