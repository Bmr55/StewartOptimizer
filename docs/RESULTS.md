# Results, feasibility and export

## Coverage and constraint policy

`metadata.coverage` is 0-100 percent of sampled workspace poses satisfying all modeled geometry, servo travel, rod length (0.5 mm tolerance), and ball-joint constraints. The ball-joint model measures the angle between the horn direction and actual rod direction; it does not model an independently oriented socket or both physical joint mountings.

Strict mode is the default. In optional soft-ball-joint mode, `metadata.relaxed_coverage` also counts otherwise valid poses that exceed the joint angle limit. It is an exploration score, not a feasibility claim. No positions or joints are physically clamped. Invalid geometry, servo travel and rod-length failures remain excluded. Strict mode's relaxed coverage equals its feasible coverage.

`constraint_policy` records mode and effective ball-joint limit. `workspace_counts` includes reachable, unreachable, relaxedReachable and violationPoses. Counts cover the full sweep; stored example poses are reservoir samples capped at 200 per class. `workspace_stats.violationCounts` counts observed violations by type; evaluation may stop at the first hard failure. `violationRate` counts poses with any violation, not the number of individual leg failures.

The three feasibility flags report sampled workspace satisfaction, home-pose satisfaction and cycle satisfaction independently. They describe the implemented checks only. A candidate can be exported with false flags. Selection is highest feasible workspace coverage from the current Pareto front; it is not guaranteed to be the minimum-torque or cycle-valid candidate.

## Metric meanings

| Export metadata | Calculation / interpretation |
| --- | --- |
| coverage | Feasible sampled workspace percentage |
| relaxed_coverage | Separate workspace exploration percentage |
| torque | Peak sampled absolute servo torque in N m, or null for an invalid cycle |
| speed_demand | Peak sampled absolute servo speed in rad/s, or null for an invalid cycle |
| dexterity | Home-pose geometric singular-value ratio; zero if unavailable |
| stiffness | Mean eligible minimum singular value, falling back to the home value; a geometric proxy, not N/m |
| isotropy | Mean eligible smallest/largest singular-value ratio across feasible workspace poses |
| load_balance | Mean 1/(1 + standard deviation of normalized absolute vertical base-to-platform leg directions); a proxy, not the solved cycle loads |
| limit_margin | Product of nonnegative ball-angle headroom and violation-free pose fraction, clipped to [0,1] |
| fatigue | (mean maximum joint-angle utilization + mean servo-span utilization) * frequency * stroke in meters; a relative heuristic, not service life or a material fatigue model |

For the geometric proxies, each row uses a normalized base-to-platform vector and its cross product with the platform point in world coordinates. Translational and rotational columns mix scales (coordinates in mm), so these values depend on scale/origin conventions and are not calibrated rotary-actuator stiffness. The home metric keeps singular values above 1e-9; workspace means omit samples with minimum singular value <= 1e-8. Singular cases can therefore be understated; these scores are not proof of singularity avoidance. The cycle model uses actual rod directions and a separate equilibrium calculation, detailed in [CYCLE_MODEL.md](./CYCLE_MODEL.md).

## Downloaded layout

The download is `optimized_layout.json` with these top-level fields:

| Field | Meaning |
| --- | --- |
| base_anchors | Six [x,y,z] coordinates in the base frame, mm |
| platform_anchors | Six local moving-platform coordinates, mm |
| beta_angles | Six horn-plane orientation angles, radians |
| horn_length / rod_length | Shared horn and rod lengths, mm |
| servo_range | Shared [minimum, maximum] travel, degrees |
| home_height | Derived platform home offset along Z, mm |
| metadata | Metrics described above |
| cycle | Validity, axis, phase count, peak demands, per-servo peaks/model when valid, failure reason when invalid |
| feasibility | Independent sampled-workspace, home-pose and cycle flags plus scope text |
| constraint_policy | Strict/soft workspace policy and joint limit |
| workspace_counts / workspace_stats | Full sweep counters and aggregated statistics |
| run | completed/cancelled status, completed generation count, and partial flag |

The on-screen JSON has a `run`/`result` wrapper and includes bounded example poses; it is not byte-for-byte identical to the download. Pose samples store translation offsets in mm and Euler angles in radians. Internal servo angles/statistics are radians. Cycles are always checked strictly, even when workspace exploration is soft. Unavailable demand is null with an explicit reason, never an implicit zero.

JSON is the only implemented export format. Import into CAD or the archived simulator may require an adapter; no general compatibility guarantee is made.
