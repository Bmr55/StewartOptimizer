> Historical design proposal, not current implementation documentation. Some formulas, defaults, dependencies and claimed features below are obsolete or unimplemented. See the [current README](../../../../README.md), [architecture](../../../../docs/architecture.md), and [results/model limitations](../../../../docs/RESULTS.md) for supported behavior. Preserve this document as research/roadmap context only.

# Stewart Platform Optimizer — Comprehensive Architecture, Math, and Reproduction Guide

## 1. Purpose and Scope

This document merges the finalized specification for the generalized Stewart platform optimizer with the detailed reproduction manual so that a new engineering team—or another autonomous coding assistant—can rebuild the entire toolchain with complete confidence. It:

- States the overall mission of the optimizer and the classes of design problems it must solve.
- Enumerates every input requirement, optimization variable, and output artifact.
- Documents the repository structure, runtime environment, and integration points between UI and numerical kernels.
- Derives the kinematic and dynamic equations, Jacobian-based dexterity and stiffness metrics, and workspace computation formulas in full mathematical detail.
- Describes the multi-objective optimization strategy (GA, NSGA-II, PSO hybridization, Monte Carlo sampling) along with implementation pseudocode and rebuild steps.

The intent is for no ambiguity to remain: all math, engineering rationales, and code restoration tasks are presented explicitly.

---

## 2. Repository Topology and Execution Environment

### 2.1 Directory Layout

```
StewartOptimizer/
├── index.html                  # Single-page UI that hosts and orchestrates the optimizer
├── optimizer.js                # Optimization controller (population management, metrics, Pareto storage)
├── workspace.js                # Workspace sweep, constraint enforcement, and pose evaluation
├── Raw Information/            # Legacy specs, schemas, and archived layouts from the previous project
├── OPTIMIZER_REPRODUCTION.md   # Historical reproduction notes
└── Additional_Repo_Stuff/
    ├── README.md               # Quick guide to the supplemental assets
    ├── docs/
    │   ├── Stewart_Optimizer_Quick_Start.md
    │   ├── Stewart_Optimizer_Final_Spec_v3.md
    │   └── Stewart_Optimizer_Comprehensive_Guide.md   # ← this file
    ├── schemas/
    │   └── Stewart_Requirements_Schema_v2.json
    └── examples/               # Reference layouts (AYVA, Circular, Hexagonal)
```

Key takeaways:

- `index.html` loads the optimizer modules using native ES module syntax, binds HTML controls to optimizer settings, and exposes buttons for importing layouts, starting the search, and exporting the best candidate.
- `optimizer.js` holds the asynchronous optimization lifecycle: population initialization, fitness evaluation (coverage, torque, dexterity, stiffness), Pareto ranking, and export helpers. The heavy numerical loops are scaffolded and can be restored using the formulas listed later in this document.
- `workspace.js` is responsible for enumerating the translation and rotation workspace, calling the inverse kinematics routines, and enforcing joint, torque, and payload constraints for each pose.
- `Raw Information/` preserves the legacy Stewart simulator math, schemas, and UI requirements that informed the present specification.

### 2.2 Runtime Dependencies

1. Host `index.html` via any static HTTP server (for example, `python -m http.server 8000`) or open it directly in a modern browser that supports ES modules.
2. Ensure the following legacy libraries (bundled in `Raw Information/Stuff from Old Project/`) are available:
   - `p5.min.js` — drawing helper functions for visualization.
   - `quaternion.min.js` — quaternion algebra used for pose conversions.
   - `stewart.min.js` — legacy Stewart class exposing `initCustom`, inverse/forward kinematics, and workspace helper routines.
3. The optimizer is dynamically imported when the user presses **Run Optimization**, so no bundler is required.

---

## 3. Problem Goal

The optimizer must design, simulate, and evaluate **arbitrary Stewart platform configurations**. It is expected to:

- Accept structured JSON requirements without assuming any predefined base or platform pattern (circular, hexagonal, etc.).
- Generate base and platform anchor geometries automatically, subject to mechanical, workspace, and payload constraints.
- Evaluate the resulting designs against translational/rotational workspace coverage, payload handling, joint limits, dynamic torque, stiffness, and dexterity.
- Execute multi-objective optimization to produce a **Pareto front** that captures the trade-offs among competing performance metrics for downstream engineering decisions.

---

## 4. Input Specification — Requirements Schema (v2)

The optimizer consumes a JSON document validated against `Stewart_Requirements_Schema_v2.json`. All length inputs are in millimetres, angles in degrees (unless stated otherwise), and mass in kilograms.

### 4.1 Required Fields

- `mass_kg` — Payload mass \(m\).
- `cycle_mm` — Peak-to-peak oscillation amplitude \(s\) of the dominant motion cycle.
- `frequency_hz` — Excitation frequency \(f\).
- `cycle_axis` — Principal axis of cyclic motion: one of `"x"`, `"y"`, or `"z"`.
- Translational workspace ranges: `x_range_mm`, `y_range_mm`, `z_range_mm` (each a `[min, max]` pair).
- Rotational workspace ranges: `rx_range_deg`, `ry_range_deg`, `rz_range_deg`.

### 4.2 Optional Fields (Become Optimization Variables When Omitted)

- `ball_joint_max_deg` — Default \(45^{\circ}\).
- `servo_max_deg` — Default \(90^{\circ}\) symmetric about zero.
- `rod_length_bounds_mm` — Default \([100, 400]\) mm.
- `horn_length_bounds_mm` — Default \([20, 120]\) mm.
- `servo_travel_bounds_deg` — Default \([-120, 120]\) degrees.

When these parameters are absent, the optimizer treats them as free design variables and will explore them within practical engineering bounds during the search process.

---

## 5. Layout Geometry and Coordinate Conventions

### 5.1 Anchor Sets

Each candidate Stewart layout is encoded as a JSON object containing:

- Base anchors \(\mathbf{B}_k = (B_{kx}, B_{ky}, B_{kz})\) for \(k = 1,\ldots,6\) expressed in the global base frame.
- Platform anchors \(\mathbf{P}_k = (P_{kx}, P_{ky}, P_{kz})\) defined in the moving platform frame prior to any motion.
- Servo axis orientation angles \(\beta_k\) (radians) that tilt each horn relative to the servo frame \(z\)-axis.
- Servo horn length \(h\) and rod length \(d\). (The system can support leg-specific values if desired, but the baseline assumes uniform lengths to simplify optimization.)
- Optional servo range \([\alpha_{\min}, \alpha_{\max}]\) and payload descriptors such as `mass_kg`, `stroke_mm`, and `frequency_hz` for convenience.

### 5.2 Pose Parameterization

A commanded pose of the moving platform consists of:

- Translation vector \(\mathbf{t} = (x, y, z)\).
- Intrinsic ZYX Euler rotation \((\psi, \theta, \phi)\) corresponding to yaw, pitch, and roll about global axes. The quaternion library converts this triple into a unit quaternion \(q = (w, x_q, y_q, z_q)\). The quaternion is used to construct a rotation matrix \(R(q)\), discussed in Appendix A.

The rotated platform anchors are obtained via:
\[
\mathbf{p}'_k = \mathbf{t} + R(q)\,\mathbf{P}_k.
\]

### 5.3 Mechanical Axes and Joint Limits

- Servo axis orientation \(\beta_k\) determines the planar rotation of each horn. Horns lie in a plane containing the servo axis; the axis is tilted by \(\beta_k\) around the servo frame.
- Ball-joint angular deviation is limited to `ball_joint_max_deg`. The relative angle between the rod direction and its nominal axis is monitored to enforce this limit.
- Servo angular travel is restricted to the provided or default `servo_travel_bounds_deg` range.

---

## 6. Inverse Kinematics and Servo Geometry

For each leg index \(k\), the servo angle \(\alpha_k\) is derived from the four-bar linkage formed by the servo horn, rod, and base/platform anchors.

1. Compute the rotated and translated platform anchor:
   \[
   \mathbf{p}'_k = \mathbf{t} + R(q)\,\mathbf{P}_k.
   \]

2. Form the vector from the base anchor to the moved platform anchor:
   \[
   \mathbf{l}_k = \mathbf{p}'_k - \mathbf{B}_k = (l_{kx}, l_{ky}, l_{kz}).
   \]

3. Evaluate the auxiliary scalars that encode horn geometry:
   \[
   e_k = 2 h\, l_{kz},\qquad
   f_k = 2 h\,(\cos\beta_k\, l_{kx} + \sin\beta_k\, l_{ky}),\qquad
   g_k = \lVert \mathbf{l}_k \rVert^2 - (d^2 - h^2).
   \]

4. Solve for the servo rotation angle by enforcing the four-bar constraint:
   \[
   \alpha_k = \arcsin\!\left(\frac{g_k}{\sqrt{e_k^2 + f_k^2}}\right) - \operatorname{atan2}(f_k, e_k).
   \]

The above expressions are valid for arbitrary base/platform geometries and servo orientations so long as horn length \(h\) and rod length \(d\) are positive and \(\sqrt{e_k^2 + f_k^2} \neq 0\).

### 6.1 Servo Limit Checking

Given the allowable range \([\alpha_{\min}, \alpha_{\max}]\), the layout satisfies servo constraints iff:
\[
\alpha_{\min} \le \alpha_k \le \alpha_{\max}\quad \text{for all } k.
\]

### 6.2 Ball-Joint Angular Deviation

Let \(\hat{z}_k\) denote the nominal axis of the servo-mounted spherical joint. The actual rod direction is the unit vector \(\hat{u}_k = \mathbf{l}_k / \lVert \mathbf{l}_k \rVert\). The angular deviation is:
\[
\theta_k = \arccos\!\left( \frac{\mathbf{l}_k \cdot \hat{z}_k}{\lVert \mathbf{l}_k \rVert} \right).
\]
A leg passes the ball-joint constraint when \(\theta_k \le \theta_{\max}\), where \(\theta_{\max}\) corresponds to `ball_joint_max_deg`. When `ballJointClamp` is false, exceeding the limit classifies the pose as unreachable.

---

## 7. Jacobian, Dexterity, and Stiffness Metrics

### 7.1 Leg Jacobian Construction

Define the unit leg direction:
\[
\hat{u}_k = \frac{\mathbf{l}_k}{\lVert \mathbf{l}_k \rVert}.
\]

The Plücker coordinates of each leg combine its direction and moment:
\[
\mathbf{m}_k = \mathbf{p}'_k \times \hat{u}_k.
\]

The instantaneous relationship between platform twist \(\dot{\mathbf{x}} = (\dot{x}, \dot{y}, \dot{z}, \omega_x, \omega_y, \omega_z)\) and leg length rates \(\dot{l}_k\) is captured by the \(6 \times 6\) Jacobian matrix \(J\) whose rows are:
\[
J_k = \begin{bmatrix} \hat{u}_k^\top & \mathbf{m}_k^\top \end{bmatrix}.
\]

Stacking all six rows yields:
\[
\dot{\mathbf{l}} = J \, \dot{\mathbf{x}}.
\]

### 7.2 Dexterity and Conditioning

Compute the singular values of \(J\): \(\sigma_1 \geq \sigma_2 \geq \cdots \geq \sigma_6 > 0\). The condition number is:
\[
\kappa(J) = \frac{\sigma_{\max}}{\sigma_{\min}} = \frac{\sigma_1}{\sigma_6}.
\]
A well-conditioned manipulator has \(\kappa(J)\) close to 1. The optimizer uses the reciprocal of the condition number as a dexterity score:
\[
\text{dexterity} = \frac{1}{\kappa(J)} = \frac{\sigma_{\min}}{\sigma_{\max}}.
\]

### 7.3 Stiffness Proxy

In the absence of a full structural finite-element model, use the smallest singular value as a proxy for stiffness/compliance:
\[
\text{stiffness} = \sigma_{\min}.
\]
Large \(\sigma_{\min}\) indicates that small forces are required to produce motions, implying higher stiffness against external disturbances.

### 7.4 Singularity and Workspace Boundaries

When \(\det(J) = 0\) or equivalently \(\sigma_{\min} \rightarrow 0\), the manipulator approaches a singular configuration. Such poses should either be excluded from the reachable workspace or heavily penalized during optimization to avoid unstable behavior.

---

## 8. Dynamic Load, Torque, and Speed Estimation

### 8.1 Harmonic Motion Acceleration

For a payload of mass \(m\) oscillating sinusoidally with peak-to-peak stroke \(s\) at frequency \(f\), the peak acceleration magnitude along the cycle axis is approximated by:
\[
a = (2\pi f)^2 \left(\frac{s}{1000}\right),
\]
where the division by 1000 converts stroke from millimetres to metres.

### 8.2 Force Distribution and Servo Torque

Assuming load sharing is uniform across all six legs, the force per servo is:
\[
F = \frac{m (g + a)}{6},
\]
with gravitational acceleration \(g = 9.81\,\text{m/s}^2\). The required torque at the servo horn (length \(h\)) is then:
\[
\tau = F h.
\]
This torque estimate is compared against servo capabilities; exceeding limits incurs penalties or invalidates the configuration.

### 8.3 Servo Speed Requirement

Given the oscillation frequency, servo angular velocity demand can be estimated by differentiating the horn angle over the cycle or by approximating:
\[
\omega_{\text{servo}} \approx 2\pi f \cdot \Delta\alpha,
\]
where \(\Delta\alpha\) is the swing amplitude of the servo angle measured in radians. This value can be compared to manufacturer speed limits to ensure dynamic feasibility.

---

## 9. Workspace Evaluation

### 9.1 Discrete Pose Grid

The workspace evaluation enumerates the Cartesian product of the translational and rotational ranges. For each degree of freedom, define:

- Translation sequences: \(x_i, y_j, z_k\) derived from `[min, max, step]` triples.
- Rotation sequences: \(\psi_p, \theta_q, \phi_r\) from the Euler ranges.

Total poses scanned:
\[
N_{\text{total}} = N_x N_y N_z N_{\psi} N_{\theta} N_{\phi}.
\]

### 9.2 Pose Evaluation Procedure

For each pose:

1. Construct quaternion \(q\) from Euler angles (see Appendix A) and compute rotated anchors.
2. Solve the IK equations to obtain \(\alpha_k\) and rod vectors \(\mathbf{l}_k\).
3. Check constraints:
   - Servo limits: \(\alpha_{\min} \le \alpha_k \le \alpha_{\max}\).
   - Rod length feasibility: \(\lVert \mathbf{l}_k \rVert\) close to design rod length \(d\) within tolerance or within `rod_length_bounds_mm` if lengths are variable.
   - Ball-joint angle \(\theta_k \le \theta_{\max}\).
   - Torque: \(\tau \le \tau_{\text{limit}}\) if servo limit is specified.
   - Collision checks: ensure rods do not intersect the base or platform (implementation detail left to mechanical constraints module).
4. Mark the pose as reachable only when all constraints pass.
5. Record violating conditions for analytics and debugging.

### 9.3 Coverage Metric

Let \(N_{\text{reachable}}\) be the number of poses satisfying all constraints. Workspace coverage is:
\[
\text{coverage} = 100 \times \frac{N_{\text{reachable}}}{N_{\text{total}}}.\tag{1}
\]

The optimizer maximizes this percentage within the multi-objective framework.

### 9.4 Reachability Ledger

Maintain arrays of reachable and unreachable poses to facilitate visualization (heat maps, scatter plots) and to identify constraint bottlenecks (e.g., servo saturation versus ball-joint limit violations).

---

## 10. Optimization Problem Definition

### 10.1 Decision Variables

The optimizer may vary:

- Base anchor coordinates \(\mathbf{B}_k\).
- Platform anchor coordinates \(\mathbf{P}_k\).
- Servo orientation angles \(\beta_k\).
- Horn lengths \(h\) and potentially leg-specific \(h_k\).
- Rod lengths \(d\) or leg-specific \(d_k\).
- Servo travel bounds \([\alpha_{\min}, \alpha_{\max}]\) when not specified.
- Ball-joint limits when flagged as design variables.

### 10.2 Objective Vector

For each candidate layout, compute the multi-objective score:
\[
\mathbf{f}(\text{layout}) = \Big[
\underbrace{\text{coverage}}_{\text{maximize}},
\underbrace{\text{dexterity}}_{\text{maximize}},
\underbrace{\text{stiffness}}_{\text{maximize}},
\underbrace{-\tau}_{\text{maximize (minimize torque)}},
\underbrace{\text{load sharing metric}}_{\text{maximize}}
\Big],
\]
where the load-sharing term penalizes imbalanced force distributions across legs. Additional penalty terms (servo limit violations, singularities) can be appended as negative contributions.

### 10.3 Constraints

- Equality constraints: mechanical compatibility equations inherent to the IK model.
- Inequality constraints: servo angle bounds, torque limits, horn/rod length bounds, ball-joint angles, collision checks, stiffness minimum thresholds.
- Hard constraints should result in infeasible classification; soft constraints may be converted into penalty terms.

---

## 11. Optimization Algorithms and Workflow

### 11.1 Initialization

1. Parse the requirements JSON and seed the `Stewart` simulator with baseline geometry (user-imported or generated).
2. Generate an initial population of \(N\) layouts. Common strategies:
   - Include the baseline layout verbatim to ensure a valid starting point.
   - Randomize additional layouts by jittering anchor coordinates, horn/rod lengths, and servo orientations within allowable bounds using Gaussian or uniform noise.
   - Optionally, perform Latin Hypercube Sampling or Monte Carlo sampling for broader coverage of the design space.

### 11.2 Fitness Evaluation

For each layout in the population:

1. Run `computeWorkspace` to obtain coverage percentage and violation log.
2. Compute Jacobian at key poses (nominal home pose and select workspace points) to derive dexterity and stiffness metrics.
3. Evaluate dynamic torque using Section 8 formulas and, if needed, servo speed requirements.
4. Assess load sharing by comparing leg forces \(F_k\) derived from inverse statics (if implemented) or by verifying torque uniformity.
5. Aggregate results into \(\mathbf{f}(\text{layout})\) and store alongside the layout definition.

### 11.3 Multi-Objective Genetic Algorithm (NSGA-II)

Implement NSGA-II as follows:

1. **Fast non-dominated sorting:** partition the population into Pareto fronts based on dominance relationships. A layout \(A\) dominates \(B\) if it is no worse in all objectives and strictly better in at least one.
2. **Crowding distance calculation:** within each front, compute the crowding distance to preserve diversity.
3. **Selection:** choose parents using binary tournament selection biased toward low rank (better fronts) and high crowding distance.
4. **Crossover:** combine parent geometries—swap subsets of base anchors, interpolate horn lengths, average servo orientations—to create offspring. Enforce constraints during crossover to maintain feasibility.
5. **Mutation:** perturb offspring using Gaussian noise scaled by `mutationRate`. Clip values to physical bounds (e.g., rod length limits).
6. **Elitism:** merge parents and offspring, rerun non-dominated sorting, and truncate to the original population size \(N\) while retaining the best fronts.

### 11.4 Hybrid Enhancements

- **Monte Carlo Sampling:** periodically introduce random individuals sampled from broad distributions to avoid premature convergence and explore disconnected Pareto regions.
- **Particle Swarm Optimization (PSO) Hybrid:** maintain a swarm overlay where each particle updates its velocity toward personal and global bests (based on coverage or combined score). Use PSO steps to inform GA mutation or crossover parameters.
- **Local Search:** after GA iterations, perform gradient-free refinements (e.g., Nelder–Mead simplex) around promising layouts to fine-tune anchors.

### 11.5 Optimization Loop Pseudocode

```python
requirements = load_json("requirements.json")

population = initialize_random_layouts(N=200, requirements=requirements)

for gen in range(max_generations):
    fitness_scores = []
    for candidate in population:
        coverage, violations = compute_workspace(candidate, requirements)
        jacobian_metrics = evaluate_jacobian(candidate)
        torque = compute_torque(candidate, requirements)
        load_balance = compute_load_sharing(candidate)

        # Compose objective vector (negate torque to convert to maximization)
        fitness_scores.append([
            coverage,
            jacobian_metrics.dexterity,
            jacobian_metrics.stiffness,
            -torque,
            load_balance
        ])

    population = nsga2_step(population, fitness_scores, mutation_rate)

pareto_front = extract_pareto_front(population, fitness_scores)
export_layouts(pareto_front, "optimized_layouts.json")
```

### 11.6 Output Artifacts

The optimizer must export:

- Final Pareto-optimal layouts (JSON and CSV).
- Associated metrics per layout: coverage, torque demand, dexterity, stiffness, load sharing, violation counts.
- Visualizations: 3D geometry renderings, workspace heat maps, and Pareto plots.

---

## 12. UI Integration and Execution Lifecycle

1. **User Input:** The UI allows importing a reference layout, entering payload/workspace specifications, and configuring GA parameters (population size, generations, mutation rate, clamp behaviors).
2. **Start Routine:** When the user presses **Run Optimization**, the UI prevents concurrent runs, initializes the optimizer with the current settings, and triggers the asynchronous optimization loop.
3. **Progress Feedback:** Status text is updated as generations progress. Violations or errors are reported in a log panel for transparency.
4. **Completion:** Upon convergence or reaching the generation limit, the optimizer resolves the promise, the UI displays the best layout JSON, and a download button becomes active.
5. **Export:** `exportBest()` serializes the top-ranked layout and forces a file download so results can be archived.

---

## 13. Reproduction Checklist for a New Implementation

1. **Rebuild the UI:** Mirror `index.html` to provide layout import/export, optimizer controls, and status outputs. Ensure ES module imports are wired correctly.
2. **Implement/Port the Stewart Class:** Supply `initCustom`, inverse kinematics, and forward kinematics functions compatible with the geometry definitions herein.
3. **Restore Workspace Sweep:** Implement the nested loops in `workspace.js` to enumerate all poses, evaluate constraints, and compute coverage per Equation (1).
4. **Derive Jacobian Metrics:** Implement `computeJacobian` and `jacobianMetrics` using Section 7 to populate dexterity and stiffness metrics.
5. **Finalize Optimization Loop:** Reinstate the full NSGA-II workflow in `optimizer.js`, including population evaluation, selection, crossover, mutation, and Pareto extraction.
6. **Integrate Load/Torque Models:** Use Section 8 formulas to ensure torque and servo speed limits are enforced during evaluation.
7. **Validate Against Examples:** Run the optimizer on the provided AYVA, Circular, and Hexagonal layouts. Confirm that coverage, torque, and dexterity outputs are consistent with engineering expectations. Adjust mutation rates or constraints as needed.
8. **Document and Version:** Export optimized layouts, compare them with baseline designs, and capture Pareto fronts for regression tracking.

---

## Appendix A — Quaternion Algebra

Given a unit quaternion \(q = (w, x, y, z)\), the rotation matrix \(R(q)\) is:
\[
R(q) = \begin{bmatrix}
1 - 2(y^2 + z^2) & 2(xy - wz)        & 2(xz + wy) \\
2(xy + wz)       & 1 - 2(x^2 + z^2) & 2(yz - wx) \\
2(xz - wy)       & 2(yz + wx)        & 1 - 2(x^2 + y^2)
\end{bmatrix}.
\]

To convert ZYX Euler angles \((\psi, \theta, \phi)\) to a quaternion, use:
\[
\begin{aligned}
q_w &= \cos\frac{\psi}{2}\cos\frac{\theta}{2}\cos\frac{\phi}{2} + \sin\frac{\psi}{2}\sin\frac{\theta}{2}\sin\frac{\phi}{2}, \\
q_x &= \cos\frac{\psi}{2}\cos\frac{\theta}{2}\sin\frac{\phi}{2} - \sin\frac{\psi}{2}\sin\frac{\theta}{2}\cos\frac{\phi}{2}, \\
q_y &= \cos\frac{\psi}{2}\sin\frac{\theta}{2}\cos\frac{\phi}{2} + \sin\frac{\psi}{2}\cos\frac{\theta}{2}\sin\frac{\phi}{2}, \\
q_z &= \sin\frac{\psi}{2}\cos\frac{\theta}{2}\cos\frac{\phi}{2} - \cos\frac{\psi}{2}\sin\frac{\theta}{2}\sin\frac{\phi}{2}.
\end{aligned}
\]

Normalize \(q\) to unit length before applying it to anchors.

---

## Appendix B — Load Sharing Metrics

For a refined load-sharing analysis, compute individual leg forces \(F_k\) by solving the static equilibrium:
\[
J^\top \mathbf{F} = \mathbf{w},
\]
where \(\mathbf{F} = (F_1, \ldots, F_6)\) and \(\mathbf{w}\) is the wrench (forces and moments) exerted by the payload. Evaluate:
\[
\text{load sharing} = 1 - \frac{\max_k F_k - \min_k F_k}{\sum_k F_k},
\]
so layouts with uniform forces approach 1, while imbalanced designs drop toward 0.

---

## Appendix C — Stiffness Estimation via Virtual Work

To approximate Cartesian stiffness without a full structural model:

1. Estimate leg stiffness constants \(k_k\) based on rod material and geometry.
2. Form a diagonal matrix \(K_l = \operatorname{diag}(k_1, \ldots, k_6)\).
3. The Cartesian stiffness matrix is approximated by:
\[
K_c = J^\top K_l J.
\]
4. The smallest eigenvalue of \(K_c\) serves as a stiffness index; maximize this value during optimization.

---

## 14. References

- *Inverse Kinematics of a Stewart Platform* — Raw research notes on servo geometry and kinematic derivations.
- Chablat & Wenger (2007), "Workspace Analysis using Interval Methods" — global workspace evaluation techniques.
- Pashkevich et al. (2007), "Geometric Synthesis of Parallel Mechanisms" — geometric parameter optimization strategies.
- Taherifar et al. (2013), "Kinematic Optimization with GA" — genetic algorithm application to Stewart platforms.
- Petrašinović et al. (2022), "Real-coded GA for Rotary Stewart Platforms" — advanced evolutionary encoding schemes.
- Albitar et al. (2018), "Hybrid GA + PSO for Parallel Manipulators" — hybrid optimization methodology.
- Kelaiaia & Zaatri (2012), "Multiobjective GA for PKM" — NSGA-II usage for parallel kinematic machines.
- Jamwal et al. (2009), "Modified GA for Rehab Robots" — constraints handling and Pareto analysis for Stewart-type robots.

