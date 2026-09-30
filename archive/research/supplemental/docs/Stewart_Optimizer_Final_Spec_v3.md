> Historical design proposal, not current implementation documentation. Some formulas, defaults, dependencies and claimed features below are obsolete or unimplemented. See the [current README](../../../../README.md), [architecture](../../../../docs/architecture.md), and [results/model limitations](../../../../docs/RESULTS.md) for supported behavior. Preserve this document as research/roadmap context only.

# Stewart Platform Optimizer — Final Consolidated Specification (v3)

## 1. Goal
Build a **generalized Stewart Platform simulator + optimizer** that:  
- Accepts structured JSON requirements (no assumption of circular/hex layouts).  
- Generates optimal base/platform geometries automatically.  
- Evaluates designs against **workspace, payload, and dynamic requirements**.  
- Uses **multi-objective optimization** to balance trade-offs.  
- Produces a **Pareto front** of designs for engineering decision-making.  

---

## 2. Input: Requirements Schema (v2)
Defined in **Stewart_Requirements_Schema_v2.json**.  

### Required
- `mass_kg`: payload mass in kg  
- `cycle_mm`: cycle amplitude (peak-to-peak, mm)  
- `frequency_hz`: cycle frequency (Hz)  
- `cycle_axis`: `"x"`, `"y"`, or `"z"`  
- Workspace: `x_range_mm`, `y_range_mm`, `z_range_mm`  
- Rotations: `rx_range_deg`, `ry_range_deg`, `rz_range_deg`  

### Optional
- `ball_joint_max_deg` (default 45°)  
- `servo_max_deg` (default 90°)  
- `rod_length_bounds_mm` (default [100, 400])  
- `horn_length_bounds_mm` (default [20, 120])  
- `servo_travel_bounds_deg` (default [-120, 120])  

If omitted, these become **free optimization variables**.  

---

## 3. Optimization Variables
- **Base (servos)**  
  - Anchor positions (6 × [x,y,z])  
  - Servo axis orientations (β angles)  
  - Horn lengths & directions  
- **Platform**  
  - Anchor positions (6 × [x,y,z])  
  - Anchor orientations  
  - Rod lengths  
- **Servo travel**  
  - Free variable unless constrained  

---

## 4. Fitness Functions (Multi-Objective)
Optimization must evaluate:  

1. **Workspace Coverage** — % of target translational + rotational workspace reachable.  
2. **Torque & Speed Demand** — compute required servo torque and RPM for cycle axis at frequency.  
3. **Dexterity** — minimize condition number of Jacobian, avoid singularities.  
4. **Stiffness / Compliance** — estimate stiffness matrix, penalize compliant designs.  
5. **Load Sharing** — ensure even force distribution among legs.  
6. **Joint & Servo Limits** — penalize designs exceeding articulation/servo limits.  

---

## 5. Optimization Methods to Implement
- **Genetic Algorithms (GA)**: NSGA-II for multi-objective optimization.  
- **Pareto Front Analysis**: keep multiple “best” designs.  
- **Monte Carlo Sampling**: broad geometry exploration.  
- **Hybrid GA + PSO**: combine GA search with swarm optimization.  
- **Jacobian Metrics**: dexterity index, isotropy, singularity avoidance.  
- **Stiffness Modeling**: compliance via Jacobian + force mapping.  

---

## 6. Workflow
1. Parse requirements JSON.  
2. Generate random initial geometries.  
3. Apply optimization algorithm (GA/NSGA-II).  
4. Evaluate fitness across all objectives.  
5. Store Pareto front designs.  
6. Visualize results in 3D (workspace + geometry).  
7. Export final layouts in JSON/CSV.  

---

## 7. Outputs
- Optimized base & platform anchor layouts.  
- Rod and horn lengths.  
- Servo travel range.  
- Performance metrics (coverage, torque, stiffness, dexterity).  
- Export formats: JSON, CSV.  
- Pareto plots for trade-off exploration.  

---

## 8. Pseudocode Example

```python
requirements = load_json("requirements.json")

population = initialize_random_layouts(N=200)

for gen in range(max_generations):
    fitness_scores = []
    for candidate in population:
        coverage = simulate_workspace(candidate, requirements)
        torque = compute_torque(candidate, requirements)
        dexterity = compute_dexterity(candidate)
        stiffness = compute_stiffness(candidate)
        penalty = joint_limit_penalty(candidate, requirements)
        fitness_scores.append([coverage, -torque, dexterity, stiffness, -penalty])
    
    population = nsga2_step(population, fitness_scores)

pareto_front = extract_pareto_front(population, fitness_scores)
export_layouts(pareto_front, "optimized_layouts.json")
```

---

## 9. References Used
- RAW Research: *Inverse Kinematics of a Stewart Platform*  
- Chablat & Wenger (2007): *Workspace Analysis using Interval Methods*  
- Pashkevich et al. (2007): *Geometric Synthesis of Parallel Mechanisms*  
- Taherifar et al. (2013): *Kinematic Optimization with GA*  
- Petrašinović et al. (2022): *Real-coded GA for Rotary Stewart Platforms*  
- Albitar et al. (2018): *Hybrid GA + PSO for Parallel Manipulators*  
- Kelaiaia & Zaatri (2012): *Multiobjective GA for PKM*  
- Jamwal et al. (2009): *Modified GA for Rehab Robots*  
