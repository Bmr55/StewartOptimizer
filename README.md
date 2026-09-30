# Stewart Platform Optimizer

A browser-based prototype for exploring six-servo Stewart platform geometries. Supply payload and movement requirements, evolve candidate layouts with NSGA-II, inspect sampled workspace/cycle results, and download one layout as JSON. Computation stays in the browser; the active optimizer has no external runtime dependencies or backend.

## Run the application

From the repository root:

```sh
python -m http.server 8000 --bind 127.0.0.1
```

Open [localhost:8000](http://localhost:8000). If that port is occupied, choose another port. Serve the directory over HTTP; opening index.html as a local file is not the supported workflow because modules and the sample JSON use browser loading rules.

1. The supplied [sample requirements](Additional_Repo_Stuff/examples/Sample_Requirements.json) load automatically.
2. Edit the JSON and/or controls. Explicit control overrides win; untouched controls follow edited JSON defaults. Load Sample Requirements resets the controls.
3. Start with population **12**, generations **5**, and the automatically populated 3-point ranges. The sample performs 72 candidate evaluations: 52,488 workspace poses and at most 4,680 home/cycle poses, for a total budget of **57,168**.
4. Run Optimization. Progress updates during evaluation. Cancel stops an active sweep and retains only the last completed population, explicitly labelled partial.
5. Inspect feasible coverage, cycle validity and constraint violations. Download Layout JSON saves the highest-feasible-coverage candidate in the retained Pareto front. A downloaded candidate may satisfy only part of the requested workspace or fail the cycle; check its feasibility fields.

The grid is deliberately coarse. A 100% sampled coverage result does not establish continuous coverage, collision freedom, load capacity, or hardware safety. Search is stochastic, so repeated runs can return different designs. Zero/low coverage is a valid result, not a promise that the optimizer found a usable design.

## Implemented scope

- Nested and flat requirements JSON with shared validation.
- Random candidate generation, crossover, mutation, non-dominated sorting, crowding distance and tournament selection.
- Six-dimensional workspace sweeps and inverse kinematics with geometry, servo, rod-length and modeled ball-joint checks.
- Separate strict feasible coverage and optional soft ball-joint exploration; soft exploration never changes physical geometry or feasible coverage.
- Axis-specific sampled cycle force-balance/servo-speed estimates, plus geometric stiffness/dexterity/load-balance/fatigue proxies.
- Bounded work, progress, cancellation and JSON export.

The root app has no layout importer, integrated 3D viewer, workspace/Pareto plots, CSV export, PSO/hybrid search or ML surrogate. Those features appear in historical specifications, not in the current UI. Collision detection, actuator capacity, component inertia/friction, material fatigue and calibrated compliance models are absent.

## Reference

- [Requirements, defaults, units and validation](docs/REQUIREMENTS.md)
- [Cycle demand model and assumptions](docs/CYCLE_MODEL.md)
- [Metrics and exported fields](docs/RESULTS.md)
- [Current architecture and reproduction guide](OPTIMIZER_REPRODUCTION.md)
- [Quick start](Additional_Repo_Stuff/docs/Stewart_Optimizer_Quick_Start.md)
- [Archived interactive simulator](Raw%20Information/Stuff%20from%20Old%20Project/README.md)

## Source layout

| File | Responsibility |
| --- | --- |
| index.html | Controls, JSON input/output, run/cancel lifecycle |
| requirements.js | Input normalization, validation and grid defaults |
| optimizer.js | Candidate generation, NSGA-II, work budget and export |
| workspace.js | Pose constraints, workspace aggregation, progress and cancellation |
| cycle.js | Sampled trajectory, force/moment equilibrium, torque and speed |
| math.js | Vectors, rotations, range construction and singular-value estimates |
| tests/ | Node regression checks, including execution of the UI script in a DOM harness |
| Raw Information/Stuff from Old Project/ | Independent archived simulator and its optional build tooling |

## Verification

Install Node.js 22 or newer, then run `npm test` (or `node --test`). The root tests need no package installation. They exercise requirements, UI overrides, feasibility/export metadata, bounded execution, cancellation, cycle force balance and speed derivatives, and the shipped legacy bundle API. There is no hosted CI workflow in this repository.

For browser verification, run the sample, confirm changing progress and responsive controls, cancel a longer run, and inspect/download JSON. The archived simulator README provides its separate canvas/animation smoke check and bundle rebuild instructions. These checks do not replace validation against known mechanisms or hardware.
