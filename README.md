# Stewart Platform Optimizer

A browser-based prototype for exploring six-servo Stewart platform geometries. Supply payload and movement requirements, evolve candidate layouts with NSGA-II, inspect sampled workspace/cycle results, and download one layout as JSON. Computation stays in the browser; the active optimizer has no external runtime dependencies or backend.

## Run the application

With Node.js 22 or newer, from the repository root (no package installation needed):

```sh
npm run dev
```

Open the printed local URL. If port 8000 is occupied, use `npm run dev -- --port 8001`; port 0 selects a free port. The server binds only to 127.0.0.1. Any static HTTP server also works, including `python -m http.server 8000 --bind 127.0.0.1`. Opening index.html as a local file is not supported because modules and sample JSON use browser loading rules.

1. The supplied [sample requirements](./examples/sample-requirements.json) load automatically.
2. Edit the requirements JSON and/or controls. Optionally paste or load a [reference layout](./docs/IMPORT.md) in its separate input to refine a previous design. Choose a [layout topology](./docs/TOPOLOGIES.md) and home-height bounds for fresh searches. Explicit control overrides win; untouched controls follow edited JSON defaults. Load Sample Requirements resets the controls.
3. Start with population **12**, generations **5**, and the default 1,024 Halton samples. The sample performs 72 candidate evaluations with a total budget of **78,408** pose checks. Choose 256 samples for a quicker run or Cartesian grid to use the axis steps.
4. Run Optimization. A module worker keeps the page responsive while progress updates. Cancel stops an active sweep and retains only the last completed population, explicitly labelled partial. If worker startup fails, choose the offered main-thread fallback to continue.
5. Inspect feasible coverage, cycle validity and constraint violations. Choose a retained candidate in the results browser; Download Layout JSON saves that candidate, including the exact reference when selected. A downloaded candidate may satisfy only part of the requested workspace or fail the cycle; check its feasibility fields.

The grid preset is deliberately coarse. A 100% sampled coverage result does not establish continuous coverage, collision freedom, load capacity, or hardware safety. A fixed seed and unchanged effective settings reproduce a run; changing either can return different designs. Zero/low coverage is a valid result, not a promise that the optimizer found a usable design.

## Implemented scope

- Nested and flat requirements JSON with shared validation.
- Circular, C3 paired, rectangular paired, and Free candidate generation with invariant-preserving crossover and mutation, independently bounded home height, non-dominated sorting, crowding distance and tournament selection.
- Exact reference-layout import with validation, diagnostic retention, bounded variations and fresh candidates, and recalculated metrics.
- Six-dimensional workspace sweeps and inverse kinematics with geometry, servo, rod-length and modeled ball-joint checks.
- Separate strict feasible coverage and optional soft ball-joint exploration; soft exploration never changes physical geometry or feasible coverage.
- Axis-specific sampled cycle force-balance/servo-speed estimates, plus geometric stiffness/dexterity/load-balance/fatigue proxies.
- Bounded work, progress, cancellation and JSON export.

The root app has no integrated 3D viewer, PSO/hybrid search or ML surrogate. Collision detection, calibrated actuator dynamics, component inertia/friction, material fatigue and calibrated compliance models are absent. Supplied servo ratings are compared with modeled cycle demand.

## Reference

- [Requirements, defaults, units and validation](./docs/REQUIREMENTS.md)
- [Layout topologies, parameters and bounds](./docs/TOPOLOGIES.md)
- [Reference-layout import and seeding](./docs/IMPORT.md)
- [Cycle demand model and assumptions](./docs/CYCLE_MODEL.md)
- [Metrics and exported fields](./docs/RESULTS.md)
- [Current architecture and reproduction guide](./docs/architecture.md)
- [Browser worker protocol and fallback](./docs/WORKER_PROTOCOL.md)
- [Quick start](./docs/quick-start.md)
- [Archived interactive simulator](./archive/simulator/README.md)

## Source layout

| File | Responsibility |
| --- | --- |
| index.html, styles/ | Page markup and appearance |
| src/main.js, src/ui/ | Initialization, controls, tooltips, run/cancel and downloads |
| src/model/ | Requirements, single-pose constraints and cycle demand |
| src/workspace/ | Workspace sweep, scheduling and statistics |
| src/optimization/ | Run lifecycle, layout operators, NSGA-II, scoring and work budget |
| src/io/ | Sample loading, result selection and serialization |
| src/math.js | Vectors, rotations, ranges and singular-value estimates |
| examples/ | Runnable sample requirements |
| tests/ | Checks grouped by responsibility; UI modules imported into a DOM harness |
| scripts/ | Local server and complete-sample smoke check |
| docs/ | Current architecture, usage and model references |
| archive/ | Independent simulator and historical research materials |

The small root JavaScript files preserve original import paths. New code should import from `src/`; see [API compatibility](docs/architecture.md#api-compatibility).

## Verification

Run `npm test` (or `node --test`). The root tests need no package installation. They exercise requirements, UI overrides, feasibility/export metadata, bounded execution, cancellation, selection, cycle force balance and speed derivatives, reference numerical outputs, the development server and the shipped legacy bundle API. There is no hosted CI workflow in this repository.

Run `npm run smoke` for a complete default sample: it verifies 72 candidate evaluations, a 78,408-pose budget, completed progress and a valid JSON export. It checks execution rather than requiring a particular coverage score or machine-dependent timing. Run `npm ci && npm run test:browser` for the real browser worker lifecycle check.

The optional `npm run legacy:build` delegates to the archived simulator's build. First install its locked development dependencies with `npm --prefix archive/simulator ci`; the active app has no build step or runtime dependencies.

For browser verification, run the sample, confirm changing progress and responsive controls, cancel a longer run, and inspect/download JSON. The archived simulator README provides its separate canvas/animation smoke check and bundle rebuild instructions. These checks do not replace validation against known mechanisms or hardware.
