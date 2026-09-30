# Agent notes for Stewart Optimizer

Browser-only NSGA-II optimizer and WebGL2 simulator for six-servo Stewart platforms. Plain ES modules, no bundler, no backend, no runtime dependencies. Read [README.md](README.md) for usage and [docs/architecture.md](docs/architecture.md) for module boundaries before changing code.

## Commands

Node.js 22 or newer. `npm run dev`, `npm test` and `npm run smoke` need no install.

| Command | Purpose |
| --- | --- |
| `npm run dev` | Static server on 127.0.0.1:8000 (`-- --port 0` picks a free port) |
| `npm test` | Regression suite, `node --test` over `tests/**/*.test.js` |
| `npm run smoke` | Full default sample run plus export checks |
| `npm run test:browser`, `test:browser:geometry`, `test:browser:diagnostics` | Playwright checks; run `npm ci` first and have Google Chrome installed |
| `npm run build:charts`, `npm run build:icons` | Refresh the vendored Chart.js and Lucide icons in `assets/` |

Run `npm test` after any change. Run `npm run smoke` when touching the optimizer, evaluator, cycle model or export. Run the browser checks when touching `index.html`, `src/ui/` or `src/simulator/`.

## Layout

- `src/model/`, `src/workspace/`, `src/optimization/`: numerical core. Never imports the UI or touches the DOM.
- `src/ui/`: page controls, worker adapter and protocol, dashboard, results view, browser save.
- `src/simulator/`: WebGL2 renderer, pose controller, geometry editor, diagnostics. The renderer only draws accepted poses; it never solves constraints.
- `src/io/`: sample loading, layout import, result serialization, Fusion/CSV export.
- `src/contracts.js`: schema and model versions, topology names, metric keys, units, failure categories. Add new metrics and versions here.
- `tests/<area>/*.test.js` mirrors `src/`. UI tests import `createApp` into a DOM harness via `tests/ui/helpers.js`.
- `examples/`, `docs/`, `scripts/`, `assets/`, `archive/` as described in the README. `archive/` is a separate legacy simulator with its own build; leave it alone unless asked.

## Rules

- Internal geometry is millimetres and radians. User-facing JSON uses mm and degrees. Convert at the parser and exporter, not in the middle.
- All pose checks go through `src/model/pose.js`. The sweep, cycle and simulator must share that evaluator.
- Prefer `parseRequirements` over hand-built requirement objects. Shared defaults such as `DEFAULT_BALL_JOINT_LIMIT_DEG` live in one place; do not duplicate literals.
- Runs are seeded and replayable. A change that alters results for an unchanged seed is a behaviour change and needs a test and a doc update.
- Long loops must honour the AbortSignal and yield to the event loop. Worker messages must stay structured-clone safe; see [docs/WORKER_PROTOCOL.md](docs/WORKER_PROTOCOL.md).
- Root `math.js`, `cycle.js`, `requirements.js`, `workspace.js` and `optimizer.js` are compatibility shims. Import from `src/` and do not add new root modules.
- No new dependencies in the active app. Dev dependencies are only for tests, browser checks and vendoring.
- The Dockerfile copies only `index.html`, `src/`, `styles/`, `assets/` and `examples/`. A new runtime directory must be added there too.
- Docs are part of the change. Update [docs/FEATURES.md](docs/FEATURES.md) and the relevant model doc when adding a control, default, limit or exported field. Do not claim capabilities the code lacks: no collision detection, no ML surrogate, no PSO.

## Git

Branch from `main` as `fix/issue-N-topic` or `claude/topic`. Commit subjects are imperative and describe the change, not the file. Do not commit `node_modules/` or vendored rebuilds unless the vendored version changed.
