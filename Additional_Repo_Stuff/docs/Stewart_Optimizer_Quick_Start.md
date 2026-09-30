# Stewart Optimizer quick start

1. From the repository root run `python -m http.server 8000 --bind 127.0.0.1` and open [localhost:8000](http://localhost:8000). Choose another port if needed. No npm installation is needed to use the application.
2. Keep the automatically loaded [sample requirements](../examples/Sample_Requirements.json), or edit it using the [input reference](../../docs/REQUIREMENTS.md). Flat JSON is also supported. Ranges are offsets from home in mm; input rotations are degrees.
3. Begin with population 12, generations 5 and the default min/midpoint/max samples. The bundled sample budgets 57,168 checks including its cycle and home poses. Finer sweeps grow as the product of all six axis counts. Requests over 100,000 workspace poses/layout or 1,000,000 budgeted checks/run are rejected before evaluation.
4. Adjust controls as needed. Explicit edits survive Run; untouched controls follow edits to the JSON. Loading the sample resets them. Strict joint evaluation is the default; optional soft exploration reports a separate relaxed score.
5. Select Run Optimization and observe progress. Cancel stops an active sweep. If a complete population exists, its results remain available and are marked partial.
6. Inspect feasible coverage, cycle validity, joint violations and modeling limitations. Download Layout JSON saves one candidate with geometry, metrics, feasibility, policy and run status. JSON is the only export format; there is no Pareto plot or integrated 3D view.

The example does not guarantee a feasible design. Coarse sampling can miss failures between samples, and motor capacity/collisions are not checked. [Results and export details](../../docs/RESULTS.md) and the [cycle model](../../docs/CYCLE_MODEL.md) explain what the reported values establish.

Run `npm test` with Node.js >=22 for automated checks. The [archived simulator](../../Raw%20Information/Stuff%20from%20Old%20Project/README.md) launches separately and uses different legacy math.
