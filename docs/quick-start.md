# Stewart Optimizer quick start

1. With Node.js >=22, run `npm run dev` from the repository root and open the printed local URL. Use `npm run dev -- --port 8001` if needed. No package installation is needed. Other static HTTP servers also work.
2. Keep the automatically loaded [sample requirements](../examples/sample-requirements.json), or edit it using the [input reference](./REQUIREMENTS.md). Flat JSON is also supported. Ranges are offsets from home in mm; input rotations are degrees.
3. Begin with population 12, generations 5 and the default min/midpoint/max samples. The bundled sample budgets 57,168 checks including its cycle and home poses. Finer sweeps grow as the product of all six axis counts. Requests over 100,000 workspace poses/layout or 1,000,000 budgeted checks/run are rejected before evaluation.
4. Adjust controls as needed. Explicit edits survive Run; untouched controls follow edits to the JSON. Loading the sample resets them. Strict joint evaluation is the default; optional soft exploration reports a separate relaxed score.
5. Select Run Optimization and observe progress. Cancel stops an active sweep. If a complete population exists, its results remain available and are marked partial.
6. Inspect feasible coverage, cycle validity, joint violations and modeling limitations. Download Layout JSON saves one candidate with geometry, metrics, feasibility, policy and run status. JSON is the only export format; there is no Pareto plot or integrated 3D view.

The example does not guarantee a feasible design. Coarse sampling can miss failures between samples, and motor capacity/collisions are not checked. [Results and export details](./RESULTS.md) and the [cycle model](./CYCLE_MODEL.md) explain what the reported values establish.

Run `npm test` for automated regression checks and `npm run smoke` to verify a complete default sample and export. The [archived simulator](../archive/simulator/README.md) launches separately and uses different legacy math.
