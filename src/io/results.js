import { radToDeg } from '../math.js';

export function selectBest(pareto, fitness) {
  const front = pareto?.length ? pareto : fitness;
  return front.slice().sort((a, b) => b.coverage - a.coverage)[0];
}

export function layoutToJSON(layout, toDegrees = radToDeg) {
  return {
    base_anchors: layout.baseAnchors.map(a => a.slice()),
    platform_anchors: layout.platformAnchors.map(a => a.slice()),
    beta_angles: layout.betaAngles.slice(),
    horn_length: layout.hornLength,
    rod_length: layout.rodLength,
    servo_range: layout.servoRangeRad.map(toDegrees),
    home_height: layout.homeHeight,
  };
}

const METRICS = [
  ['coverage', 'coverage'], ['relaxedCoverage', 'relaxed_coverage'],
  ['dexterity', 'dexterity'], ['stiffness', 'stiffness'], ['torque', 'torque'],
  ['speedDemand', 'speed_demand'], ['loadBalance', 'load_balance'],
  ['isotropy', 'isotropy'], ['limitMargin', 'limit_margin'], ['fatigue', 'fatigue'],
];

export function displayResult(evaluation) {
  return {
    metrics: Object.fromEntries(METRICS.map(([key]) => [key, evaluation[key]])),
    // Preserve the display's original floating-point conversion order.
    layout: layoutToJSON(evaluation.layout, rad => rad * 180 / Math.PI),
    cycle: evaluation.cycle,
    feasibility: evaluation.feasibility,
    constraint_policy: evaluation.workspace?.constraintPolicy,
    workspace_stats: evaluation.workspace?.stats,
    workspace_counts: evaluation.workspace?.counts,
    workspace_samples: evaluation.workspace?.samples,
  };
}

export function exportResult(evaluation, run) {
  return {
    ...layoutToJSON(evaluation.layout),
    metadata: Object.fromEntries(METRICS.map(([key, name]) => [name, evaluation[key] ?? null])),
    cycle: evaluation.cycle ?? null,
    feasibility: evaluation.feasibility ?? null,
    constraint_policy: evaluation.workspace?.constraintPolicy ?? null,
    workspace_counts: evaluation.workspace?.counts ?? null,
    workspace_stats: evaluation.workspace?.stats ?? null,
    run,
  };
}
