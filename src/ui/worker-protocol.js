import { failureCategories, isPassing, selectBest } from '../io/results.js';

// All run-scoped messages carry runId. Progress contains summaries only;
// checkpoints and results may carry a completed retained population.
export const WORKER_MESSAGE_TYPES = Object.freeze([
  'start', 'cancel', 'started', 'progress', 'checkpoint', 'result', 'error',
]);

export function optionsFromEffectiveSettings(settings) {
  return { ...settings, ranges: settings.bounds,
    referenceLayout: settings.reference_layout ?? settings.referenceLayout ?? null };
}

export function candidateSummary(evaluation) {
  if (!evaluation) return null;
  return {
    id: evaluation.layout?.id ?? null,
    passing: isPassing(evaluation),
    failedCategories: failureCategories(evaluation),
    coverage: evaluation.coverage ?? null,
    torque: Number.isFinite(evaluation.torque) ? evaluation.torque : null,
    speedDemand: Number.isFinite(evaluation.speedDemand) ? evaluation.speedDemand : null,
    conditioningQuality: Number.isFinite(evaluation.conditioningQuality) ? evaluation.conditioningQuality : null,
  };
}

export function estimateRemainingMs(elapsedMs, completedCandidates, totalCandidates) {
  if (!Number.isFinite(elapsedMs) || elapsedMs <= 0
    || !Number.isSafeInteger(completedCandidates) || completedCandidates < 3
    || !Number.isSafeInteger(totalCandidates) || totalCandidates < completedCandidates) return null;
  return elapsedMs / completedCandidates * (totalCandidates - completedCandidates);
}

export function progressSnapshot(optimizer, progress, elapsedMs) {
  const completedCandidates = optimizer.completedEvaluations || 0;
  const totalCandidates = optimizer.populationSize * (optimizer.generations + 1);
  const pareto = optimizer.pareto ?? [];
  const fitness = optimizer.fitness ?? [];
  const best = selectBest(pareto, fitness);
  const etaMs = estimateRemainingMs(elapsedMs, completedCandidates, totalCandidates);
  return {
    elapsedMs: Number.isFinite(elapsedMs) ? elapsedMs : null,
    completedCandidates, totalCandidates,
    generation: progress.generation ?? optimizer.generation,
    frontSize: pareto.length,
    bestCandidate: candidateSummary(best),
    actualCompletedPoseWork: progress.completed ?? optimizer.completedPoseWork ?? 0,
    budgetedPoseWork: optimizer.workEstimate?.totalPoses ?? progress.total ?? null,
    etaMs, etaApproximate: etaMs !== null,
  };
}

export function checkpointSnapshot(optimizer) {
  return {
    generation: optimizer.generation,
    completedCandidates: optimizer.completedEvaluations,
    fitness: optimizer.fitness,
    paretoIds: optimizer.pareto.map(candidate => candidate.layout.id),
  };
}
