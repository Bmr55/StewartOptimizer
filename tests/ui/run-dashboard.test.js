import test from 'node:test';
import assert from 'node:assert/strict';
import { createRunDashboard, formatDuration, bestCandidateText } from '../../src/ui/run-dashboard.js';
import { estimateRemainingMs, progressSnapshot } from '../../src/ui/worker-protocol.js';
import { loadUI } from './helpers.js';

function elements() {
  const nodes = new Map();
  const getElementById = id => {
    if (!nodes.has(id)) nodes.set(id, { textContent: '' });
    return nodes.get(id);
  };
  return { document: { getElementById }, value: id => getElementById(id).textContent };
}

const summary = overrides => ({ elapsedMs: 900, completedCandidates: 3, totalCandidates: 8,
  generation: 0, frontSize: 2,
  bestCandidate: { id: 7, passing: true, failedCategories: [], coverage: 80,
    torque: 1.23, speedDemand: 0.5, conditioningQuality: 0.25 },
  actualCompletedPoseWork: 17, budgetedPoseWork: 100,
  etaMs: 1500, etaApproximate: true, ...overrides });

test('controlled elapsed/candidate observations warm ETA only after three completions', () => {
  assert.equal(estimateRemainingMs(900, 2, 8), null);
  assert.equal(estimateRemainingMs(900, 3, 8), 1500);
  assert.equal(estimateRemainingMs(900, 8, 8), 0);
  for (const elapsed of [0, -1, NaN, Infinity]) {
    assert.equal(estimateRemainingMs(elapsed, 3, 8), null);
  }
  assert.equal(estimateRemainingMs(900, 3, 2), null);
  assert.equal(formatDuration(NaN), 'Unavailable');
  assert.equal(formatDuration(59940), '59.9 s');
  assert.equal(formatDuration(59960), '1m 0s');
  assert.equal(formatDuration(90000), '1m 30s');
  assert.equal(formatDuration(119600), '2m 0s');
  assert.equal(formatDuration(3599600), '60m 0s');
});

test('snapshot uses actual work, a bounded best summary, and completed population selection', () => {
  const passing = { layout: { id: 7 }, coverage: 80, torque: 1.23, speedDemand: 0.5,
    conditioningQuality: 0.25, feasibility: { passing: true } };
  const diagnostic = { layout: { id: 8 }, coverage: 99, torque: 0.1, speedDemand: 0.1,
    feasibility: { passing: false, failedCategories: ['cycle'] } };
  const optimizer = { completedEvaluations: 3, populationSize: 4, generations: 1,
    pareto: [diagnostic, passing], fitness: [diagnostic, passing],
    workEstimate: { totalPoses: 100 } };
  const snapshot = progressSnapshot(optimizer, { completed: 17, total: 100, generation: 0 }, 900);
  assert.equal(snapshot.bestCandidate.id, 7);
  assert.equal(snapshot.bestCandidate.passing, true);
  assert.equal(snapshot.actualCompletedPoseWork, 17);
  assert.equal(snapshot.budgetedPoseWork, 100);
  assert.equal(snapshot.frontSize, 2);
  assert.equal(snapshot.etaMs, 1500);
  assert.equal('fitness' in snapshot, false);
  assert.equal('layout' in snapshot.bestCandidate, false);
  assert.match(bestCandidateText(snapshot.bestCandidate), /#7 · passing · coverage 80.0%/);
});

test('dashboard publishes at most 10 Hz and rejects stale snapshots through all states', () => {
  let time = 0;
  const view = elements();
  const dashboard = createRunDashboard(view.document, { now: () => time });
  dashboard.start(10, { candidates: 8, generations: 1, poseWork: 100 });
  assert.equal(view.value('runPhase'), 'Running');
  assert.equal(view.value('runCandidates'), '0 / 8');
  assert.equal(dashboard.publish(9, summary()), false);
  assert.equal(dashboard.publish(10, summary({ completedCandidates: 2, etaMs: null,
    etaApproximate: false })), true);
  assert.equal(view.value('runEta'), 'Unavailable until 3 completed candidates');
  time = 50;
  assert.equal(dashboard.publish(10, summary()), false);
  assert.equal(view.value('runCandidates'), '2 / 8');
  time = 100;
  assert.equal(dashboard.publish(10, summary()), true);
  assert.equal(view.value('runCandidates'), '3 / 8');
  assert.equal(view.value('runGeneration'), '0 / 1');
  assert.equal(view.value('runFrontSize'), '2');
  assert.equal(view.value('runElapsed'), '0.9 s');
  assert.equal(view.value('runEta'), '≈ 1.5 s (approximate)');
  assert.equal(view.value('runPoseWork'), '17 actual / 100 budgeted');
  assert.match(view.value('runBestCandidate'), /#7 · passing/);
  time = 200;
  dashboard.publish(10, summary({ etaMs: null, etaApproximate: false }));
  assert.equal(view.value('runEta'), 'Unavailable (timing data)');
  dashboard.cancelRequested(10);
  assert.equal(view.value('runPhase'), 'Cancelling');
  dashboard.finish(10, { status: 'cancelled', partialResults: true,
    snapshot: summary({ completedCandidates: 4, actualCompletedPoseWork: 21 }) });
  assert.equal(view.value('runPhase'), 'cancelled · partial results');
  assert.equal(view.value('runCandidates'), '4 / 8');
  assert.equal(view.value('runPoseWork'), '21 actual / 100 budgeted');
  assert.equal(view.value('runEta'), 'Unavailable');
  assert.equal(dashboard.publish(10, summary({ completedCandidates: 999 })), false);
  dashboard.start(11, { candidates: 8, generations: 1, poseWork: 100 });
  assert.equal(dashboard.publish(10, summary()), false);
  dashboard.finish(11, { status: 'failed', snapshot: summary({ bestCandidate: null }) });
  assert.equal(view.value('runPhase'), 'failed');
  assert.equal(view.value('runBestCandidate'), 'No completed population yet');
});

test('UI adapts direct headless progress with the same dashboard counts and final state', async () => {
  let time = 0;
  class StubOptimizer {
    constructor(_requirements, options) {
      this.onProgress = options.onProgress;
      this.populationSize = options.populationSize;
      this.generations = options.generations;
      this.fitness = []; this.pareto = []; this.generation = 0;
      this.completedEvaluations = 0;
    }
    estimateWork() { return { totalPoses: 100, evaluations: 8 }; }
    async start() {
      this.workEstimate = this.estimateWork();
      time = 100;
      this.completedPoseWork = 17;
      this.completedEvaluations = 3;
      this.onProgress({ completed: 17, total: 100, generation: 0 });
      return { status: 'completed' };
    }
  }
  const element = await loadUI(StubOptimizer, { now: () => time });
  await element('runOptimization').handlers.click();
  assert.equal(element('runPhase').textContent, 'completed');
  assert.equal(element('runCandidates').textContent, '3 / 8');
  assert.equal(element('runPoseWork').textContent, '17 actual / 100 budgeted');
  assert.equal(element('runEta').textContent, 'Complete');
});
