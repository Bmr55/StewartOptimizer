import test from 'node:test';
import assert from 'node:assert/strict';
import { compareCandidates, rankCandidates, selectBest, exportResult } from '../../src/io/results.js';
import { fastNonDominatedSort } from '../../src/optimization/nsga2.js';
import { jointFixture } from '../fixtures/layout.js';

function candidate(id, { passing = true, failedCategories = [], coverage = 100, torque = 2,
  speedDemand = 3, conditioningQuality = 0.2 } = {}) {
  return {
    layout: { ...jointFixture(), id }, coverage, torque, speedDemand, conditioningQuality,
    feasibility: { passing, failedCategories }, objectives: [coverage, -torque],
  };
}

test('passing candidates outrank diagnostics in selection and NSGA fronts', () => {
  const passing = candidate(1, { coverage: 50, torque: 10 });
  const diagnostic = candidate(2, { passing: false, failedCategories: ['cycle'], coverage: 100, torque: 0.01 });
  assert.equal(selectBest([diagnostic], [diagnostic, passing]), passing);
  assert.deepEqual(rankCandidates([diagnostic, passing]), [passing, diagnostic]);
  const evaluations = [diagnostic, passing];
  assert.deepEqual(fastNonDominatedSort(evaluations), [[1], [0]]);
});

test('diagnostic order counts categories, feasible coverage, then available demand', () => {
  const moreFailures = candidate(1, { passing: false, failedCategories: ['home', 'cycle'], coverage: 99, torque: 1 });
  const lessCoverage = candidate(2, { passing: false, failedCategories: ['cycle'], coverage: 50, torque: 1 });
  const unavailable = candidate(3, { passing: false, failedCategories: ['cycle'], coverage: 75, torque: null });
  const lowerDemand = candidate(4, { passing: false, failedCategories: ['cycle'], coverage: 75, torque: 2 });
  assert.deepEqual(rankCandidates([moreFailures, lessCoverage, unavailable, lowerDemand]),
    [lowerDemand, unavailable, lessCoverage, moreFailures]);
  assert.equal(compareCandidates(lowerDemand, unavailable) < 0, true);
});

test('passing initial selection uses torque, speed, then conditioning', () => {
  const slow = candidate(1, { torque: 2, speedDemand: 4, conditioningQuality: 0.9 });
  const worseCondition = candidate(2, { torque: 2, speedDemand: 3, conditioningQuality: 0.1 });
  const best = candidate(3, { torque: 2, speedDemand: 3, conditioningQuality: 0.3 });
  assert.equal(selectBest([], [slow, worseCondition, best]), best);
});

test('otherwise equal candidates order by numeric id in the passing and diagnostic branches', () => {
  const ids = [10, 9, 2];
  const passing = ids.map(id => candidate(id));
  assert.deepEqual(rankCandidates(passing).map(item => item.layout.id), [2, 9, 10]);
  assert.equal(selectBest([], passing).layout.id, 2);
  const diagnostics = ids.map(id => candidate(id, { passing: false, failedCategories: ['cycle'] }));
  assert.deepEqual(rankCandidates(diagnostics).map(item => item.layout.id), [2, 9, 10]);
  assert.equal(selectBest(diagnostics, []).layout.id, 2);
  assert.ok(compareCandidates(candidate(10), candidate(9)) > 0);
});

test('diagnostic JSON preserves unavailable demand and failure details', () => {
  const evaluation = candidate(7, { passing: false, failedCategories: ['cycle'], torque: Infinity });
  const json = JSON.parse(JSON.stringify(exportResult(evaluation, { status: 'cancelled', partial: true })));
  assert.equal(json.diagnostic, true);
  assert.equal(json.metadata.torque, null);
  assert.deepEqual(json.feasibility.failedCategories, ['cycle']);
  assert.equal(json.id, 7);
  assert.equal(json.servo_range.length, 2);
});
