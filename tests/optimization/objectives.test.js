import test from 'node:test';
import assert from 'node:assert/strict';
import { objectiveDefinitions, objectiveValues, normalizeObjectiveSet } from '../../src/optimization/objectives.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { layoutToJSON, isPassing, compareCandidates } from '../../src/io/results.js';
import { loadUI } from '../ui/helpers.js';

test('Compact and Full have named directions and retain proxy labels', () => {
  const compact = objectiveDefinitions('compact');
  const full = objectiveDefinitions('full');
  assert.deepEqual(compact.map(({ key, direction }) => [key, direction]), [
    ['coverage', 'max'], ['conditioningQuality', 'max'], ['torque', 'min'], ['speedDemand', 'min'],
  ]);
  assert.deepEqual(full.map(item => item.key), [
    'coverage', 'conditioningQuality', 'torque', 'speedDemand', 'dexterity',
    'stiffness', 'loadSharing', 'limitMargin', 'fatigue',
  ]);
  assert.equal(full.length, 9);
  for (const key of ['stiffness', 'loadSharing', 'limitMargin', 'fatigue']) {
    assert.ok(full.find(item => item.key === key).approximation);
  }
  const legacyFull = objectiveDefinitions('full-v1');
  assert.equal(legacyFull[6].key, 'loadBalance');
  assert.match(legacyFull[6].approximation, /legacy directional proxy/);
  assert.equal(compact.find(item => item.key === 'torque').unit, 'N m');
  assert.equal(compact.find(item => item.key === 'speedDemand').unit, 'rad/s');
  assert.throws(() => normalizeObjectiveSet('unknown'), /objectiveSet/);
});

test('unavailable demand is worst in either objective set and passing still wins', () => {
  const metrics = { coverage: 100, conditioningQuality: 0.2, torque: 3, speedDemand: 4,
    dexterity: 0.1, stiffness: 2, loadBalance: 0.9, limitMargin: 0.8, fatigue: 0.3 };
  assert.deepEqual(objectiveValues(metrics, 'compact'), [100, 0.2, -3, -4]);
  assert.deepEqual(objectiveValues(metrics, 'full-v1'), [100, 0.2, -3, -4, 0.1, 2, 0.9, 0.8, -0.3]);
  assert.deepEqual(objectiveValues({ ...metrics, loadSharing: 0.7 }, 'full'), [100, 0.2, -3, -4, 0.1, 2, 0.7, 0.8, -0.3]);
  const missing = { ...metrics, torque: null, speedDemand: NaN };
  assert.deepEqual(objectiveValues(missing, 'compact').slice(-2), [-Infinity, -Infinity]);
  const passing = { ...metrics, layout: { id: 1 }, feasibility: { passing: true } };
  const failing = { ...metrics, layout: { id: 2 }, torque: 0,
    feasibility: { passing: false, failedCategories: ['cycle'] } };
  assert.equal(isPassing(passing), true);
  assert.equal(compareCandidates(passing, failing), -1);
});

test('search defaults, mutation boundaries, and old objective snapshots are explicit', () => {
  const defaults = new Optimizer();
  assert.equal(defaults.populationSize, 12);
  assert.equal(defaults.generations, 5);
  assert.equal(defaults.mutationRate, 0.35);
  assert.equal(defaults.objectiveSet, 'compact');
  assert.equal(new Optimizer({}, { mutationRate: 0 }).mutationRate, 0);
  assert.equal(new Optimizer({}, { mutationRate: 1 }).mutationRate, 1);
  for (const mutationRate of [-0.01, 1.01, NaN, Infinity, '0.5']) {
    assert.throws(() => new Optimizer({}, { mutationRate }), /mutationRate/);
  }
  const legacy = ['coverage', 'relaxedCoverage', 'dexterity', 'stiffness', 'loadBalance',
    'isotropy', 'limitMargin', 'torque', 'speedDemand', 'fatigue'];
  assert.equal(normalizeObjectiveSet(legacy), 'legacy-v2');
  assert.equal(objectiveDefinitions(legacy).length, 10);
});

test('effective objective settings and zero mutation replay identically from saved JSON', () => {
  const optimizer = new Optimizer({}, { populationSize: 4, generations: 1,
    objectiveSet: 'full', mutationRate: 0, seed: 57 });
  const settings = optimizer.effectiveSettings();
  assert.equal(settings.objectiveSet, 'full');
  assert.equal(settings.objectiveDefinitions.length, 9);
  assert.equal(settings.mutationRate, 0);
  const saved = { ...layoutToJSON(optimizer.createRandomLayout()), run: { effective_settings: settings } };
  const replay = Optimizer.fromReplay(saved);
  assert.deepEqual(replay.effectiveSettings(), settings);
  assert.deepEqual(layoutToJSON(replay.createRandomLayout()).base_anchors,
    layoutToJSON(new Optimizer({}, { populationSize: 4, generations: 1,
      objectiveSet: 'full', mutationRate: 0, seed: 57 }).createRandomLayout()).base_anchors);
});

test('UI passes objective set and mutation probability, including zero', async () => {
  let captured;
  class StubOptimizer {
    constructor(_requirements, options) { captured = options; this.pareto = []; this.fitness = []; }
    estimateWork() { return { totalPoses: 8 }; }
    start() { return Promise.resolve({ status: 'completed' }); }
  }
  const element = await loadUI(StubOptimizer);
  element('optObjectiveSet').value = 'full';
  element('optMutationRate').value = '0';
  await element('runOptimization').handlers.click();
  assert.equal(captured.objectiveSet, 'full');
  assert.equal(captured.mutationRate, 0);
  element('optMutationRate').value = '';
  await element('runOptimization').handlers.click();
  assert.match(element('optStatus').textContent, /mutationRate/);
});
