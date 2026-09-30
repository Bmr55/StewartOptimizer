import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { DEFAULT_DESIGN_SPACE, mutateLayout, crossoverLayouts, finalizeLayout } from '../../src/optimization/layout-operators.js';
import { initialPopulation } from '../../src/optimization/reference-seeding.js';
import { layoutToJSON } from '../../src/io/results.js';
import { importLayout } from '../../src/io/layout-import.js';

const runOptions = { seed: 3, populationSize: 4, generations: 2,
  sampling: { strategy: 'halton', sampleCount: 4 }, ballJointLimitDeg: 180 };

function reversedKeys(object) {
  return Object.fromEntries(Object.entries(object).reverse());
}

test('seeded runs do not depend on the key order of topology_parameters', async () => {
  for (const topology of ['circular', 'c3_paired', 'rectangular_paired']) {
    const reference = layoutToJSON(new Optimizer({}, { topology, seed: 11 }).createRandomLayout());
    const reordered = { ...reference, topology_parameters: reversedKeys(reference.topology_parameters) };
    assert.notDeepEqual(Object.keys(reordered.topology_parameters), Object.keys(reference.topology_parameters));
    const populations = [];
    for (const referenceLayout of [reference, reordered]) {
      const optimizer = new Optimizer({}, { ...runOptions, referenceLayout });
      const outcome = await optimizer.run();
      assert.equal(outcome.status, 'completed');
      populations.push(optimizer.fitness.map(item => layoutToJSON(item.layout)));
    }
    assert.deepEqual(populations[0], populations[1], `${topology} diverged`);
  }
});

function freeReferenceWithRaisedPlatform() {
  const reference = layoutToJSON(new Optimizer({}, { topology: 'free', seed: 2 }).createRandomLayout());
  for (const point of reference.platform_anchors) point[2] = 40;
  for (const point of reference.base_anchors) point[2] = 30;
  return reference;
}

test('free-topology platform Z is flagged for the reference and kept in-plane for generated candidates', async () => {
  const reference = freeReferenceWithRaisedPlatform();
  const optimizer = new Optimizer({}, { ...runOptions, referenceLayout: reference });
  const conflicts = optimizer.referenceDiagnostics.boundsConflicts;
  for (let i = 0; i < 6; i++) {
    assert.ok(conflicts.some(item => item.field === `platform_anchors[${i}][2]` && item.value === 40),
      `platform_anchors[${i}][2] not reported`);
    assert.ok(conflicts.some(item => item.field === `base_anchors[${i}][2]`));
  }
  const population = initialPopulation(optimizer);
  for (const layout of population.slice(1)) {
    for (const point of layout.platformAnchors) assert.equal(point[2], 0);
  }
  const outcome = await optimizer.run();
  assert.equal(outcome.status, 'completed');
  for (const item of optimizer.fitness) {
    const expected = item.layout.seedOrigin === 'reference' ? 40 : 0;
    for (const point of item.layout.platformAnchors) assert.equal(point[2], expected);
    if (item.layout.seedOrigin !== 'reference') {
      for (const point of item.layout.baseAnchors) {
        assert.ok(Math.abs(point[2]) <= optimizer.designSpace.baseZJitter + 1e-9);
      }
    }
  }
});

test('finalizeLayout drops the imported servoRangeDeg so direct operator use exports the finalized range', () => {
  const source = layoutToJSON(new Optimizer({}, { topology: 'free', seed: 5 }).createRandomLayout());
  source.servo_range = [-137, 113];
  const imported = importLayout(source).layout;
  assert.deepEqual(imported.servoRangeDeg, [-137, 113]);
  const context = { designSpace: DEFAULT_DESIGN_SPACE, servoRangeRad: [-2, 2] };
  const expected = [-2, 2].map(value => value * 180 / Math.PI);
  for (const child of [mutateLayout(imported, context), crossoverLayouts(imported, imported, context),
    finalizeLayout(JSON.parse(JSON.stringify(imported)), context)]) {
    assert.equal(child.servoRangeDeg, undefined);
    assert.deepEqual(layoutToJSON(child).servo_range, expected);
  }
  assert.deepEqual(imported.servoRangeDeg, [-137, 113]);
});
