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

test('generated and evolved candidates export the servo range the run settings record', async () => {
  const optimizer = new Optimizer({ servo_travel_bounds_deg: [-120, 120] }, { ...runOptions, seed: 11 });
  const random = optimizer.createRandomLayout();
  assert.deepEqual(layoutToJSON(random).servo_range, [-120, 120]);
  assert.deepEqual(layoutToJSON(optimizer.mutateLayout(random)).servo_range, [-120, 120]);
  assert.deepEqual(layoutToJSON(optimizer.crossoverLayouts(random, optimizer.createRandomLayout())).servo_range, [-120, 120]);
  assert.deepEqual(random.servoRangeRad, [-120, 120].map(degrees => degrees * Math.PI / 180));
  await optimizer.run();
  const exported = JSON.parse(optimizer.exportBest());
  assert.deepEqual(exported.servo_range, exported.run.effective_settings.servoRangeDeg);
  assert.deepEqual(exported.servo_range, [-120, 120]);
  // An imported reference keeps its own degrees; its variations take the run's.
  const source = layoutToJSON(random);
  source.servo_range = [-119, 119];
  const seeded = new Optimizer({ servo_travel_bounds_deg: [-120, 120] }, { ...runOptions, referenceLayout: source });
  const variation = seeded.mutateLayout(seeded.referenceLayout);
  assert.deepEqual(layoutToJSON(seeded.referenceLayout).servo_range, [-119, 119]);
  assert.deepEqual(layoutToJSON(variation).servo_range, [-120, 120]);
});

test('an exported candidate re-imports as the reference without bounds conflicts', () => {
  // ±105° does not round-trip exactly through radians; ±120 (the sample) happens to round inward.
  for (const bound of [105, 114, 96, 57, 52.5, 28.5, 12, 1.5]) {
    const requirements = { servo_travel_bounds_deg: [-bound, bound] };
    const exported = layoutToJSON(new Optimizer(requirements, { topology: 'c3_paired', seed: 7 }).createRandomLayout());
    const optimizer = new Optimizer(requirements, { ...runOptions, referenceLayout: exported });
    assert.deepEqual(optimizer.referenceDiagnostics.boundsConflicts, [], `±${bound}° reported a conflict`);
    assert.equal(optimizer.referenceLayout.servoRangeDeg[1], exported.servo_range[1]);
  }
});

test('reference bounds tolerate rounding noise but still flag real violations', () => {
  const reference = layoutToJSON(new Optimizer({}, { topology: 'free', seed: 2 }).createRandomLayout());
  for (const point of reference.platform_anchors) point[2] = 1e-12;
  const clean = new Optimizer({}, { ...runOptions, referenceLayout: reference });
  assert.deepEqual(clean.referenceDiagnostics.boundsConflicts, []);
  reference.platform_anchors[0][2] = 1e-6;
  reference.servo_range = [-120, 120.000001];
  const flagged = new Optimizer({}, { ...runOptions, referenceLayout: reference }).referenceDiagnostics.boundsConflicts;
  assert.deepEqual(flagged.map(item => item.field), ['servo_range', 'platform_anchors[0][2]']);
});
