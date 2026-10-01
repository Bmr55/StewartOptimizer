import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { DEFAULT_DESIGN_SPACE, mutateLayout, crossoverLayouts, finalizeLayout } from '../../src/optimization/layout-operators.js';
import { initialPopulation, referenceBoundsConflicts, seedComposition } from '../../src/optimization/reference-seeding.js';
import { topologyGeometry } from '../../src/optimization/topology.js';
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

test('variations and fresh seeds carry neither the reference diagnostics nor its migration record', () => {
  const reference = layoutToJSON(new Optimizer({}, { topology: 'c3_paired', seed: 4 }).createRandomLayout());
  assert.equal(reference.mounting, null, 'a generated layout exports no mounting, so the import records a migration');
  const optimizer = new Optimizer({}, { ...runOptions, populationSize: 9, referenceLayout: reference });
  assert.equal(optimizer.referenceLayout.migration?.upgraded, false);
  assert.match(optimizer.referenceLayout.migration.note, /derived from the home geometry/);
  assert.equal(optimizer.referenceLayout.referenceDiagnostics, optimizer.referenceDiagnostics);
  const composition = seedComposition(9);
  assert.deepEqual(composition, { reference: 1, variations: 6, fresh: 2 });
  const population = initialPopulation(optimizer);
  assert.deepEqual(population.map(layout => layout.seedOrigin),
    ['reference', ...new Array(6).fill('variation'), ...new Array(2).fill('fresh')]);
  assert.equal(new Set(population.map(layout => layout.id)).size, 9);
  // The retained reference keeps both records for diagnosis; every generated candidate drops them.
  assert.deepEqual(population[0].referenceDiagnostics, optimizer.referenceDiagnostics);
  assert.deepEqual(population[0].migration, optimizer.referenceLayout.migration);
  for (const layout of population.slice(1)) {
    assert.equal(Object.hasOwn(layout, 'referenceDiagnostics'), false, `${layout.seedOrigin} ${layout.id} keeps referenceDiagnostics`);
    assert.equal(Object.hasOwn(layout, 'migration'), false, `${layout.seedOrigin} ${layout.id} keeps migration`);
    assert.equal(layoutToJSON(layout).migration, null);
  }
});

test('a reference servo range below the lower travel bound is a conflict, like one above the upper bound', () => {
  const requirements = { servo_travel_bounds_deg: [-120, 120] };
  const reference = layoutToJSON(new Optimizer(requirements, { topology: 'circular', seed: 9 }).createRandomLayout());
  const conflicts = servoRange => new Optimizer(requirements, { ...runOptions,
    referenceLayout: { ...reference, servo_range: servoRange } }).referenceDiagnostics.boundsConflicts;
  assert.deepEqual(conflicts([-120, 120]), []);
  assert.deepEqual(conflicts([-110, 110]), [], 'a narrower range is inside the search bounds');
  for (const servoRange of [[-130, 120], [-120.001, 120], [-120, 130], [-130, 130]]) {
    const flagged = conflicts(servoRange);
    assert.equal(flagged.length, 1, `${servoRange} flagged ${JSON.stringify(flagged)}`);
    assert.equal(flagged[0].field, 'servo_range');
    assert.equal(flagged[0].unit, 'deg');
    assert.deepEqual(flagged[0].value, servoRange);
    flagged[0].bounds.forEach((bound, i) => assert.ok(Math.abs(bound - [-120, 120][i]) < 1e-9));
  }
  // The direct check works in radians and is the same test on the lower bound.
  const imported = importLayout({ ...reference, servo_range: [-130, 120] }).layout;
  const radians = [-120, 120].map(degrees => degrees * Math.PI / 180);
  assert.deepEqual(referenceBoundsConflicts(imported, DEFAULT_DESIGN_SPACE, radians).map(item => item.field), ['servo_range']);
  assert.deepEqual(referenceBoundsConflicts(imported, DEFAULT_DESIGN_SPACE, [-2.3, 2.1]), []);
});

test('C3 pair-gap conflicts are reported per plate from that plate\'s own parameter', () => {
  const build = (base_pair_gap, platform_pair_gap) => {
    const parameters = { base_radius: 120, platform_radius: 80, base_pair_gap, platform_pair_gap,
      base_orientation: 0, beta_offset: 0 };
    return { topology: 'c3_paired', topologyParameters: parameters, ...topologyGeometry('c3_paired', parameters),
      hornLength: 50, rodLength: 200, homeHeight: 180, servoRangeRad: [-2, 2] };
  };
  const bounds = DEFAULT_DESIGN_SPACE.pairGapBounds;
  assert.ok(bounds[0] > 8 && bounds[1] < 60, `fixture gaps 8 and 60 lie outside ${bounds}`);
  const fields = layout => referenceBoundsConflicts(layout, DEFAULT_DESIGN_SPACE, [-2, 2]).map(item => item.field);
  assert.deepEqual(fields(build(20, 30)), []);
  assert.deepEqual(fields(build(20, 60)), ['topology_parameters.platform_pair_gap']);
  assert.deepEqual(fields(build(60, 20)), ['topology_parameters.base_pair_gap']);
  assert.deepEqual(referenceBoundsConflicts(build(8, 60), DEFAULT_DESIGN_SPACE, [-2, 2]), [
    { field: 'topology_parameters.base_pair_gap', value: 8, bounds },
    { field: 'topology_parameters.platform_pair_gap', value: 60, bounds }]);
  const optimizer = new Optimizer({}, { ...runOptions, referenceLayout: layoutToJSON(build(20, 60)) });
  assert.deepEqual(optimizer.referenceDiagnostics.boundsConflicts.map(item => [item.field, item.value]),
    [['topology_parameters.platform_pair_gap', 60]]);
});
