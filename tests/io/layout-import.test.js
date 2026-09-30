import test from 'node:test';
import assert from 'node:assert/strict';
import { importLayout } from '../../src/io/layout-import.js';
import { layoutToJSON } from '../../src/io/results.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { initialPopulation, seedComposition } from '../../src/optimization/reference-seeding.js';
import { jointFixture } from '../fixtures/layout.js';

function asymmetric() {
  const source = layoutToJSON(jointFixture());
  source.base_anchors[0][0] += 7.125;
  source.platform_anchors[1][1] -= 3.75;
  source.beta_angles[2] += 0.123;
  source.servo_range = [-137, 113];
  source.home_height = 177.5;
  source.topology = undefined;
  source.topology_parameters = undefined;
  source.mounting = undefined;
  source.model_version = 1;
  source.metadata = { coverage: 9999, torque: -1 };
  return source;
}

test('plain, downloaded, and displayed-result wrappers import exact geometry and discard old metrics', async () => {
  const plain = asymmetric();
  const variants = [plain, { ...plain, metadata: { coverage: -100 } },
    { layout: plain }, { run: { status: 'completed' }, result: { layout: plain, metrics: { coverage: 9999 } } }];
  for (const input of variants) {
    const { layout } = importLayout(JSON.stringify(input));
    assert.equal(layout.topology, 'free');
    assert.deepEqual(layout.baseAnchors, plain.base_anchors);
    assert.deepEqual(layout.platformAnchors, plain.platform_anchors);
    assert.deepEqual(layout.betaAngles, plain.beta_angles);
    assert.equal(layout.hornLength, plain.horn_length);
    assert.equal(layout.rodLength, plain.rod_length);
    assert.equal(layout.homeHeight, plain.home_height);
    assert.deepEqual(layout.servoRangeDeg, plain.servo_range);
    assert.equal(layout.migration.upgraded, true);
    assert.equal(layout.migration.fromModelVersion, 1);
    assert.equal('metadata' in layout, false);
  }
  const optimizer = new Optimizer({}, { referenceLayout: plain, populationSize: 4, generations: 1 });
  const evaluated = await optimizer.evaluateLayout(optimizer.referenceLayout);
  assert.notEqual(evaluated.coverage, 9999);
  assert.deepEqual(layoutToJSON(evaluated.layout).base_anchors, plain.base_anchors);
  assert.deepEqual(layoutToJSON(evaluated.layout).servo_range, plain.servo_range);
});

test('malformed layouts and inconsistent declared topology fail before evaluation with field names', () => {
  const baseline = asymmetric();
  const bad = (field, value) => assert.throws(() => importLayout({ ...baseline, [field]: value }), new RegExp(field));
  bad('base_anchors', baseline.base_anchors.slice(0, 5));
  bad('platform_anchors', [[Infinity, 0, 0], ...baseline.platform_anchors.slice(1)]);
  bad('beta_angles', [NaN, ...baseline.beta_angles.slice(1)]);
  bad('horn_length', 0);
  bad('rod_length', -5);
  bad('servo_range', [20, -20]);
  bad('home_height', null);
  assert.throws(() => importLayout('{broken'), /Layout JSON/);
  assert.throws(() => new Optimizer({}, { referenceLayout: { ...baseline, base_anchors: [] } }), /base_anchors/);
  const declared = layoutToJSON(new Optimizer().createRandomLayout());
  declared.base_anchors[0][0] += 1;
  assert.throws(() => importLayout(declared), /baseAnchors\[0\].*topology_parameters/);
});

test('population 12 is 1 exact, 8 bounded variations, and 3 fresh of the reference topology', () => {
  const reference = asymmetric();
  reference.home_height = 600;
  reference.horn_length = 140;
  const optimizer = new Optimizer({}, { referenceLayout: reference, populationSize: 12,
    homeHeightBounds: [100, 200], designSpace: { baseRadius: [95, 105], platformRadius: [45, 55] } });
  const population = initialPopulation(optimizer);
  assert.deepEqual(seedComposition(12), { reference: 1, variations: 8, fresh: 3 });
  assert.deepEqual(population.map(item => item.seedOrigin),
    ['reference', ...Array(8).fill('variation'), ...Array(3).fill('fresh')]);
  assert.deepEqual(population[0].baseAnchors, reference.base_anchors);
  assert.equal(population[0].homeHeight, 600);
  assert.ok(optimizer.referenceDiagnostics.boundsConflicts.some(item => item.field === 'home_height'));
  assert.ok(optimizer.referenceDiagnostics.boundsConflicts.some(item => item.field === 'horn_length'));
  for (const layout of population.slice(1)) {
    assert.equal(layout.topology, 'free');
    assert.ok(layout.homeHeight >= 100 && layout.homeHeight <= 200);
    assert.ok(layout.hornLength >= 30 && layout.hornLength <= 120);
    for (const point of layout.baseAnchors) assert.ok(Math.hypot(point[0], point[1]) <= 105 + 1e-9);
    for (const point of layout.platformAnchors) assert.ok(Math.hypot(point[0], point[1]) <= 55 + 1e-9);
  }
  assert.equal(new Set(population.map(item => item.id)).size, 12);
});

test('other population sizes use deterministic quarter rounding', () => {
  assert.deepEqual(seedComposition(4), { reference: 1, variations: 2, fresh: 1 });
  assert.deepEqual(seedComposition(8), { reference: 1, variations: 5, fresh: 2 });
  assert.deepEqual(seedComposition(20), { reference: 1, variations: 14, fresh: 5 });
});

test('declared paired reference seeds only the same topology without reshaping its anchors', () => {
  const original = layoutToJSON(new Optimizer({}, { topology: 'c3_paired' }).createRandomLayout());
  const optimizer = new Optimizer({}, { referenceLayout: original, topology: 'circular', populationSize: 8 });
  assert.equal(optimizer.topology, 'c3_paired');
  const population = initialPopulation(optimizer);
  assert.deepEqual(population[0].baseAnchors, original.base_anchors);
  assert.equal(population.filter(item => item.seedOrigin === 'variation').length, 5);
  assert.equal(population.filter(item => item.seedOrigin === 'fresh').length, 2);
  assert.ok(population.every(item => item.topology === 'c3_paired'));
});

test('diagnostic reference remains selectable and exports original geometry after generations', async () => {
  const source = asymmetric();
  source.home_height = 600;
  const optimizer = new Optimizer({}, { referenceLayout: source, populationSize: 4,
    generations: 2, ranges: {}, homeHeightBounds: [100, 200] });
  const evaluatedLayouts = [];
  const evaluate = optimizer.evaluateLayout.bind(optimizer);
  optimizer.evaluateLayout = layout => {
    evaluatedLayouts.push(layout);
    return evaluate(layout);
  };
  await optimizer.run();
  for (const layout of evaluatedLayouts.filter(item => item.seedOrigin !== 'reference')) {
    assert.ok(layout.homeHeight >= 100 && layout.homeHeight <= 200);
    assert.ok(layout.hornLength >= 30 && layout.hornLength <= 120);
    assert.ok(layout.rodLength >= 160 && layout.rodLength <= 420);
    assert.deepEqual(layout.servoRangeRad, optimizer.servoRangeRad);
  }
  const reference = optimizer.fitness.find(result => result.layout.seedOrigin === 'reference');
  assert.ok(reference);
  assert.equal(reference.feasibility.passing, false);
  assert.ok(reference.feasibility.failedCategories.includes('geometry'));
  assert.equal(reference.referenceDiagnostics.homePoseSatisfied, false);
  assert.ok(reference.referenceDiagnostics.homePoseViolations.length);
  optimizer.selectCandidate(reference.layout.id);
  const exported = JSON.parse(optimizer.exportBest());
  for (const field of ['base_anchors', 'platform_anchors', 'beta_angles', 'horn_length',
    'rod_length', 'servo_range', 'home_height']) assert.deepEqual(exported[field], source[field]);
  assert.equal(exported.seed_origin, 'reference');
  assert.equal(exported.diagnostic, true);
  assert.ok(exported.reference_diagnostics.boundsConflicts.length);
  assert.notEqual(exported.metadata.coverage, source.metadata.coverage);
  assert.equal(exported.migration.upgraded, true);
});

test('asymmetric reference run replays from exported effective settings and reimports without changing geometry', async () => {
  const source = asymmetric();
  const settings = { referenceLayout: source, populationSize: 4, generations: 1, seed: 73,
    sampling: { strategy: 'halton', sampleCount: 16 },
    ranges: { x: { min: -2, max: 2, step: 1 } },
    lowerBallJointLimitDeg: 70, upperBallJointLimitDeg: 80 };
  const original = new Optimizer({}, settings);
  await original.run();
  const exported = JSON.parse(original.exportBest());
  assert.deepEqual(exported.run.effective_settings.seed_composition,
    { reference: 1, variations: 2, fresh: 1 });
  assert.deepEqual(exported.run.effective_settings.reference_layout.base_anchors, source.base_anchors);
  assert.deepEqual(importLayout(exported).layout.baseAnchors, exported.base_anchors);
  const replay = Optimizer.fromReplay(exported);
  await replay.run();
  assert.deepEqual(JSON.parse(replay.exportBest()), exported);
  const displayedReplay = Optimizer.fromReplay({ run: exported.run, result: { layout: exported } });
  assert.deepEqual(displayedReplay.effectiveSettings(), original.effectiveSettings());
  assert.throws(() => Optimizer.fromReplay(source), /run.effective_settings/);
});

test('a locked servo range runs, exports, replays and reimports', async () => {
  const settings = { populationSize: 4, generations: 1, seed: 5, ranges: {}, sampling: { strategy: 'grid' } };
  const original = new Optimizer({ servo_travel_bounds_deg: [0, 0] }, settings);
  await original.run();
  const exported = JSON.parse(original.exportBest());
  assert.deepEqual(exported.servo_range, [0, 0]);
  assert.deepEqual(exported.run.effective_settings.servoRangeDeg, [0, 0]);
  const imported = importLayout(exported);
  assert.deepEqual(imported.layout.servoRangeRad, [0, 0]);
  assert.deepEqual(importLayout({ run: exported.run, result: { layout: exported } }).layout.servoRangeRad, [0, 0]);
  const replay = Optimizer.fromReplay(exported);
  await replay.run();
  assert.deepEqual(JSON.parse(replay.exportBest()), exported);
  const asReference = new Optimizer({ servo_travel_bounds_deg: [0, 0] }, { ...settings, referenceLayout: exported });
  assert.deepEqual(asReference.referenceDiagnostics.boundsConflicts, []);
  assert.throws(() => importLayout({ ...exported, servo_range: [1, 0] }), /servo_range must contain two finite bounds with max >= min/);
});

test('free-topology parameters must be an object, are copied, and version errors show the offending value', () => {
  const plain = asymmetric();
  for (const parameters of ['garbage', 42, true, [1, 2]]) {
    assert.throws(() => importLayout({ ...plain, topology: 'free', topology_parameters: parameters }),
      /topology_parameters must be an object/, JSON.stringify(parameters));
  }
  const input = { ...plain, topology: 'free', topology_parameters: { note: 'kept' } };
  const { layout } = importLayout(input);
  assert.deepEqual(layout.topologyParameters, { note: 'kept' });
  input.topology_parameters.note = 'changed';
  assert.equal(layout.topologyParameters.note, 'kept', 'topology_parameters were stored by reference');
  assert.deepEqual(importLayout({ ...plain, topology_parameters: null }).layout.topologyParameters, {});
  assert.throws(() => importLayout({ ...plain, schema_version: '2' }), /schema_version "2" is unsupported/);
  assert.throws(() => importLayout({ ...plain, model_version: 1.5 }), /model_version 1.5 is unsupported/);
  assert.throws(() => importLayout('{'), /Layout JSON is invalid/);
});
