import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { DEFAULT_DESIGN_SPACE, crossoverLayouts } from '../../src/optimization/layout-operators.js';
import { topologyGeometry, validateTopology } from '../../src/optimization/topology.js';
import { layoutToJSON } from '../../src/io/results.js';
import { importLayout } from '../../src/io/layout-import.js';
import { evaluatePose } from '../../src/model/pose.js';

const families = ['circular', 'c3_paired', 'rectangular_paired', 'free'];

function within(value, [min, max]) {
  assert.ok(value >= min - 1e-9 && value <= max + 1e-9, `${value} outside [${min}, ${max}]`);
}

function checkBounds(layout, space) {
  within(layout.hornLength, space.hornLengthBounds);
  within(layout.rodLength, space.rodLengthBounds);
  within(layout.homeHeight, space.homeHeightBounds);
  if (layout.topology === 'free') {
    for (const point of layout.baseAnchors) within(Math.hypot(point[0], point[1]), space.baseRadius);
    for (const point of layout.platformAnchors) within(Math.hypot(point[0], point[1]), space.platformRadius);
  } else {
    within(layout.topologyParameters.base_radius, space.baseRadius);
    within(layout.topologyParameters.platform_radius, space.platformRadius);
    for (const point of layout.baseAnchors) assert.equal(point[2], 0);
    for (const point of layout.platformAnchors) assert.equal(point[2], 0);
  }
}

test('all topology families preserve their invariants and effective bounds through operators and generations', async () => {
  for (const topology of families) {
    const optimizer = new Optimizer({}, {
      topology, populationSize: 4, generations: 2, mutationRate: 1,
      designSpace: { baseRadius: [95, 100], platformRadius: [55, 60],
        homeHeightBounds: [175, 185], hornLengthBounds: [45, 50], rodLengthBounds: [200, 210] },
    });
    const a = optimizer.createRandomLayout();
    const b = optimizer.createRandomLayout();
    for (const layout of [a, b, optimizer.mutateLayout(a), optimizer.crossoverLayouts(a, b)]) {
      assert.equal(layout.topology, topology);
      assert.equal(validateTopology(layout), topology);
      checkBounds(layout, optimizer.designSpace);
    }
    await optimizer.run();
    assert.equal(optimizer.generation, 2);
    for (const layout of optimizer.population) {
      assert.equal(validateTopology(layout), topology);
      checkBounds(layout, optimizer.designSpace);
    }
  }
});

test('height remains a separate inherited variable, clamped only to editable bounds', () => {
  const optimizer = new Optimizer({}, { homeHeightBounds: [50, 450] });
  const a = optimizer.createRandomLayout();
  const b = optimizer.createRandomLayout();
  a.homeHeight = 90;
  b.homeHeight = 330;
  assert.equal(optimizer.finalizeLayout(a).homeHeight, 90);
  assert.equal(optimizer.finalizeLayout(b).homeHeight, 330);
  assert.ok([90, 330].includes(optimizer.crossoverLayouts(a, b).homeHeight));
  assert.notEqual(optimizer.mutateLayout(a).homeHeight, a.homeHeight);
  assert.equal(optimizer.finalizeLayout({ ...a, homeHeight: 999 }).homeHeight, 450);
  assert.deepEqual(DEFAULT_DESIGN_SPACE.homeHeightBounds, [50, 450]);
  assert.deepEqual(DEFAULT_DESIGN_SPACE.baseRadius, [90, 160]);
  assert.deepEqual(DEFAULT_DESIGN_SPACE.platformRadius, [40, 120]);
  assert.deepEqual(DEFAULT_DESIGN_SPACE.hornLengthBounds, [30, 120]);
  assert.deepEqual(DEFAULT_DESIGN_SPACE.rodLengthBounds, [160, 420]);
});

test('offspring receive fresh IDs within the run', () => {
  const optimizer = new Optimizer({}, { populationSize: 4 });
  const parents = Array.from({ length: 4 }, () => optimizer.createRandomLayout());
  optimizer.tournamentSelect = () => ({ layout: parents[0] });
  const offspring = optimizer.createOffspring([]);
  assert.equal(new Set([...parents, ...offspring].map(layout => layout.id)).size, 8);
});

test('declared topology parameters recreate geometry exactly and inconsistent metadata names the field', () => {
  const optimizer = new Optimizer({}, { topology: 'c3_paired' });
  const layout = optimizer.createRandomLayout();
  const parsed = JSON.parse(JSON.stringify(layout));
  assert.deepEqual(topologyGeometry(parsed.topology, parsed.topologyParameters), {
    baseAnchors: parsed.baseAnchors, platformAnchors: parsed.platformAnchors,
    betaAngles: parsed.betaAngles,
  });
  assert.equal(validateTopology(parsed), 'c3_paired');
  parsed.baseAnchors[1][0] += 0.1;
  assert.throws(() => validateTopology(parsed), /baseAnchors\[1\].*topology_parameters/);
  const malformed = JSON.parse(JSON.stringify(layout));
  malformed.topologyParameters.base_pair_gap = -1;
  assert.throws(() => validateTopology(malformed), /topology_parameters\.base_pair_gap/);
});

test('exported layouts round-trip topology, parameters, geometry, units, and home height', () => {
  for (const topology of families) {
    const layout = new Optimizer({}, { topology }).createRandomLayout();
    const json = JSON.parse(JSON.stringify(layoutToJSON(layout)));
    assert.equal(json.schema_version, 2);
    assert.equal(json.model_version, 2);
    assert.equal(json.topology, topology);
    assert.deepEqual(json.topology_parameters, layout.topologyParameters);
    assert.deepEqual(json.base_anchors, layout.baseAnchors);
    assert.deepEqual(json.platform_anchors, layout.platformAnchors);
    assert.deepEqual(json.beta_angles, layout.betaAngles);
    assert.equal(json.home_height, layout.homeHeight);
    assert.equal(json.horn_length, layout.hornLength);
    assert.equal(json.rod_length, layout.rodLength);
    // Generated layouts export the run's exact degree bounds, not the radians converted back.
    assert.deepEqual(json.servo_range, [-120, 120]);
    assert.deepEqual(layout.servoRangeRad, [-120, 120].map(degrees => degrees * Math.PI / 180));
    assert.equal(validateTopology(json), topology);
  }
});

test('headless designSpace horn and rod bounds are honoured; requirement bounds still win', () => {
  const option = new Optimizer({}, { designSpace: { hornLengthBounds: [50, 60], rodLengthBounds: [200, 210],
    homeHeightBounds: [100, 110] } });
  assert.deepEqual(option.designSpace.hornLengthBounds, [50, 60]);
  assert.deepEqual(option.designSpace.rodLengthBounds, [200, 210]);
  assert.deepEqual(option.designSpace.homeHeightBounds, [100, 110]);
  for (let i = 0; i < 20; i++) checkBounds(option.createRandomLayout(), option.designSpace);
  const requirement = new Optimizer({ horn_length_bounds_mm: [40, 45], rod_length_bounds_mm: [300, 310] },
    { designSpace: { hornLengthBounds: [50, 60], rodLengthBounds: [200, 210] } });
  assert.deepEqual(requirement.designSpace.hornLengthBounds, [40, 45]);
  assert.deepEqual(requirement.designSpace.rodLengthBounds, [300, 310]);
  assert.deepEqual(new Optimizer().designSpace.hornLengthBounds, DEFAULT_DESIGN_SPACE.hornLengthBounds);
  assert.throws(() => new Optimizer({}, { designSpace: { hornLengthBounds: [60, 50] } }), /horn/i);
});

test('new searches default to C3 paired and reject unsupported topology and invalid height bounds', () => {
  assert.equal(new Optimizer().topology, 'c3_paired');
  assert.throws(() => new Optimizer({}, { topology: 'hexagonal' }), /topology must be/);
  assert.throws(() => new Optimizer({}, { homeHeightBounds: [450, 50] }), /homeHeightBounds/);
  assert.throws(() => new Optimizer({}, { homeHeightBounds: [0, 450] }), /homeHeightBounds/);
});

test('Circular and Rectangular generators admit nonsingular home poses', () => {
  for (const topology of ['circular', 'rectangular_paired']) {
    const optimizer = new Optimizer({}, { topology, seed: 1 });
    const layout = optimizer.createRandomLayout();
    Object.assign(layout.topologyParameters, { base_radius: 100, platform_radius: 50,
      base_orientation: 0, platform_orientation: 0.4, beta_offset: 0,
      ...(topology === 'rectangular_paired' ? { base_aspect: 1.2, platform_aspect: 0.8 } : {}) });
    Object.assign(layout, topologyGeometry(topology, layout.topologyParameters),
      { hornLength: 50, rodLength: 200, homeHeight: 200 });
    const home = evaluatePose(layout);
    assert.equal(home.reachable, true, `${topology}: ${JSON.stringify(home.violations)}`);
    assert.ok(home.conditioning.reciprocal > 1e-4);

    const imported = importLayout(layoutToJSON(layout)).layout;
    assert.deepEqual(imported.betaAngles, layout.betaAngles);
    assert.equal(evaluatePose(imported).reachable, true);
    for (const child of [optimizer.mutateLayout(layout), optimizer.crossoverLayouts(layout, imported)]) {
      assert.equal(validateTopology(child), topology);
      assert.ok(Number.isFinite(child.topologyParameters.beta_pair_offset));
      checkBounds(child, optimizer.designSpace);
    }

    let validHomes = 0;
    for (let i = 0; i < 100; i++) {
      if (evaluatePose(optimizer.createRandomLayout()).reachable) validHomes++;
    }
    assert.ok(validHomes > 0, `${topology} search never generated a valid home`);
  }
});

test('legacy symmetric imports retain their exact horn directions and optional offset semantics', () => {
  for (const topology of ['circular', 'rectangular_paired']) {
    const layout = new Optimizer({}, { topology }).createRandomLayout();
    delete layout.topologyParameters.beta_pair_offset;
    Object.assign(layout, topologyGeometry(topology, layout.topologyParameters));
    const saved = layoutToJSON(layout);
    const imported = importLayout(saved).layout;
    assert.deepEqual(layoutToJSON(imported).beta_angles, saved.beta_angles);
    assert.deepEqual(imported.topologyParameters, saved.topology_parameters);
    assert.deepEqual(imported.baseAnchors, saved.base_anchors);
    assert.deepEqual(imported.platformAnchors, saved.platform_anchors);
    assert.throws(() => topologyGeometry(topology,
      { ...layout.topologyParameters, beta_pair_offset: NaN }), /beta_pair_offset/);
    const generated = new Optimizer({}, { topology }).createRandomLayout();
    const optimizer = new Optimizer({}, { topology });
    for (const [a, b] of [[imported, generated], [generated, imported]]) {
      const child = optimizer.crossoverLayouts(a, b);
      assert.equal(validateTopology(child), topology);
      assert.ok(Number.isFinite(child.topologyParameters.beta_pair_offset));
    }
  }
});

function wideGapReference() {
  const parameters = { base_radius: 150, platform_radius: 100,
    base_pair_gap: 250, platform_pair_gap: 150,
    base_orientation: 0, platform_orientation: 0.6, beta_offset: 0 };
  return { topology: 'c3_paired', topologyParameters: parameters,
    ...topologyGeometry('c3_paired', parameters), hornLength: 50,
    rodLength: 200, homeHeight: 180, servoRangeRad: [-2, 2] };
}

test('crossover bounds coupled C3 radii and gaps before constructing offspring', () => {
  const reference = wideGapReference();
  const parameters = { ...reference.topologyParameters, base_radius: 90, platform_radius: 40,
    base_pair_gap: 12, platform_pair_gap: 12 };
  const other = { ...reference, topologyParameters: parameters,
    ...topologyGeometry('c3_paired', parameters) };
  const before = structuredClone([reference, other]);
  let draws = 0;
  const child = crossoverLayouts(reference, other, { designSpace: DEFAULT_DESIGN_SPACE,
    servoRangeRad: [-2, 2], random: () => draws++ < 2 ? 0.9 : 0.1 });
  assert.equal(validateTopology(child), 'c3_paired');
  for (const plate of ['base', 'platform']) {
    within(child.topologyParameters[`${plate}_pair_gap`], DEFAULT_DESIGN_SPACE.pairGapBounds);
    assert.ok(child.topologyParameters[`${plate}_pair_gap`] <= 1.2 * child.topologyParameters[`${plate}_radius`]);
  }
  assert.deepEqual([reference, other], before);
});

test('a wide-gap diagnostic reference completes seeded evolution and stays unchanged', async () => {
  const reference = layoutToJSON(wideGapReference());
  const optimizer = new Optimizer({}, { referenceLayout: reference, seed: 7,
    populationSize: 4, generations: 5, sampling: { strategy: 'halton', sampleCount: 4 },
    ballJointLimitDeg: 180 });
  assert.equal(optimizer.referenceDiagnostics.homePoseSatisfied, true);
  assert.equal(optimizer.referenceDiagnostics.boundsConflicts.length, 2);
  const outcome = await optimizer.run();
  assert.equal(outcome.status, 'completed');
  assert.equal(outcome.completedGenerations, 5);
  const retained = optimizer.fitness.find(item => item.layout.seedOrigin === 'reference');
  for (const field of ['base_anchors', 'platform_anchors', 'beta_angles', 'home_height']) {
    assert.deepEqual(layoutToJSON(retained.layout)[field], reference[field]);
  }
  for (const item of optimizer.fitness.filter(item => item !== retained)) {
    checkBounds(item.layout, optimizer.designSpace);
    within(item.layout.topologyParameters.base_pair_gap, optimizer.designSpace.pairGapBounds);
    within(item.layout.topologyParameters.platform_pair_gap, optimizer.designSpace.pairGapBounds);
  }
});
