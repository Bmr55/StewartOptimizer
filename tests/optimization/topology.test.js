import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { DEFAULT_DESIGN_SPACE } from '../../src/optimization/layout-operators.js';
import { topologyGeometry, validateTopology } from '../../src/optimization/topology.js';
import { layoutToJSON } from '../../src/io/results.js';

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
    assert.deepEqual(json.servo_range, layout.servoRangeRad.map(radians => radians * 180 / Math.PI));
    assert.equal(validateTopology(json), topology);
  }
});

test('new searches default to C3 paired and reject unsupported topology and invalid height bounds', () => {
  assert.equal(new Optimizer().topology, 'c3_paired');
  assert.throws(() => new Optimizer({}, { topology: 'hexagonal' }), /topology must be/);
  assert.throws(() => new Optimizer({}, { homeHeightBounds: [450, 50] }), /homeHeightBounds/);
  assert.throws(() => new Optimizer({}, { homeHeightBounds: [0, 450] }), /homeHeightBounds/);
});
