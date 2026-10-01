import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { DEFAULT_DESIGN_SPACE, crossoverLayouts, createRandomLayout, finalizeLayout, mutateLayout } from '../../src/optimization/layout-operators.js';
import { topologyGeometry, validateTopology, wrapAngle } from '../../src/optimization/topology.js';
import { createRandom } from '../../src/optimization/random.js';
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
    // Outside the pair-gap bounds, but narrow enough that no two legs collide.
    base_pair_gap: 120, platform_pair_gap: 80,
    base_orientation: 0, beta_offset: 0 };
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

const close = (actual, expected, tolerance = 1e-9, message = '') =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${message} ${actual} vs ${expected}`);

function parametricLayout(topology, parameters) {
  return { topology, topologyParameters: parameters, ...topologyGeometry(topology, parameters),
    hornLength: 50, rodLength: 200, homeHeight: 180, servoRangeRad: [-2, 2] };
}

test('C3 pair gaps are capped at 1.2 times the plate radius by generation, mutation and finalization', () => {
  const space = { ...DEFAULT_DESIGN_SPACE, baseRadius: [100, 100], platformRadius: [20, 20] };
  const context = { designSpace: space, servoRangeRad: [-2, 2] };
  const source = parametricLayout('c3_paired', { base_radius: 100, platform_radius: 20,
    base_pair_gap: 35, platform_pair_gap: 35, base_orientation: 0, beta_offset: 0 });
  const finalized = finalizeLayout(structuredClone(source), context);
  assert.equal(finalized.topologyParameters.base_pair_gap, 35, 'inside [12, min(45, 1.2 * 100)]');
  assert.equal(finalized.topologyParameters.platform_pair_gap, 24, 'clamped to 1.2 * 20');
  assert.deepEqual(topologyGeometry('c3_paired', finalized.topologyParameters).platformAnchors, finalized.platformAnchors);
  assert.equal(validateTopology(finalized), 'c3_paired');
  // Random draws and mutations stay under the cap and use the room beneath it.
  const random = createRandom(3);
  const gaps = [];
  for (let i = 0; i < 40; i++) {
    const layout = createRandomLayout({ ...context, topology: 'c3_paired', random });
    gaps.push(layout.topologyParameters.platform_pair_gap);
    gaps.push(mutateLayout(layout, { ...context, random }).topologyParameters.platform_pair_gap);
  }
  assert.ok(gaps.every(gap => gap >= 12 && gap <= 24), `${gaps}`);
  assert.ok(Math.max(...gaps) > 21, `the cap, not the floor, bounds the draws: ${Math.max(...gaps)}`);
  // A radius too small for the gap floor at this ratio is rejected rather than clamped.
  assert.throws(() => finalizeLayout(structuredClone(source),
    { ...context, designSpace: { ...space, platformRadius: [9, 9] } }), /cannot fit/);
});

test('operators wrap topology orientation angles and free beta angles into (-pi, pi]', () => {
  const context = { designSpace: DEFAULT_DESIGN_SPACE, servoRangeRad: [-2, 2] };
  const inRange = angle => angle > -Math.PI && angle <= Math.PI;
  const unwrapped = { base_radius: 110, platform_radius: 60, base_pair_gap: 20, platform_pair_gap: 20,
    base_orientation: 0.3 + 2 * Math.PI, beta_offset: 0.1 + 2 * Math.PI };
  const source = parametricLayout('c3_paired', unwrapped);
  const finalized = finalizeLayout(structuredClone(source), context);
  close(finalized.topologyParameters.base_orientation, 0.3);
  close(finalized.topologyParameters.beta_offset, 0.1);
  assert.equal('platform_orientation' in finalized.topologyParameters, false);
  finalized.baseAnchors.forEach((point, i) => point.forEach((value, k) => close(value, source.baseAnchors[i][k])));
  finalized.betaAngles.forEach((angle, i) => close(angle, source.betaAngles[i]));
  // Paired-horn families also wrap beta_pair_offset.
  const circular = parametricLayout('circular', { base_radius: 110, platform_radius: 60, base_orientation: 0,
    platform_orientation: 0.4, beta_offset: 0, beta_pair_offset: 0.5 - 2 * Math.PI });
  close(finalizeLayout(structuredClone(circular), context).topologyParameters.beta_pair_offset, 0.5);
  close(crossoverLayouts(circular, circular, { ...context, random: createRandom(1) }).topologyParameters.beta_pair_offset, 0.5);
  // Mutating an orientation at +pi or just above -pi lands back inside the range.
  for (let seed = 1; seed <= 12; seed++) {
    const nearPi = parametricLayout('rectangular_paired', { base_radius: 110, platform_radius: 60,
      base_aspect: 1, platform_aspect: 1, beta_pair_offset: 0,
      base_orientation: Math.PI, platform_orientation: -Math.PI + 1e-9, beta_offset: Math.PI });
    const child = mutateLayout(nearPi, { ...context, random: createRandom(seed) });
    for (const field of ['base_orientation', 'platform_orientation', 'beta_offset']) {
      const value = child.topologyParameters[field];
      assert.ok(inRange(value), `${field} ${value} (seed ${seed})`);
      assert.ok(Math.abs(wrapAngle(value - nearPi.topologyParameters[field])) < 0.5, `${field} moved ${value}`);
    }
    const pairChild = mutateLayout(parametricLayout('circular', { ...circular.topologyParameters, beta_pair_offset: Math.PI }),
      { ...context, random: createRandom(seed) });
    assert.ok(inRange(pairChild.topologyParameters.beta_pair_offset), `beta_pair_offset ${pairChild.topologyParameters.beta_pair_offset}`);
  }
  // Free topology: generated beta angles and mutated angles at the seam are wrapped.
  for (let seed = 1; seed <= 12; seed++) {
    const random = createRandom(seed);
    const layout = createRandomLayout({ ...context, topology: 'free', random });
    assert.ok(layout.betaAngles.every(inRange), `generated ${layout.betaAngles}`);
    layout.betaAngles = layout.betaAngles.map((_, i) => (i % 2 ? Math.PI : -Math.PI + 1e-9));
    const child = mutateLayout(layout, { ...context, random });
    assert.ok(child.betaAngles.every(inRange), `mutated ${child.betaAngles}`);
    child.betaAngles.forEach((angle, i) => assert.ok(Math.abs(wrapAngle(angle - layout.betaAngles[i])) < 0.5));
  }
});

test('C3 legs part from each base pair to the neighbouring platform pairs, with mirrored servos', () => {
  const azimuth = ([x, y]) => Math.atan2(y, x);
  const parameters = { base_radius: 120, platform_radius: 70, base_pair_gap: 30, platform_pair_gap: 24,
    base_orientation: 0.4, beta_offset: 0.3 };
  const { baseAnchors, platformAnchors, betaAngles } = topologyGeometry('c3_paired', parameters);
  for (let k = 0; k < 3; k++) {
    const axis = parameters.base_orientation + k * 2 * Math.PI / 3;
    const [low, high] = [2 * k, 2 * k + 1];
    // The pair straddles its axis, and each leg leans 60 deg away from it, so
    // the two legs of a pair reach different platform pairs.
    close(wrapAngle(azimuth(baseAnchors[high]) - axis), -wrapAngle(azimuth(baseAnchors[low]) - axis));
    const halfP = Math.asin(parameters.platform_pair_gap / (2 * parameters.platform_radius));
    close(wrapAngle(azimuth(platformAnchors[high]) - axis), Math.PI / 3 - halfP);
    close(wrapAngle(azimuth(platformAnchors[low]) - axis), -(Math.PI / 3 - halfP));
    // Each platform pair joins legs from two neighbouring base pairs.
    const next = (2 * k + 2) % 6;
    close(Math.hypot(platformAnchors[high][0] - platformAnchors[next][0],
      platformAnchors[high][1] - platformAnchors[next][1]), parameters.platform_pair_gap, 1e-9);
    // Mirrored servos: reflecting one horn direction across the pair axis gives its partner's.
    close(wrapAngle(betaAngles[high] - axis), -wrapAngle(betaAngles[low] - axis));
    // Each horn points away from its partner.
    for (const leg of [low, high]) {
      const tangent = [-Math.sin(azimuth(baseAnchors[leg])), Math.cos(azimuth(baseAnchors[leg]))];
      const away = leg === high ? 1 : -1;
      assert.ok(away * (Math.cos(betaAngles[leg]) * tangent[0] + Math.sin(betaAngles[leg]) * tangent[1]) > 0, `leg ${leg}`);
    }
  }
  assert.throws(() => topologyGeometry('c3_paired', { ...parameters, beta_offset: 2 }), /beta_offset.*90/);
  const context = { designSpace: DEFAULT_DESIGN_SPACE, servoRangeRad: [-2, 2] };
  const edge = parametricLayout('c3_paired', { ...parameters, beta_offset: Math.PI / 2 });
  for (let seed = 1; seed <= 12; seed++) {
    const child = mutateLayout(edge, { ...context, random: createRandom(seed) });
    assert.ok(Math.abs(child.topologyParameters.beta_offset) <= Math.PI / 2, `seed ${seed}`);
    assert.equal(validateTopology(child), 'c3_paired');
  }
});
