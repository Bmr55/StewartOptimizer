import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { createRandom } from '../../src/optimization/random.js';
import { DEFAULT_DESIGN_SPACE, createRandomLayout, crossoverLayouts, mutateLayout } from '../../src/optimization/layout-operators.js';
import { initialPopulation } from '../../src/optimization/reference-seeding.js';
import { layoutToJSON } from '../../src/io/results.js';
import { TOPOLOGIES } from '../../src/contracts.js';
import { computeWorkspace } from '../../src/workspace/sweep.js';
import { asymmetricJointFixture, pairedFixture } from '../fixtures/layout.js';

// Golden fixture for the seed-stability rule in AGENTS.md: a change that alters
// results for an unchanged seed is a behaviour change. Every other replay test
// compares the code with itself, so a changed RNG constant, operator, seeding
// rule or reservoir seed would pass them. This file pins tiny seeded runs per
// topology, a reference-seeded run, a Halton reservoir, the raw RNG stream and
// the documented design-space defaults.
//
// After an intentional behaviour change, regenerate with
//   UPDATE_SEED_FIXTURE=1 node --test tests/optimization/seed-stability.test.js
// and review the fixture diff alongside the code and doc changes.
const FIXTURE_URL = new URL('../fixtures/seed-stability.json', import.meta.url);
const TOLERANCE = 1e-9;

const RUN = { seed: 3, populationSize: 4, generations: 1, ranges: {}, sampling: { strategy: 'grid' } };
const LAYOUT_FIELDS = ['id', 'topology', 'topology_parameters', 'base_anchors', 'platform_anchors',
  'beta_angles', 'horn_length', 'rod_length', 'home_height', 'servo_range'];
const METRIC_FIELDS = ['coverage', 'conditioningQuality', 'torque', 'speedDemand', 'objectives'];

function pick(source, fields) {
  return Object.fromEntries(fields.map(field => [field, source[field] ?? null]));
}

function snapshot(optimizer) {
  return {
    seedOrigins: optimizer.fitness.map(candidate => candidate.layout.seedOrigin ?? null),
    paretoIds: optimizer.pareto.map(candidate => candidate.layout.id),
    candidates: optimizer.fitness.map(candidate => ({
      layout: pick(layoutToJSON(candidate.layout), LAYOUT_FIELDS),
      metrics: pick(candidate, METRIC_FIELDS),
    })),
  };
}

async function generate() {
  const topologies = {};
  for (const topology of TOPOLOGIES) {
    const optimizer = new Optimizer({}, { ...RUN, topology });
    await optimizer.run();
    topologies[topology] = snapshot(optimizer);
  }
  const seeded = new Optimizer({}, { ...RUN, populationSize: 8, referenceLayout: asymmetricJointFixture() });
  await seeded.run();
  const reservoir = new Optimizer({}, { ...RUN, topology: 'c3_paired',
    ranges: { x: { min: -8, max: 8, step: 8 }, y: { min: -8, max: 8, step: 8 }, z: { min: -8, max: 8, step: 8 } },
    sampling: { strategy: 'halton', sampleCount: 256 } });
  await reservoir.run();
  // Operators and seeding pinned directly, so the fixture does not depend on
  // which layouts survive selection.
  const operatorOptions = { designSpace: DEFAULT_DESIGN_SPACE, servoRangeRad: [-2 * Math.PI / 3, 2 * Math.PI / 3] };
  const operators = {};
  for (const topology of TOPOLOGIES) {
    const random = createRandom(7);
    const a = createRandomLayout({ ...operatorOptions, id: 1, topology, random });
    const b = createRandomLayout({ ...operatorOptions, id: 2, topology, random });
    const layoutFields = LAYOUT_FIELDS.filter(field => field !== 'id');
    operators[topology] = {
      parents: [a, b].map(layout => pick(layoutToJSON(layout), layoutFields)),
      mutated: pick(layoutToJSON(mutateLayout(a, { ...operatorOptions, random })), layoutFields),
      crossed: pick(layoutToJSON(crossoverLayouts(a, b, { ...operatorOptions, random })), layoutFields),
    };
  }
  const seedingOptimizer = new Optimizer({}, { ...RUN, populationSize: 8, referenceLayout: asymmetricJointFixture() });
  const seedingPopulation = initialPopulation(seedingOptimizer).map(layout => ({
    seedOrigin: layout.seedOrigin ?? null, layout: pick(layoutToJSON(layout), LAYOUT_FIELDS),
  }));
  // A direct sweep without an injected random seeds its reservoir from the
  // Halton sequence start; the optimizer injects its own random instead.
  const direct = await computeWorkspace(pairedFixture(),
    { x: { min: -8, max: 8, step: 8 }, y: { min: -8, max: 8, step: 8 }, z: { min: -8, max: 8, step: 8 } },
    { sampling: { strategy: 'halton', sampleCount: 256 }, ballJointLimitDeg: 180 });
  const digest = samples => samples.reduce((sum, sample, slot) =>
    sum + (slot + 1) * Object.values(sample.pose).reduce((inner, value, axis) => inner + value * 10 ** axis, 0), 0);
  return {
    description: 'Seed-stability golden fixture; see tests/optimization/seed-stability.test.js.',
    directWorkspace: { coverage: direct.coverage, reachableSamples: direct.samples.reachable.length,
      reachableDigest: digest(direct.samples.reachable) },
    random: { seed1: Array.from({ length: 8 }, createRandom(1)), seed3: Array.from({ length: 4 }, createRandom(3)) },
    operators,
    seedingPopulation,
    topologies,
    referenceSeeded: snapshot(seeded),
    haltonReservoir: reservoir.fitness.map(candidate => ({
      id: candidate.layout.id,
      coverage: candidate.coverage,
      reachableSamples: candidate.workspace.samples.reachable.length,
      firstReachable: candidate.workspace.samples.reachable.slice(0, 3).map(sample => sample.pose),
      // Slot-weighted digest of every retained pose: a different reservoir
      // replacement anywhere changes it without pinning 200 poses.
      reachableDigest: candidate.workspace.samples.reachable.reduce((sum, sample, slot) =>
        sum + (slot + 1) * Object.values(sample.pose).reduce((inner, value, axis) => inner + value * 10 ** axis, 0), 0),
      violationDigest: candidate.workspace.samples.violations.reduce((sum, sample, slot) =>
        sum + (slot + 1) * Object.values(sample.pose).reduce((inner, value, axis) => inner + value * 10 ** axis, 0), 0),
    })),
  };
}

function closeDeep(actual, expected, path = 'fixture') {
  if (Array.isArray(expected)) {
    assert.ok(Array.isArray(actual), `${path} should be an array`);
    assert.equal(actual.length, expected.length, `${path} length`);
    expected.forEach((value, i) => closeDeep(actual[i], value, `${path}[${i}]`));
  } else if (expected && typeof expected === 'object') {
    assert.ok(actual && typeof actual === 'object', `${path} should be an object`);
    assert.deepEqual(Object.keys(actual).sort(), Object.keys(expected).sort(), `${path} keys`);
    for (const key of Object.keys(expected)) closeDeep(actual[key], expected[key], `${path}.${key}`);
  } else if (typeof expected === 'number' && Number.isFinite(expected) && !Number.isInteger(expected)) {
    assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= TOLERANCE * Math.max(1, Math.abs(expected)),
      `${path}: ${actual} differs from pinned ${expected}`);
  } else {
    assert.deepEqual(actual, expected, path);
  }
}

test('seeded runs, reference seeding, the Halton reservoir and the RNG stream match the pinned fixture', async () => {
  const actual = JSON.parse(JSON.stringify(await generate()));
  if (process.env.UPDATE_SEED_FIXTURE) {
    fs.writeFileSync(FIXTURE_URL, JSON.stringify(actual, null, 2) + '\n');
  }
  const expected = JSON.parse(fs.readFileSync(FIXTURE_URL, 'utf8'));
  assert.deepEqual(Object.keys(actual.topologies), [...TOPOLOGIES]);
  for (const topology of TOPOLOGIES) {
    // Each pinned population has evaluated offspring, so crossover and mutation are covered.
    assert.ok(actual.topologies[topology].seedOrigins.includes('offspring'), `${topology} ran a generation`);
  }
  assert.deepEqual(actual.referenceSeeded.seedOrigins.filter(origin => origin === 'reference').length, 1);
  assert.deepEqual(actual.seedingPopulation.map(entry => entry.seedOrigin),
    ['reference', 'variation', 'variation', 'variation', 'variation', 'variation', 'fresh', 'fresh'], '1 + 5 + 2 for a population of 8');
  assert.ok(actual.haltonReservoir.every(candidate => candidate.reachableSamples === 200 || candidate.coverage < 100),
    'the Halton run overflows the 200-sample reservoir');
  closeDeep(actual, expected);
});

test('DEFAULT_DESIGN_SPACE equals the constants documented in TOPOLOGIES.md and FEATURES.md', () => {
  const degrees = value => value * Math.PI / 180;
  assert.deepEqual(DEFAULT_DESIGN_SPACE, {
    baseRadius: [90, 160], platformRadius: [40, 120], homeHeightBounds: [50, 450],
    hornLengthBounds: [30, 120], rodLengthBounds: [160, 420],
    pairGapBounds: [12, 45], rectangularAspectBounds: [0.6, 1.4],
    betaJitterRad: degrees(20), anchorJitter: 6, platformJitter: 6, baseZJitter: 2,
    mutationHorn: 4, mutationRod: 6, mutationHeight: 15, mutationAngle: degrees(4),
  });
});
