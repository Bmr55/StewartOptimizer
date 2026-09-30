import test from 'node:test';
import assert from 'node:assert/strict';
import { computeCycleDemand, evaluateDynamicPose } from '../../src/model/cycle.js';
import { legacyTrajectory, normalizeTrajectory, trajectoryState } from '../../src/model/trajectory.js';
import { normalizeMassProperties } from '../../src/model/mass-properties.js';
import { LEGACY_CYCLE_SAMPLING, periodicSampleWeights } from '../../src/model/cycle-sampling.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { degToRad } from '../../src/math.js';
import { computeFatigue, evaluateLayout } from '../../src/optimization/evaluate-layout.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { layoutToJSON } from '../../src/io/results.js';
import { pairedFixture } from '../fixtures/layout.js';
import { sampleText } from '../ui/helpers.js';

const close = (actual, expected, tolerance = 1e-9, message = '') =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${message} ${actual} vs ${expected}`);
const base = { ranges: {}, sampling: { strategy: 'grid' }, payload: 2, ballJointLimitDeg: 180 };
const moving = { ...base, stroke: 10, frequency: 2, cycleAxis: 'x', cycleSampling: LEGACY_CYCLE_SAMPLING };

test('fatigue is the cycle frequency times the mean RMS servo excursion and ignores the servo travel bounds', async () => {
  const layout = pairedFixture();
  const narrow = await evaluateLayout(layout, { ...moving, servoRangeRad: [-Math.PI / 2, Math.PI / 2] });
  const wide = await evaluateLayout(layout, { ...moving, servoRangeRad: [-Math.PI, Math.PI] });
  assert.ok(narrow.fatigue > 0);
  assert.equal(narrow.fatigue, wide.fatigue, 'the travel bound is not a usage measure');
  // Independent: the 64 uniform samples of the legacy x trajectory, time-weighted.
  const trajectory = legacyTrajectory({ stroke: 10, frequency: 2, axis: 'x' });
  const mass = normalizeMassProperties({ mass_kg: 2 });
  const times = Array.from({ length: 64 }, (_, k) => 0.5 * k / 64);
  const weights = periodicSampleWeights(times, 0.5);
  const angles = times.map(time =>
    evaluateDynamicPose(layout, trajectoryState(trajectory, time), mass, { ballJointLimitDeg: 180 }).servoAngles);
  const rms = Array.from({ length: 6 }, (_, leg) => {
    const mean = angles.reduce((sum, sample, k) => sum + weights[k] * sample[leg], 0);
    return Math.sqrt(angles.reduce((sum, sample, k) => sum + weights[k] * (sample[leg] - mean) ** 2, 0));
  });
  narrow.cycle.servoExcursionRmsRad.forEach((value, leg) => close(value, rms[leg], 1e-12, `servo ${leg + 1}`));
  close(narrow.fatigue, 2 * rms.reduce((sum, value) => sum + value, 0) / 6, 1e-12);
  assert.equal(computeFatigue(narrow.cycle), narrow.fatigue);
});

test('fatigue is nonzero for a rotation-only trajectory, zero at rest and null for an invalid cycle', async () => {
  const rotation = await evaluateLayout(pairedFixture(), { ...base, servoRangeRad: [-Math.PI, Math.PI],
    trajectory: normalizeTrajectory({ frequency_hz: 1, components: [{ axis: 'rz', amplitude_deg: 4 }] }) });
  assert.equal(rotation.cycle.valid, true);
  assert.ok(rotation.fatigue > 0, 'a rotation-only cycle still moves the servos');
  const still = await evaluateLayout(pairedFixture(), { ...base, stroke: 0, frequency: 0, servoRangeRad: [-Math.PI, Math.PI] });
  assert.equal(still.cycle.sampling.status, 'stationary');
  assert.deepEqual(still.cycle.servoExcursionRmsRad, [0, 0, 0, 0, 0, 0]);
  assert.equal(still.fatigue, 0);
  const invalid = computeCycleDemand(pairedFixture(), { mass: 2, stroke: 10, frequency: 2, axis: 'z', ballJointLimitDeg: 0 });
  assert.equal(invalid.valid, false);
  assert.equal(computeFatigue(invalid), null);
  const ranked = await evaluateLayout(pairedFixture(), { ...moving, cycleAxis: 'z', servoRangeRad: [-Math.PI, Math.PI],
    ballJointLimitDeg: 0, objectiveSet: 'full' });
  assert.equal(ranked.fatigue, null);
  assert.equal(ranked.objectives[8], -Infinity, 'an invalid cycle ranks worst on fatigue');
});

test('limit margin measures reachable headroom instead of collapsing when a sampled pose violates', async () => {
  const options = { sampling: { strategy: 'grid' }, payload: 2, stroke: 0, frequency: 0, servoRangeRad: [-Math.PI, Math.PI],
    ballJointLimitDeg: 12, ranges: { rx: { min: -25, max: 25, step: 12.5 } } };
  const partial = await evaluateLayout(pairedFixture(), options);
  const stats = partial.workspace.stats;
  assert.ok(partial.coverage > 0 && partial.coverage < 100, `coverage ${partial.coverage}`);
  const reachableWorst = Math.max(...stats.reachableLowerJointMax, ...stats.reachableUpperJointMax);
  const allWorst = Math.max(...stats.lowerJointMax, ...stats.upperJointMax);
  assert.ok(allWorst > degToRad(12), 'the violating poses exceed the limit');
  assert.ok(reachableWorst > 0 && reachableWorst < degToRad(12));
  close(partial.limitMargin, 1 - reachableWorst / degToRad(12), 1e-12);
  assert.ok(partial.limitMargin > 0, 'headroom of the reachable poses is reported, not 0');
  const none = await evaluateLayout(pairedFixture(), { ...options, ranges: { z: { min: 500, max: 500, step: 5 } } });
  assert.equal(none.coverage, 0);
  assert.equal(none.limitMargin, 0, 'no reachable pose means no headroom');
});

test('a stiffness model selects physical stiffness by default, false keeps the proxy, and older exports replay with the proxy', async () => {
  const model = { servo_torsional_stiffness_nm_per_rad: 40, rods: 'rigid' };
  const settings = { populationSize: 4, generations: 1, ranges: {}, sampling: { strategy: 'grid' }, objectiveSet: 'full', seed: 9 };
  const physical = new Optimizer({ mass_kg: 1, stiffness_model: model }, settings);
  assert.equal(physical.effectiveSettings().objectiveDefinitions[5].key, 'physicalStiffness');
  assert.equal(physical.effectiveSettings().stiffnessModel.useAsObjective, true);
  const proxy = new Optimizer({ mass_kg: 1, stiffness_model: { ...model, use_as_objective: false } }, settings);
  assert.equal(proxy.effectiveSettings().objectiveDefinitions[5].key, 'stiffness');
  assert.equal(new Optimizer({ mass_kg: 1 }, settings).effectiveSettings().objectiveDefinitions[5].key, 'stiffness');
  const data = JSON.parse(sampleText);
  data.constraints.stiffness_model = model;
  assert.equal(parseRequirements(JSON.stringify(data)).normalized.stiffness_model.use_as_objective, true, 'the resolved choice is recorded');
  data.constraints.stiffness_model = { ...model, use_as_objective: false };
  assert.equal(parseRequirements(JSON.stringify(data)).normalized.stiffness_model.use_as_objective, false);
  await physical.run();
  await proxy.run();
  assert.notDeepEqual(physical.fitness.map(ev => ev.objectives[5]), proxy.fitness.map(ev => ev.objectives[5]));
  // An export from before the default: no flag in requirements, the proxy in objectiveDefinitions.
  const older = { ...layoutToJSON(proxy.getSelectedCandidate().layout), run: { effective_settings: {
    ...proxy.effectiveSettings(), requirements: { mass_kg: 1, stiffness_model: model } } } };
  const replay = Optimizer.fromReplay(older);
  assert.equal(replay.effectiveSettings().objectiveDefinitions[5].key, 'stiffness');
  await replay.run();
  assert.deepEqual(replay.fitness.map(ev => ev.objectives), proxy.fitness.map(ev => ev.objectives));
  const current = Optimizer.fromReplay({ ...layoutToJSON(physical.getSelectedCandidate().layout),
    run: { effective_settings: physical.effectiveSettings() } });
  assert.equal(current.effectiveSettings().objectiveDefinitions[5].key, 'physicalStiffness');
});
