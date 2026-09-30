import test from 'node:test';
import assert from 'node:assert/strict';
import { evaluateCompliance, normalizeStiffnessModel } from '../../src/model/compliance.js';
import { physicalMotionJacobian, solveLinear } from '../../src/model/cycle.js';
import { evaluatePose } from '../../src/model/pose.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { evaluateLayout } from '../../src/optimization/evaluate-layout.js';
import { objectiveDefinitions } from '../../src/optimization/objectives.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { exportResult, layoutToJSON } from '../../src/io/results.js';
import { asymmetricJointFixture, pairedFixture } from '../fixtures/layout.js';
import { sampleText } from '../ui/helpers.js';

const close = (actual, expected, tolerance, message = '') =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${message} ${actual} vs ${expected}`);
const home = layout => evaluatePose(layout, {}, { ballJointLimitDeg: 180, recordLegData: true });
const wrench = { name: 'mixed', force_n: [12, -5, 30], moment_nm: [0.4, -0.2, 0.3] };
const model = extra => normalizeStiffnessModel({ servo_torsional_stiffness_nm_per_rad: [40, 45, 50, 38, 42, 47],
  test_wrenches: [wrench], ...extra });
const W = [...wrench.force_n, ...wrench.moment_nm];
const twistOf = result => [...result.testWrenches[0].translationMm.map(v => v / 1000),
  ...result.testWrenches[0].rotationDeg.map(v => v * Math.PI / 180)];

// Independent series-spring solution: rod forces from equilibrium, each leg deflects f/k,
// and the platform twist follows from the rigid leg kinematics A dX = s.
function springSolution(layout, servo, rodStiffness) {
  const pose = home(layout);
  const motion = physicalMotionJacobian(layout, pose);
  const forces = solveLinear(Array.from({ length: 6 }, (_, r) => motion.A.map(row => row[r])), W);
  const servoTorque = forces.map((f, i) => f * motion.transmissions[i]);
  const rodStretch = forces.map(f => f / rodStiffness);
  const servoRotation = servoTorque.map((t, i) => t / servo[i]);
  const legTravel = rodStretch.map((s, i) => s + motion.transmissions[i] * servoRotation[i]);
  const twist = solveLinear(motion.A, legTravel);
  const energy = forces.reduce((sum, f) => sum + (Number.isFinite(rodStiffness) ? f * f / (2 * rodStiffness) : 0), 0)
    + servoTorque.reduce((sum, t, i) => sum + t * t / (2 * servo[i]), 0);
  return { twist, energy, motion };
}

test('rigid rods recover K = G^T K_servo G and the analytic spring displacement and energy', () => {
  const layout = asymmetricJointFixture();
  const result = evaluateCompliance(layout, home(layout), model({ rods: 'rigid' }));
  assert.equal(result.status, 'available');
  const servo = [40, 45, 50, 38, 42, 47];
  const { twist, energy, motion } = springSolution(layout, servo, Infinity);
  const expected = Array.from({ length: 6 }, (_, r) => Array.from({ length: 6 }, (_, c) =>
    motion.rows.reduce((sum, row, i) => sum + row[r] * servo[i] * row[c], 0)));
  const scale = Math.max(...expected.flat().map(Math.abs));
  result.stiffnessMatrix.forEach((row, r) => row.forEach((value, c) => close(value, expected[r][c], 1e-10 * scale)));
  twistOf(result).forEach((value, i) => close(value, twist[i], 1e-9 * Math.max(...twist.map(Math.abs))));
  close(result.testWrenches[0].energyJ, energy, 1e-9 * energy);
});

test('rod and servo springs combine in series without double counting', () => {
  const layout = pairedFixture();
  const servo = [40, 45, 50, 38, 42, 47];
  const combined = evaluateCompliance(layout, home(layout), model({ rod_axial_stiffness_n_per_m: 2e5 }));
  const { twist, energy } = springSolution(layout, servo, 2e5);
  twistOf(combined).forEach((value, i) => close(value, twist[i], 1e-9 * Math.max(...twist.map(Math.abs))));
  close(combined.testWrenches[0].energyJ, energy, 1e-9 * energy);
  combined.legStiffnessNPerM.forEach((k, i) => {
    assert.ok(k < 2e5, 'a series leg is softer than its rod');
    assert.ok(combined.servoComplianceFraction[i] > 0 && combined.servoComplianceFraction[i] < 1);
  });
  // A near-rigid servo leaves only rod compliance.
  const rodOnly = evaluateCompliance(layout, home(layout), normalizeStiffnessModel({ servo_torsional_stiffness_nm_per_rad: 1e12,
    rod_axial_stiffness_n_per_m: 2e5 }));
  rodOnly.legStiffnessNPerM.forEach(k => close(k, 2e5, 1e-3));
});

test('stiffness is symmetric and positive; compliant directions are explicit', () => {
  const layout = asymmetricJointFixture();
  const result = evaluateCompliance(layout, home(layout), model({ rod_axial_stiffness_n_per_m: 5e5 }));
  const K = result.stiffnessMatrix;
  const scale = Math.max(...K.flat().map(Math.abs));
  K.forEach((row, r) => row.forEach((value, c) => close(value, K[c][r], 1e-12 * scale)));
  assert.ok(result.scaledEigenvaluesNPerM.every(value => value > 0));
  const identity = K.map((row, r) => result.complianceMatrix.map((_, c) =>
    row.reduce((sum, value, k) => sum + value * result.complianceMatrix[k][c], 0)));
  identity.forEach((row, r) => row.forEach((value, c) => close(value, r === c ? 1 : 0, 1e-8)));
  const soft = evaluateCompliance(layout, home(layout), normalizeStiffnessModel({
    servo_torsional_stiffness_nm_per_rad: [1e-12, 40, 40, 40, 40, 40], rods: 'rigid', test_wrenches: [wrench] }));
  assert.equal(soft.status, 'singular');
  assert.equal(soft.complianceMatrix, null);
  assert.deepEqual(soft.testWrenches, []);
});

test('spring scaling and unit choices behave physically', () => {
  const layout = pairedFixture();
  const base = evaluateCompliance(layout, home(layout), model({ rod_axial_stiffness_n_per_m: 3e5 }));
  const doubled = evaluateCompliance(layout, home(layout), normalizeStiffnessModel({
    servo_torsional_stiffness_nm_per_rad: [80, 90, 100, 76, 84, 94], rod_axial_stiffness_n_per_m: 6e5, test_wrenches: [wrench] }));
  twistOf(doubled).forEach((value, i) => close(value, twistOf(base)[i] / 2, 1e-12));
  // E A / L from diameter or area equals the directly supplied axial stiffness (L = 200 mm).
  const k = 200e9 * Math.PI * 0.002 ** 2 / 0.2;
  const direct = evaluateCompliance(layout, home(layout), model({ rod_axial_stiffness_n_per_m: k }));
  for (const material of [{ youngs_modulus_gpa: 200, diameter_mm: 4 }, { youngs_modulus_gpa: 200, area_mm2: Math.PI * 4 }]) {
    const derived = evaluateCompliance(layout, home(layout), model({ rod_material: material }));
    twistOf(derived).forEach((value, i) => close(value, twistOf(direct)[i], 1e-12));
  }
  // The characteristic length changes only the scalar score, never the physical predictions.
  const shortL = evaluateCompliance(layout, home(layout), model({ rod_axial_stiffness_n_per_m: 3e5, characteristic_length_mm: 20 }));
  const longL = evaluateCompliance(layout, home(layout), model({ rod_axial_stiffness_n_per_m: 3e5, characteristic_length_mm: 200 }));
  assert.deepEqual(shortL.testWrenches, longL.testWrenches);
  assert.notEqual(shortL.minScaledStiffnessNPerM, longL.minScaledStiffnessNPerM);
  assert.equal(base.characteristicLengthSource, 'platform RMS anchor radius');
  close(base.characteristicLengthM, 0.05, 1e-12);
});

test('material inputs change physical predictions with geometry and conditioning unchanged', async () => {
  const options = stiffness => ({ ranges: {}, sampling: { strategy: 'grid' }, payload: 1, stroke: 0, frequency: 0,
    cycleAxis: 'z', ballJointLimitDeg: 180, servoRangeRad: [-Math.PI, Math.PI], stiffnessModel: stiffness });
  const steel = await evaluateLayout(pairedFixture(), options(model({ rod_material: { youngs_modulus_gpa: 200, diameter_mm: 4 } })));
  const nylon = await evaluateLayout(pairedFixture(), options(model({ rod_material: { youngs_modulus_gpa: 2.5, diameter_mm: 4 } })));
  assert.equal(steel.conditioningQuality, nylon.conditioningQuality);
  assert.equal(steel.stiffness, nylon.stiffness, 'the geometric proxy cannot see material');
  assert.ok(steel.physicalStiffness > nylon.physicalStiffness);
  assert.ok(Math.abs(nylon.compliance.testWrenches[0].translationMm[2]) > Math.abs(steel.compliance.testWrenches[0].translationMm[2]));
});

test('missing parameters stay unavailable in ranking and export; validation is explicit', async () => {
  const evaluation = await evaluateLayout(pairedFixture(), { ranges: {}, sampling: { strategy: 'grid' }, payload: 1,
    stroke: 0, frequency: 0, cycleAxis: 'z', ballJointLimitDeg: 180, servoRangeRad: [-Math.PI, Math.PI],
    objectiveSet: 'full', objectiveVariant: { stiffnessMetric: 'physicalStiffness' } });
  assert.equal(evaluation.physicalStiffness, null);
  assert.equal(evaluation.compliance.status, 'unavailable');
  assert.equal(evaluation.objectives[5], -Infinity);
  const exported = exportResult(evaluation, {});
  assert.equal(exported.metadata.physical_stiffness, null);
  assert.equal(exported.physical_stiffness.status, 'unavailable');
  assert.ok(Number.isFinite(exported.metadata.stiffness), 'the proxy remains available');
  for (const [input, pattern] of [[{ rods: 'rigid' }, /servo_torsional/],
    [{ servo_torsional_stiffness_nm_per_rad: 10 }, /exactly one/],
    [{ servo_torsional_stiffness_nm_per_rad: 10, rods: 'rigid', rod_axial_stiffness_n_per_m: 5 }, /exactly one/],
    [{ servo_torsional_stiffness_nm_per_rad: [1, 2], rods: 'rigid' }, /six/],
    [{ servo_torsional_stiffness_nm_per_rad: 10, rod_material: { youngs_modulus_gpa: 200 } }, /area_mm2 or diameter_mm/],
    [{ servo_torsional_stiffness_nm_per_rad: 10, rod_material: { youngs_modulus_gpa: 200, area_mm2: null, diameter_mm: 5 } },
      /area_mm2 or diameter_mm/],
    [{ servo_torsional_stiffness_nm_per_rad: 10, rod_material: { youngs_modulus_gpa: 200, area_mm2: null } }, /area_mm2 must be/],
    [{ servo_torsional_stiffness_nm_per_rad: 10, rod_material: { youngs_modulus_gpa: 200, diameter_mm: null } }, /diameter_mm must be/],
    [{ servo_torsional_stiffness_nm_per_rad: -1, rods: 'rigid' }, /positive/]]) {
    assert.throws(() => normalizeStiffnessModel(input), pattern);
  }
  const data = JSON.parse(sampleText);
  data.constraints.stiffness_model = { servo_torsional_stiffness_nm_per_rad: 10 };
  assert.throws(() => parseRequirements(JSON.stringify(data)), /exactly one/);
});

test('the proxy remains the default objective; physical stiffness is opt-in and replayable', async () => {
  assert.equal(objectiveDefinitions('full')[5].key, 'stiffness');
  assert.equal(objectiveDefinitions('full', { stiffnessMetric: 'physicalStiffness' })[5].key, 'physicalStiffness');
  const requirements = { mass_kg: 1, cycle_mm: 0, frequency_hz: 0, cycle_axis: 'z',
    stiffness_model: { servo_torsional_stiffness_nm_per_rad: 40, rods: 'rigid', use_as_objective: true,
      characteristic_length_mm: 50, test_wrenches: [wrench] } };
  const optimizer = new Optimizer(requirements, { populationSize: 4, generations: 1, ranges: {}, sampling: { strategy: 'grid' },
    objectiveSet: 'full' });
  const settings = optimizer.effectiveSettings();
  assert.equal(settings.objectiveDefinitions[5].key, 'physicalStiffness');
  assert.equal(settings.stiffnessModel.characteristicLengthM, 0.05);
  await optimizer.run();
  const replay = Optimizer.fromReplay({ ...layoutToJSON(optimizer.getSelectedCandidate().layout), run: { effective_settings: settings } });
  await replay.run();
  assert.deepEqual(replay.fitness.map(ev => ev.physicalStiffness), optimizer.fitness.map(ev => ev.physicalStiffness));
});
