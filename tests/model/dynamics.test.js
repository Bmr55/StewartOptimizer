import test from 'node:test';
import assert from 'node:assert/strict';
import { computeCycleDemand, evaluateDynamicPose, requiredWrench } from '../../src/model/cycle.js';
import { evaluatePose } from '../../src/model/pose.js';
import { normalizeMassProperties } from '../../src/model/mass-properties.js';
import { legacyTrajectory, normalizeTrajectory, trajectoryState } from '../../src/model/trajectory.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { layoutToJSON } from '../../src/io/results.js';
import { rotationMatrixFromEuler, rotateVector, vectorNormalize, vectorCross, vectorDot,
  vectorScale } from '../../src/math.js';
import { asymmetricJointFixture, pairedFixture } from '../fixtures/layout.js';
import { sampleText } from '../ui/helpers.js';

const options = { ballJointLimitDeg: 180 };
const close = (actual, expected, tolerance, message) =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${message ?? ''} ${actual} vs ${expected}`);
const combined = normalizeTrajectory({ frequency_hz: 1.5, components: [
  { axis: 'x', amplitude_mm: 6, phase_deg: 20 }, { axis: 'z', amplitude_mm: 5, phase_deg: 70 },
  { axis: 'rx', amplitude_deg: 4, phase_deg: 110 }, { axis: 'ry', amplitude_deg: 3, phase_deg: 35 },
  { axis: 'rz', amplitude_deg: 5, phase_deg: 200 },
] });
const rigid = normalizeMassProperties({ mass_kg: 2.2, center_of_mass_mm: [12, -7, 25],
  inertia_kg_m2: { ixx: 0.012, iyy: 0.015, izz: 0.02, ixy: 0.001, ixz: -0.0005, iyz: 0.0007 } });
const residual = result => result.equilibrium.map((row, i) =>
  row.reduce((sum, value, k) => sum + value * result.rodForces[k], 0)
  - [...result.requiredForce, ...result.requiredMoment][i]);

// Reimplementation of the reviewed single-axis model, used as the legacy reference.
function legacyReference(layout, { mass, stroke, frequency, axis }) {
  const axisIndex = ['x', 'y', 'z'].indexOf(axis);
  const amplitude = stroke / 2000, omega = 2 * Math.PI * frequency;
  const torque = new Array(6).fill(0), speed = new Array(6).fill(0);
  for (let i = 0; i < 64; i++) {
    const phase = 2 * Math.PI * i / 64;
    const displacement = amplitude * Math.sin(phase);
    const result = evaluatePose(layout, { [axis]: displacement * 1000 }, { ...options, recordLegData: true });
    const u = result.rodVectors.map(vectorNormalize);
    const columns = u.map((d, leg) => [...d, ...vectorCross(vectorScale(rotateVector(result.rotationMatrix,
      layout.platformAnchors[leg]), 0.001), d)]);
    const velocity = [0, 0, 0], acceleration = [0, 0, 0];
    velocity[axisIndex] = omega * amplitude * Math.cos(phase);
    acceleration[axisIndex] = -omega * omega * displacement;
    const rhs = [...acceleration.map((a, k) => mass * (a + (k === 2 ? 9.81 : 0))), 0, 0, 0];
    const a = Array.from({ length: 6 }, (_, row) => [...columns.map(col => col[row]), rhs[row]]);
    for (let col = 0; col < 6; col++) {
      let pivot = col;
      for (let row = col + 1; row < 6; row++) if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
      [a[col], a[pivot]] = [a[pivot], a[col]];
      const scale = a[col][col];
      for (let j = col; j <= 6; j++) a[col][j] /= scale;
      for (let row = 0; row < 6; row++) {
        if (row === col) continue;
        const factor = a[row][col];
        for (let j = col; j <= 6; j++) a[row][j] -= factor * a[col][j];
      }
    }
    for (let leg = 0; leg < 6; leg++) {
      const alpha = result.servoAngles[leg], beta = layout.betaAngles[leg], h = layout.hornLength / 1000;
      const tangent = [-h * Math.sin(alpha) * Math.cos(beta), -h * Math.sin(alpha) * Math.sin(beta), h * Math.cos(alpha)];
      const transmission = vectorDot(u[leg], tangent);
      torque[leg] = Math.max(torque[leg], Math.abs(a[leg][6] * transmission));
      speed[leg] = Math.max(speed[leg], Math.abs(vectorDot(u[leg], velocity) / transmission));
    }
  }
  return { torque, speed };
}

test('legacy centered translational cycles reproduce the reviewed single-axis results', () => {
  for (const axis of ['x', 'y', 'z']) {
    const input = { mass: 2.5, stroke: 10, frequency: 2, axis };
    const reference = legacyReference(asymmetricJointFixture(), input);
    const current = computeCycleDemand(asymmetricJointFixture(), { ...input, ...options });
    assert.equal(current.valid, true);
    assert.equal(current.massModel, 'legacy-point');
    assert.equal(current.trajectorySource, 'legacy-cycle');
    assert.equal(current.axis, axis);
    current.perServoTorqueNm.forEach((value, i) => close(value, reference.torque[i], 1e-9));
    current.perServoSpeedRadPerSec.forEach((value, i) => close(value, reference.speed[i], 1e-12));
    // The same motion as an explicit trajectory with a zero-offset rigid body is identical.
    const explicit = computeCycleDemand(asymmetricJointFixture(), { ...options,
      trajectory: legacyTrajectory(input), massProperties: normalizeMassProperties({ mass_kg: 2.5,
        center_of_mass_mm: [0, 0, 0] }) });
    assert.equal(explicit.massModel, 'rigid-body');
    assert.deepEqual(explicit.perServoTorqueNm, current.perServoTorqueNm);
  }
});

test('static offset center of mass produces the gravity moment and unequal rod loads', () => {
  const state = trajectoryState(legacyTrajectory(), 0);
  const centered = evaluateDynamicPose(pairedFixture(), state, normalizeMassProperties({ mass_kg: 2 }), options);
  const spread = Math.max(...centered.rodForces) - Math.min(...centered.rodForces);
  assert.ok(spread < 1e-9, 'symmetric centered load is shared equally');
  const offset = evaluateDynamicPose(pairedFixture(), state,
    normalizeMassProperties({ mass_kg: 2, center_of_mass_mm: [30, 0, 40] }), options);
  assert.deepEqual(offset.requiredForce, [0, 0, 2 * 9.81]);
  close(offset.requiredMoment[0], 0, 1e-12);
  close(offset.requiredMoment[1], -0.03 * 2 * 9.81, 1e-12, 'M = c x F');
  close(offset.requiredMoment[2], 0, 1e-12);
  assert.ok(Math.max(...offset.rodForces) - Math.min(...offset.rodForces) > 0.5);
  residual(offset).forEach(value => close(value, 0, 1e-9));
  // Tilting the platform moves the offset center of mass and changes the gravity moment.
  const tilted = evaluateDynamicPose(pairedFixture(), { ...state, pose: { ry: 0.2 } },
    normalizeMassProperties({ mass_kg: 2, center_of_mass_mm: [30, 0, 40] }), options);
  const c = rotateVector(rotationMatrixFromEuler(0, 0.2, 0), [0.03, 0, 0.04]);
  close(tilted.requiredMoment[1], -c[0] * 2 * 9.81, 1e-12);
});

test('pure rotation reproduces I alpha + omega x I omega about the center of mass', () => {
  const trajectory = normalizeTrajectory({ frequency_hz: 2, components: [{ axis: 'rz', amplitude_deg: 8 }] });
  const mass = normalizeMassProperties({ mass_kg: 1.5, inertia_kg_m2: [[0.01, 0, 0], [0, 0.012, 0], [0, 0, 0.018]] });
  const state = trajectoryState(trajectory, 0.1);
  const result = evaluateDynamicPose(pairedFixture(), state, mass, options);
  assert.equal(result.valid, true);
  close(result.requiredMoment[2], 0.018 * state.angularAcceleration[2], 1e-12);
  close(result.requiredMoment[0], 0, 1e-12);
  assert.deepEqual(result.requiredForce, [0, 0, 1.5 * 9.81]);
  residual(result).forEach(value => close(value, 0, 1e-9));
});

test('combined motion matches finite-difference Newton-Euler balances and angular velocity', () => {
  const dt = 1e-5, t = 0.23;
  const world = time => {
    const s = trajectoryState(combined, time);
    const R = rotationMatrixFromEuler(s.pose.rx, s.pose.ry, s.pose.rz);
    const p = [s.pose.x / 1000, s.pose.y / 1000, s.pose.z / 1000];
    const c = rotateVector(R, rigid.centerOfMassM).map((v, i) => v + p[i]);
    const Iw = R.map(row => [0, 1, 2].map(j => row.reduce((sum, v, k) =>
      sum + v * rotateVector(rigid.inertiaKgM2, R[j])[k], 0)));
    return { s, R, p, c, H: rotateVector(Iw, s.omega) };
  };
  const [before, now, after] = [t - dt, t, t + dt].map(world);
  // Physical angular velocity from dR/dt R^T, not Euler-angle rates.
  const Rdot = now.R.map((row, i) => row.map((_, j) => (after.R[i][j] - before.R[i][j]) / (2 * dt)));
  const W = Rdot.map((row, i) => [0, 1, 2].map(j => row.reduce((sum, v, k) => sum + v * now.R[j][k], 0)));
  close(W[2][1], now.s.omega[0], 1e-8);
  close(W[0][2], now.s.omega[1], 1e-8);
  close(W[1][0], now.s.omega[2], 1e-8);
  const alphaFd = [0, 1, 2].map(i => (after.s.omega[i] - before.s.omega[i]) / (2 * dt));
  alphaFd.forEach((value, i) => close(value, now.s.angularAcceleration[i], 1e-6));
  const centerAcceleration = [0, 1, 2].map(i => (after.c[i] - 2 * now.c[i] + before.c[i]) / (dt * dt));
  const force = centerAcceleration.map((a, i) => rigid.massKg * (a + (i === 2 ? 9.81 : 0)));
  const momentCom = [0, 1, 2].map(i => (after.H[i] - before.H[i]) / (2 * dt));
  const offset = now.c.map((value, i) => value - now.p[i]);
  const moment = vectorCross(offset, force).map((value, i) => value + momentCom[i]);
  const wrench = requiredWrench(now.s, rigid, now.R);
  wrench.force.forEach((value, i) => close(value, force[i], 1e-4));
  wrench.moment.forEach((value, i) => close(value, moment[i], 1e-6));
  const result = evaluateDynamicPose(asymmetricJointFixture(), now.s, rigid, options);
  assert.equal(result.valid, true);
  residual(result).forEach(value => close(value, 0, 1e-9));
});

test('servo rates and accelerations match finite differences at nonzero orientation', () => {
  const layout = asymmetricJointFixture();
  const dt = 1e-5;
  for (const t of [0.05, 0.31, 0.52]) {
    const state = trajectoryState(combined, t);
    assert.ok(Math.abs(state.pose.rx) + Math.abs(state.pose.ry) + Math.abs(state.pose.rz) > 0.03);
    const [before, now, after] = [t - dt, t, t + dt].map(time =>
      evaluatePose(layout, trajectoryState(combined, time).pose, options).servoAngles);
    const result = evaluateDynamicPose(layout, state, rigid, options);
    assert.equal(result.valid, true);
    for (let i = 0; i < 6; i++) {
      close(result.signedSpeed[i], (after[i] - before[i]) / (2 * dt), 1e-7, `rate ${i}`);
      close(result.servoAcceleration[i], (after[i] - 2 * now[i] + before[i]) / (dt * dt), 2e-3, `accel ${i}`);
    }
    // Signed virtual power: servo power equals the wrench power on the moving origin twist.
    const servoPower = result.signedTorque.reduce((sum, value, i) => sum + value * result.signedSpeed[i], 0);
    const bodyPower = vectorDot(result.requiredForce, state.velocity) + vectorDot(result.requiredMoment, state.omega);
    close(servoPower, bodyPower, 1e-9 * Math.max(1, Math.abs(bodyPower)));
  }
});

test('mass properties are validated and legacy point mode is identified', () => {
  assert.equal(normalizeMassProperties({ mass_kg: 1 }).mode, 'legacy-point');
  assert.equal(normalizeMassProperties({ mass_kg: 1, external_force_n: [0, 0, 1] }).mode, 'rigid-body');
  for (const [input, pattern] of [
    [{ mass_kg: 1, inertia_kg_m2: [[1, 0, 0], [0, 1, 0], [0, 0, -1]] }, /semidefinite/],
    [{ mass_kg: 1, inertia_kg_m2: [[1, 0, 0], [0, 1, 0], [0, 0, 3]] }, /triangle/],
    [{ mass_kg: 1, inertia_kg_m2: [[1, 0.2, 0], [0, 1, 0], [0, 0, 1]] }, /symmetric/],
    [{ mass_kg: 0, inertia_kg_m2: { ixx: 1, iyy: 1, izz: 1 } }, /positive mass/],
    [{ mass_kg: 1, center_of_mass_mm: [1, 2] }, /center_of_mass_mm/],
    [{ mass_kg: 1, inertia_kg_m2: { ixx: 1, iyy: 1 } }, /izz/],
  ]) assert.throws(() => normalizeMassProperties(input), pattern);
  assert.throws(() => normalizeTrajectory({ frequency_hz: 1, components: [{ axis: 'rx', amplitude_mm: 2 }] }), /amplitude_deg/);
  assert.throws(() => normalizeTrajectory({ frequency_hz: 1, components: [{ axis: 'x', amplitude_mm: 1 },
    { axis: 'X', amplitude_mm: 2 }] }), /repeated/);
});

test('requirements carry trajectories and rigid-body fields; legacy fields cannot be mixed', () => {
  const data = JSON.parse(sampleText);
  delete data.payload.cycle_mm; delete data.payload.frequency_hz; delete data.payload.cycle_axis;
  data.payload.trajectory = { frequency_hz: 1, components: [{ axis: 'RZ', amplitude_deg: 3 }] };
  data.payload.center_of_mass_mm = [0, 0, 20];
  data.payload.inertia_kg_m2 = { ixx: 0.01, iyy: 0.01, izz: 0.01 };
  const { normalized } = parseRequirements(JSON.stringify(data));
  assert.equal(normalized.trajectory.components[0].axis, 'rz');
  assert.deepEqual(normalized.center_of_mass_mm, [0, 0, 20]);
  assert.throws(() => parseRequirements(JSON.stringify({ ...data, payload: { ...data.payload, cycle_mm: 5 } })),
    /cannot be combined/);
  assert.throws(() => parseRequirements(JSON.stringify({ ...data, payload: { ...data.payload,
    inertia_kg_m2: [[1, 0, 0], [0, 1, 0], [0, 0, 5]] } })), /triangle/);
  assert.throws(() => parseRequirements(JSON.stringify({ ...data, payload: { ...data.payload, center_of_mass_mm: null } })),
    /must not be null/);
});

test('invalid poses stay explicit failures with trajectory identity', () => {
  const trajectory = normalizeTrajectory({ frequency_hz: 1, components: [{ axis: 'ry', amplitude_deg: 5 }] });
  const result = computeCycleDemand(asymmetricJointFixture(), { trajectory, massProperties: rigid, ballJointLimitDeg: 0 });
  assert.equal(result.valid, false);
  assert.equal(result.torqueNm, null);
  assert.equal(result.trajectorySource, 'supplied');
  assert.match(result.trajectoryId, /ry:5deg/);
});

test('optimizer budgets, exports and replays a rotational rigid-body cycle', async () => {
  const requirements = { mass_kg: 1.5, trajectory: { frequency_hz: 1, components: [{ axis: 'rz', amplitude_deg: 4 }] },
    inertia_kg_m2: { ixx: 0.01, iyy: 0.01, izz: 0.02 }, center_of_mass_mm: [0, 0, 15] };
  const settings = { populationSize: 4, generations: 1, ranges: { x: { min: 0, max: 0, step: 1 } },
    sampling: { strategy: 'grid' }, seed: 7 };
  const optimizer = new Optimizer(requirements, settings);
  assert.equal(optimizer.estimateWork().cyclePosesPerLayout, 64, 'rotation-only cycles are not stationary');
  await optimizer.run();
  const exported = JSON.parse(optimizer.exportBest());
  assert.equal(exported.run.effective_settings.cycleModel.massProperties.mode, 'rigid-body');
  assert.match(exported.run.effective_settings.cycleModel.trajectoryId, /rz:4deg/);
  const replay = Optimizer.fromReplay({ ...layoutToJSON(optimizer.getSelectedCandidate().layout),
    run: { effective_settings: exported.run.effective_settings } });
  await replay.run();
  assert.deepEqual(replay.fitness.map(ev => ev.torque), optimizer.fitness.map(ev => ev.torque));
  const cycle = optimizer.fitness.find(ev => ev.cycle.valid)?.cycle;
  if (cycle) assert.equal(cycle.trajectorySource, 'supplied');
});
