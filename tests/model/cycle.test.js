import test from 'node:test';
import assert from 'node:assert/strict';
import { computeCycleDemand, evaluateCyclePose } from '../../cycle.js';
import { evaluatePose } from '../../workspace.js';
import { evaluateDynamicPose } from '../../src/model/cycle.js';
import { legacyTrajectory, normalizeTrajectory, trajectoryState } from '../../src/model/trajectory.js';
import { normalizeMassProperties } from '../../src/model/mass-properties.js';
import { jointFixture } from '../fixtures/layout.js';

const argmax = values => values.reduce((best, value, i) => (value > values[best] ? i : best), 0);
const close = (actual, expected, tolerance = 1e-12, message = '') =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${message} ${actual} vs ${expected}`);

function asymmetricLayout() {
  const layout = jointFixture();
  layout.platformAnchors[0][0] += 8;
  layout.platformAnchors[2][1] -= 12;
  layout.baseAnchors[3][0] -= 14;
  layout.betaAngles[1] += 0.2;
  layout.betaAngles[4] -= 0.3;
  return layout;
}

test('cycle axis changes geometric speed and load demand on asymmetric layout', () => {
  const results = ['x', 'y', 'z'].map(axis => computeCycleDemand(asymmetricLayout(), {
    mass: 2.5, stroke: 10, frequency: 2, axis, ballJointLimitDeg: 180
  }));
  assert.ok(results.every(result => result.valid), JSON.stringify(results));
  for (let i = 0; i < 2; i++) {
    assert.ok(Math.abs(results[i].torqueNm - results[i + 1].torqueNm) > 1e-3);
    assert.ok(Math.abs(results[i].speedRadPerSec - results[i + 1].speedRadPerSec) > 1e-3);
  }
});

test('rod forces balance the applied wrench with gravity remaining vertical', () => {
  const result = evaluateCyclePose(asymmetricLayout(), {}, [0.1, 0, 0], [2, 0, 0], 3, { ballJointLimitDeg: 180 });
  assert.equal(result.valid, true);
  assert.deepEqual(result.requiredForce, [6, 0, 29.43]);
  const actual = result.equilibrium.map(row => row.reduce((sum, value, i) => sum + value * result.rodForces[i], 0));
  const expected = [...result.requiredForce, 0, 0, 0];
  actual.forEach((value, i) => assert.ok(Math.abs(value - expected[i]) < 1e-8));
});

test('a zero-frequency cycle evaluates one stationary home sample whose gravity torque is axis-independent', () => {
  // The stroke and axis are irrelevant at 0 Hz: every axis collapses to the same
  // single home-pose evaluation, so the demand must equal the static balance there.
  const home = evaluateCyclePose(asymmetricLayout(), {}, [0, 0, 0], [0, 0, 0], 2, { ballJointLimitDeg: 180 });
  assert.equal(home.valid, true);
  const staticTorque = Math.max(...home.torque);
  assert.ok(staticTorque > 0.1, 'gravity loads the servos at home');
  const results = ['x', 'y', 'z'].map(axis => computeCycleDemand(asymmetricLayout(), {
    mass: 2, stroke: 10, frequency: 0, axis, ballJointLimitDeg: 180
  }));
  for (const result of results) {
    assert.equal(result.valid, true);
    assert.equal(result.samples, 1);
    assert.equal(result.sampling.status, 'stationary');
    assert.equal(result.periodS, 0);
    assert.equal(result.speedRadPerSec, 0);
    assert.equal(result.accelerationRadPerSec2, 0);
    assert.equal(result.torqueNm, staticTorque);
    assert.deepEqual(result.perServoTorqueNm, home.torque);
    assert.deepEqual(result.servoExcursionRmsRad, new Array(6).fill(0));
  }
  // A moving cycle is not axis-independent: the same stroke along z loads the servos differently.
  const moving = computeCycleDemand(asymmetricLayout(), { mass: 2, stroke: 10, frequency: 2, axis: 'z', ballJointLimitDeg: 180 });
  assert.ok(moving.samples > 1);
  assert.notEqual(moving.torqueNm, staticTorque);
});

test('limiting entries name the 1-based servo with the largest absolute peak and its sorted sample', () => {
  const period = 0.5;
  for (const axis of ['x', 'y', 'z']) {
    const input = { mass: 2.5, stroke: 10, frequency: 2, axis, ballJointLimitDeg: 180 };
    const result = computeCycleDemand(asymmetricLayout(), input);
    assert.equal(result.valid, true);
    for (const [key, peaks] of [['torque', result.perServoTorqueNm], ['speed', result.perServoSpeedRadPerSec],
      ['acceleration', result.perServoAccelerationRadPerSec2]]) {
      const entry = result.limiting[key];
      const leg = argmax(peaks);
      assert.ok(leg > 0, `${axis} ${key} peak is not on servo 1, so 0-based and 1-based numbering differ`);
      assert.equal(entry.servo, leg + 1, `${axis} ${key} servo`);
      assert.equal(Math.abs(entry.value), peaks[leg], `${axis} ${key} value is the per-servo peak`);
      assert.equal(Math.abs(entry.value), Math.max(...peaks), `${axis} ${key} value is the overall peak`);
      // The sample index refers to the time-sorted uniform grid, and the signed value
      // is the servo's demand recomputed independently at that time.
      assert.ok(Number.isInteger(entry.sample) && entry.sample >= 0 && entry.sample < 64);
      assert.equal(entry.time, period * entry.sample / 64);
      const state = trajectoryState(legacyTrajectory(input), entry.time);
      const at = evaluateDynamicPose(asymmetricLayout(), state, normalizeMassProperties({ mass_kg: 2.5 }),
        { ballJointLimitDeg: 180 });
      const signed = { torque: at.signedTorque, speed: at.signedSpeed, acceleration: at.servoAcceleration }[key];
      close(entry.value, signed[leg], 1e-12, `${axis} ${key} signed value`);
    }
  }
  const x = computeCycleDemand(asymmetricLayout(), { mass: 2.5, stroke: 10, frequency: 2, axis: 'x', ballJointLimitDeg: 180 });
  assert.equal(x.limiting.torque.servo, 5);
  assert.equal(x.limiting.speed.servo, 3);
});

test('a cycle that fails mid-period reports the 0-based index and time of the first invalid sample', () => {
  const layout = asymmetricLayout();
  const input = { stroke: 30, frequency: 1, axis: 'x' };
  const options = { ballJointLimitDeg: 3 };
  const samples = 16;
  // Independent scan of the uniform grid with the shared pose evaluator.
  const trajectory = legacyTrajectory(input);
  const reachable = Array.from({ length: samples }, (_, i) =>
    evaluatePose(layout, trajectoryState(trajectory, i / samples).pose, options).reachable);
  const firstFailure = reachable.indexOf(false);
  assert.ok(firstFailure > 1, `the home sample and its neighbour pass: ${reachable}`);
  const result = computeCycleDemand(layout, { mass: 1, ...input, ...options,
    sampling: { strategy: 'uniform', samples } });
  assert.equal(result.valid, false);
  assert.equal(result.sampling.status, 'violated');
  assert.equal(result.failedSample, firstFailure);
  assert.equal(result.samples, firstFailure + 1, 'evaluation stops at the failed sample');
  assert.equal(result.sampling.evaluatedSamples, firstFailure + 1);
  assert.equal(result.failedTime, firstFailure / samples);
  assert.deepEqual(result.failedPose, trajectoryState(trajectory, firstFailure / samples).pose);
  assert.ok(result.violations.some(v => v.type === 'ballJoint'));
  assert.equal(result.torqueNm, null);
  assert.equal(firstFailure, 2);
});

test('adaptive refinement inspects every initial midpoint, then bisects the largest normalized estimate', () => {
  // Independent computation of the documented rule (CYCLE_MODEL.md, Sampling and
  // convergence): the first 2N samples are the uniform 2N grid; each initial interval's
  // unresolved variation is a quarter of the midpoint's departure from linear
  // interpolation, normalized per quantity; the largest estimate is bisected next,
  // ties to the earliest interval.
  const layout = asymmetricLayout();
  const trajectory = normalizeTrajectory({ frequency_hz: 2, components: [
    { axis: 'x', amplitude_mm: 5, phase_deg: 20 }, { axis: 'rz', amplitude_deg: 4, phase_deg: 110 }] });
  const massProperties = normalizeMassProperties({ mass_kg: 2.5 });
  const options = { ballJointLimitDeg: 180 };
  const period = 0.5, initial = 8, grid = 2 * initial;
  const values = [], limits = [];
  for (let i = 0; i < grid; i++) {
    const at = evaluateDynamicPose(layout, trajectoryState(trajectory, period * i / grid), massProperties, options);
    assert.equal(at.valid, true);
    values.push([...at.signedTorque, ...at.signedSpeed, at.conditioning.reciprocal,
      ...at.jointAngles.lower, ...at.jointAngles.upper, ...at.servoAngles]);
    limits.push(at.jointLimits);
  }
  const peak = from => Math.max(...values.flatMap(row => row.slice(from, from + 6).map(Math.abs)));
  const servoSpan = layout.servoRangeRad[1] - layout.servoRangeRad[0];
  const scales = [...new Array(6).fill(peak(0)), ...new Array(6).fill(peak(6)),
    Math.max(Math.min(...values.map(row => row[12])), 1e-3),
    ...new Array(6).fill(limits[0].lower), ...new Array(6).fill(limits[0].upper), ...new Array(6).fill(servoSpan)];
  const estimates = Array.from({ length: initial }, (_, i) => {
    const a = values[2 * i], mid = values[2 * i + 1], b = values[(2 * i + 2) % grid];
    return mid.reduce((worst, value, q) =>
      Math.max(worst, Math.abs(value - (a[q] + b[q]) / 2) / 4 / scales[q]), 0);
  });
  const worst = argmax(estimates);
  const sorted = [...estimates].sort((p, q) => q - p);
  assert.ok(sorted[0] - sorted[1] > 1e-3, `the worst interval is unambiguous: ${estimates}`);
  const uniformTimes = Array.from({ length: grid }, (_, i) => period * i / grid);
  const run = maxSamples => computeCycleDemand(layout, { trajectory, massProperties, ...options,
    sampling: { strategy: 'adaptive', initialSamples: initial, maxSamples, tolerance: 1e-9 } });

  // Budget exhausted right after the mandatory midpoint pass: exactly the uniform grid.
  const grid16 = run(grid);
  assert.equal(grid16.valid, true);
  assert.equal(grid16.sampling.status, 'budget-limited');
  assert.equal(grid16.sampling.evaluatedSamples, grid);
  assert.equal(grid16.samples, grid);
  assert.deepEqual(grid16.history.times, uniformTimes);
  close(grid16.sampling.maxUnresolved, estimates[worst], 1e-15, 'maxUnresolved');

  // One more sample bisects the earlier half of the worst initial interval.
  const grid17 = run(grid + 1);
  assert.equal(grid17.sampling.evaluatedSamples, grid + 1);
  const extra = grid17.history.times.filter(time => !uniformTimes.includes(time));
  assert.deepEqual(extra, [period * worst / initial + period / (4 * initial)]);
  assert.equal(worst, 3);

  // The tolerance is compared against the same normalized estimate.
  const converge = tolerance => computeCycleDemand(layout, { trajectory, massProperties, ...options,
    sampling: { strategy: 'adaptive', initialSamples: initial, maxSamples: 64, tolerance } }).sampling;
  const above = converge(estimates[worst] * (1 + 1e-9));
  assert.equal(above.status, 'converged');
  assert.equal(above.evaluatedSamples, grid, 'no refinement beyond the midpoint pass');
  // "At most tolerance" is inclusive: the reported maxUnresolved itself converges.
  const exact = converge(grid16.sampling.maxUnresolved);
  assert.equal(exact.status, 'converged');
  assert.equal(exact.evaluatedSamples, grid);
  const below = converge(estimates[worst] * (1 - 1e-9));
  assert.equal(below.status, 'converged');
  assert.ok(below.evaluatedSamples > grid, 'a tighter tolerance forces at least one bisection');
  assert.ok(below.maxUnresolved <= estimates[worst] * (1 - 1e-9));
});

test('invalid cycle poses are explicitly unavailable, not zero-demand results', () => {
  const result = computeCycleDemand(asymmetricLayout(), { mass: 2, stroke: 10, frequency: 2, ballJointLimitDeg: 0 });
  assert.equal(result.valid, false);
  assert.equal(result.torqueNm, null);
  assert.match(result.reason, /constraint/);
});

test('servo speeds agree with finite differences of the inverse kinematics', () => {
  const layout = asymmetricLayout();
  const velocity = [0, 0.1, 0];
  const dt = 1e-5;
  const options = { ballJointLimitDeg: 180 };
  const demand = evaluateCyclePose(layout, {}, velocity, [0, 0, 0], 2, options);
  const before = evaluatePose(layout, { y: -velocity[1] * dt * 1000 }, options);
  const after = evaluatePose(layout, { y: velocity[1] * dt * 1000 }, options);
  assert.ok(before.reachable && after.reachable && demand.valid);
  demand.speed.forEach((speed, i) => {
    const difference = Math.abs((after.servoAngles[i] - before.servoAngles[i]) / (2 * dt));
    assert.ok(Math.abs(speed - difference) < 1e-6);
  });
});
