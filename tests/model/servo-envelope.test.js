import test from 'node:test';
import assert from 'node:assert/strict';
import { computeCycleDemand } from '../../src/model/cycle.js';
import { actuatorTorque, envelopeCapacity, evaluateServoCapacity, normalizeServoRatings,
  windowRms } from '../../src/model/servo-ratings.js';
import { DEFAULT_CYCLE_SAMPLING } from '../../src/model/cycle-sampling.js';
import { normalizeTrajectory } from '../../src/model/trajectory.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { evaluateLayout } from '../../src/optimization/evaluate-layout.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { layoutToJSON } from '../../src/io/results.js';
import { asymmetricJointFixture, pairedFixture } from '../fixtures/layout.js';
import { sampleText } from '../ui/helpers.js';

const close = (actual, expected, tolerance = 1e-9, message = '') =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${message} ${actual} vs ${expected}`);
const deg = value => value * Math.PI / 180;
// Peak load torque 0.434 N m at rest; 0.263 N m while moving at 403 deg/s.
const cycleInput = { mass: 3, stroke: 40, frequency: 3, axis: 'z', ballJointLimitDeg: 180 };
const curve = { speed_deg_s: [0, 410], torque_nm: [0.45, 0.2] };

test('a cycle within scalar peak ratings fails the simultaneous torque-speed envelope', async () => {
  const cycle = computeCycleDemand(pairedFixture(), cycleInput);
  const scalar = normalizeServoRatings({ servo_torque_rating_nm: 0.45, servo_speed_rating_deg_s: 410 });
  assert.equal(evaluateServoCapacity(cycle, scalar).status, 'below');
  const ratings = normalizeServoRatings({ servo_torque_rating_nm: 0.45, servo_speed_rating_deg_s: 410,
    servo_torque_speed_curve: curve });
  const capacity = evaluateServoCapacity(cycle, ratings);
  assert.equal(capacity.status, 'above');
  assert.equal(capacity.enforcedSatisfied, false);
  const envelope = capacity.perServo[0].envelope;
  assert.equal(envelope.status, 'above');
  // The limiting operating point is a high-speed sample below both scalar maxima.
  assert.ok(Math.abs(envelope.limiting.speedRadPerSec) > deg(300));
  assert.ok(Math.abs(envelope.limiting.torqueNm) < cycle.torqueNm);
  close(envelope.limiting.capacityNm, 0.45 - 0.25 * Math.abs(envelope.limiting.speedRadPerSec) / deg(410), 1e-12);
  close(envelope.worstHeadroomFraction, (envelope.limiting.capacityNm - Math.abs(envelope.limiting.torqueNm)) / 0.45, 1e-12);
  assert.equal(capacity.peak.status, 'above');

  const evaluation = await evaluateLayout(pairedFixture(), { ranges: {}, sampling: { strategy: 'grid' },
    payload: 3, stroke: 40, frequency: 3, cycleAxis: 'z', ballJointLimitDeg: 180,
    servoRangeRad: [-Math.PI, Math.PI], servoRatings: ratings });
  assert.ok(evaluation.feasibility.failedCategories.includes('servo_capacity'));
  assert.equal(JSON.parse(JSON.stringify(evaluation.cycle)).history, undefined, 'histories stay out of exports');
});

test('envelope interpolation, endpoints, domain, quadrants, units and overrides', () => {
  const ratings = normalizeServoRatings({ servo_torque_speed_curve: { speed_deg_s: [0, 90, 180], torque_nm: [2, 1.5, 0] },
    per_servo_ratings: [null, { torque_speed_curve: { speed_deg_s: [30, 60], torque_nm: [1, 1],
      quadrants: 'motoring-braking' } }, { torque_speed_curve: { speed_deg_s: [0, 60], torque_nm: [1, 1],
      quadrants: 'motoring-braking', braking_torque_nm: [3, 3] } }, null, null, null] });
  const shared = ratings.perServo[0].curve;
  close(shared.speedRadPerSec[1], Math.PI / 2, 1e-15, 'deg/s converts to rad/s');
  close(envelopeCapacity(shared, 1, 0).capacity, 2);
  close(envelopeCapacity(shared, 1, deg(45)).capacity, 1.75, 1e-12);
  close(envelopeCapacity(shared, -1, -deg(135)).capacity, 0.75, 1e-12, 'symmetric applies to both directions');
  close(envelopeCapacity(shared, 1, deg(180)).capacity, 0, 1e-12);
  assert.equal(envelopeCapacity(shared, 1, deg(181)).reason, 'outOfDomain');
  const partial = ratings.perServo[1].curve;
  assert.equal(ratings.perServo[1].source.curve, 'override');
  assert.equal(envelopeCapacity(partial, 1, deg(10)).reason, 'outOfDomain', 'below the first supplied speed');
  assert.equal(envelopeCapacity(partial, 1, deg(45)).quadrant, 'motoring');
  assert.equal(envelopeCapacity(partial, -1, deg(45)).reason, 'unmodeledQuadrant');
  assert.equal(envelopeCapacity(ratings.perServo[2].curve, -1, deg(45)).capacity, 3);
  for (const bad of [{ speed_deg_s: [0, 0], torque_nm: [1, 1] }, { speed_deg_s: [0, 10], torque_nm: [1] },
    { speed_deg_s: [0, 10], torque_nm: [1, -1] }, { speed_deg_s: [0, 10], torque_nm: [0, 0] },
    { speed_deg_s: [0, 10], torque_nm: [1, 1], braking_torque_nm: [1, 1] }, { speed_deg_s: [0, 10], torque_nm: [1, 1], quadrants: 'any' }]) {
    assert.throws(() => normalizeServoRatings({ servo_torque_speed_curve: bad }), RangeError);
  }
  // Out-of-domain operating points are never passed by extrapolation.
  const cycle = computeCycleDemand(pairedFixture(), cycleInput);
  const limited = evaluateServoCapacity(cycle, normalizeServoRatings({
    servo_torque_speed_curve: { speed_deg_s: [0, 300], torque_nm: [5, 5] } }));
  assert.equal(limited.status, 'outOfDomain');
  assert.equal(limited.compliant, false);
  assert.equal(limited.worstHeadroomFraction, null);
});

test('RMS torque matches analytic constant and sinusoidal histories with time weighting', () => {
  const n = 32, times = Array.from({ length: n }, (_, k) => k / n);
  close(windowRms(times, times.map(() => 1.3), 1, 0.2), 1.3, 1e-12);
  close(windowRms(times, times.map(t => 2 * Math.sin(2 * Math.PI * t)), 1, 1), Math.SQRT2, 1e-12);
  close(windowRms(times, times.map(t => 2 * Math.sin(2 * Math.PI * t)), 1, 0.5), Math.SQRT2, 1e-12);
  // Cycle RMS for a stationary cycle is its static torque; for a sinusoid the trapezoid is exact.
  const stationary = computeCycleDemand(pairedFixture(), { ...cycleInput, frequency: 0 });
  close(stationary.actuator.rmsTorqueNm, stationary.torqueNm, 1e-12);
  const legacy = computeCycleDemand(pairedFixture(), cycleInput);
  const reference = computeCycleDemand(pairedFixture(), { ...cycleInput, sampling: { strategy: 'uniform', samples: 1024 } });
  close(legacy.actuator.rmsTorqueNm, reference.actuator.rmsTorqueNm, 1e-4 * reference.actuator.rmsTorqueNm);
  // Adaptive samples are nonuniform: weighted RMS agrees with the dense uniform reference,
  // while an unweighted sample mean would not.
  const trajectory = normalizeTrajectory({ frequency_hz: 1, components: [{ axis: 'x', amplitude_mm: 25, phase_deg: 13 },
    { axis: 'rz', amplitude_deg: 14, phase_deg: 71 }, { axis: 'z', amplitude_mm: 20, phase_deg: 200 }] });
  const options = { mass: 2, trajectory, ballJointLimitDeg: 180 };
  const dense = computeCycleDemand(asymmetricJointFixture(), { ...options, sampling: { strategy: 'uniform', samples: 1024 } });
  const adaptive = computeCycleDemand(asymmetricJointFixture(), { ...options,
    sampling: { ...DEFAULT_CYCLE_SAMPLING, initialSamples: 8, tolerance: 0.002 } });
  assert.ok(new Set(adaptive.history.weights.map(w => w.toFixed(9))).size > 1, 'nonuniform schedule');
  adaptive.actuator.perServoRmsTorqueNm.forEach((rms, i) =>
    close(rms, dense.actuator.perServoRmsTorqueNm[i], 5e-3 * dense.actuator.perServoRmsTorqueNm[i], `servo ${i}`));
  const unweighted = Math.sqrt(adaptive.history.signedTorqueNm[0].reduce((sum, v) => sum + v * v, 0) / adaptive.samples);
  assert.ok(Math.abs(unweighted - dense.actuator.perServoRmsTorqueNm[0]) > 5 * Math.abs(adaptive.actuator.perServoRmsTorqueNm[0] - dense.actuator.perServoRmsTorqueNm[0]));
});

test('peak, continuous and duration-limited ratings have separate status and margins', () => {
  const cycle = computeCycleDemand(pairedFixture(), cycleInput);
  const rms = cycle.actuator.rmsTorqueNm;
  const capacity = evaluateServoCapacity(cycle, normalizeServoRatings({ servo_torque_rating_nm: 0.5,
    servo_continuous_torque_rating_nm: 0.9 * rms, servo_duration_ratings: [{ torque_nm: 0.44, duration_s: 0.01 }] }));
  assert.equal(capacity.peak.status, 'below');
  close(capacity.peak.worstHeadroomFraction, (0.5 - cycle.torqueNm) / 0.5, 1e-12);
  assert.equal(capacity.continuous.status, 'above');
  close(capacity.continuous.worstHeadroomFraction, -1 / 9, 1e-9);
  assert.equal(capacity.duration.status, 'below', 'a short window stays near the 0.434 N m peak');
  assert.ok(capacity.perServo[0].duration[0].demand > rms && capacity.perServo[0].duration[0].demand <= cycle.torqueNm);
  assert.equal(capacity.status, 'above');
  assert.match(capacity.rmsNote, /not a calibrated temperature/);
  const longWindow = evaluateServoCapacity(cycle, normalizeServoRatings({ servo_duration_ratings: [{ torque_nm: 1, duration_s: 10 }] }));
  close(longWindow.perServo[0].duration[0].demand, rms, 1e-3 * rms, 'windows spanning whole periods approach RMS');
});

test('reduced actuator inertia and friction match analytic terms; omission keeps the ideal model', () => {
  close(actuatorTorque(null, 1.2, 3, 4), 1.2);
  close(actuatorTorque({ outputInertiaKgM2: 0.01, viscousNmSPerRad: 0.02, coulombNm: 0.05 }, 1, -2, 10), 1 + 0.1 - 0.04 - 0.05, 1e-12);
  const base = { ...cycleInput, mass: 0 };
  const ideal = computeCycleDemand(pairedFixture(), base);
  assert.equal(ideal.actuator.model, 'ideal');
  assert.deepEqual(ideal.actuator.perServoPeakTorqueNm, ideal.perServoTorqueNm);
  const inertia = computeCycleDemand(pairedFixture(), { ...base, actuators: new Array(6).fill({ outputInertiaKgM2: 2e-4, viscousNmSPerRad: 0, coulombNm: 0 }) });
  assert.equal(inertia.actuator.model, 'reduced');
  inertia.actuator.perServoPeakTorqueNm.forEach((value, i) => close(value, 2e-4 * inertia.perServoAccelerationRadPerSec2[i], 1e-12));
  const viscous = computeCycleDemand(pairedFixture(), { ...base, actuators: new Array(6).fill({ outputInertiaKgM2: 0, viscousNmSPerRad: 0.003, coulombNm: 0 }) });
  viscous.actuator.perServoPeakTorqueNm.forEach((value, i) => close(value, 0.003 * viscous.perServoSpeedRadPerSec[i], 1e-12));
  const coulomb = computeCycleDemand(pairedFixture(), { ...base, frequency: 0, actuators: new Array(6).fill({ outputInertiaKgM2: 0, viscousNmSPerRad: 0, coulombNm: 0.1 }) });
  assert.equal(coulomb.actuator.peakTorqueNm, 0, 'Coulomb friction is zero at rest (static friction unmodeled)');
  // Ratings compare the actuator-referred torque.
  const loaded = computeCycleDemand(pairedFixture(), { ...cycleInput, actuators: new Array(6).fill({ outputInertiaKgM2: 1e-3, viscousNmSPerRad: 0, coulombNm: 0 }) });
  const capacity = evaluateServoCapacity(loaded, normalizeServoRatings({ servo_torque_rating_nm: 0.45,
    servo_actuator: { output_inertia_kg_m2: 1e-3 } }));
  close(capacity.perServo[0].torque.demand, loaded.actuator.perServoPeakTorqueNm[0]);
  assert.ok(loaded.actuator.peakTorqueNm > loaded.torqueNm);
  assert.match(capacity.demandReference, /actuator output torque/);
});

test('unavailable demand cannot pass envelope or continuous ratings; advisory stays visible', () => {
  for (const policy of ['enforced', 'advisory']) {
    const ratings = normalizeServoRatings({ servo_torque_speed_curve: curve, servo_continuous_torque_rating_nm: 1, servo_rating_policy: policy });
    for (const input of [null, { valid: false }, computeCycleDemand(pairedFixture(), { ...cycleInput, ballJointLimitDeg: 0 })]) {
      const capacity = evaluateServoCapacity(input, ratings);
      assert.equal(capacity.status, 'unavailable');
      assert.equal(capacity.perServo[0].envelope.status, 'unavailable');
      assert.equal(capacity.perServo[0].continuous.status, 'unavailable');
      assert.equal(capacity.compliant, false);
      assert.equal(capacity.enforcedSatisfied, policy === 'advisory');
    }
  }
});

test('requirements, UI merging and replay preserve curves, continuous ratings and actuators', async () => {
  const data = JSON.parse(sampleText);
  Object.assign(data.constraints, { servo_torque_speed_curve: curve, servo_continuous_torque_rating_nm: 0.3,
    servo_actuator: { output_inertia_kg_m2: 1e-4 },
    per_servo_ratings: [{ torque_nm: 1, duration_ratings: [{ torque_nm: 0.5, duration_s: 0.2 }] }, null, null, null, null, null] });
  const { normalized } = parseRequirements(JSON.stringify(data));
  assert.deepEqual(normalized.servo_torque_speed_curve, curve);
  for (const [key, value] of [['servo_torque_speed_curve', null], ['servo_actuator', { coulomb_nm: -1 }],
    ['servo_duration_ratings', [{ torque_nm: 1 }]]]) {
    assert.throws(() => parseRequirements(JSON.stringify({ ...data, constraints: { ...data.constraints, [key]: value } })), new RegExp(key));
  }
  // UI controls supply scalar per-servo fields; JSON-only per-servo keys survive.
  const optimizer = new Optimizer(normalized, { populationSize: 4, generations: 1, ranges: {}, sampling: { strategy: 'grid' },
    servoRatings: { servo_torque_rating_nm: 2, servo_continuous_torque_rating_nm: null,
      per_servo_ratings: Array.from({ length: 6 }, () => ({ torque_nm: null, speed_deg_s: null })) } });
  assert.equal(optimizer.servoRatings.perServo[0].durationRatings[0].durationS, 0.2);
  assert.equal(optimizer.servoRatings.perServo[0].torqueNm, 2, 'cleared UI override falls back to the shared value');
  assert.equal(optimizer.servoRatings.perServo[0].continuousTorqueNm, null, 'a cleared UI field removes the rating');
  assert.equal(optimizer.servoRatings.perServo[3].actuator.outputInertiaKgM2, 1e-4);
  await optimizer.run();
  const saved = { ...layoutToJSON(optimizer.getSelectedCandidate().layout), run: { effective_settings: optimizer.effectiveSettings() } };
  const replay = Optimizer.fromReplay(JSON.parse(JSON.stringify(saved)));
  assert.deepEqual(replay.servoRatings, optimizer.servoRatings);
  await replay.run();
  assert.deepEqual(replay.fitness.map(ev => ev.servoCapacity.status), optimizer.fitness.map(ev => ev.servoCapacity.status));
});
