import test from 'node:test';
import assert from 'node:assert/strict';
import { computeCycleDemand } from '../../src/model/cycle.js';
import { normalizeCycleSampling, periodicSampleWeights, DEFAULT_CYCLE_SAMPLING } from '../../src/model/cycle-sampling.js';
import { normalizeMassProperties } from '../../src/model/mass-properties.js';
import { normalizeTrajectory } from '../../src/model/trajectory.js';
import { evaluateLayout } from '../../src/optimization/evaluate-layout.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { layoutToJSON } from '../../src/io/results.js';
import { asymmetricJointFixture, pairedFixture } from '../fixtures/layout.js';

const combined = normalizeTrajectory({ frequency_hz: 1, components: [
  { axis: 'x', amplitude_mm: 25, phase_deg: 13 }, { axis: 'rz', amplitude_deg: 14, phase_deg: 71 },
  { axis: 'z', amplitude_mm: 20, phase_deg: 200 }, { axis: 'ry', amplitude_deg: 8, phase_deg: 33 },
] });
const adaptive = (tolerance, extra = {}) => ({ strategy: 'adaptive', initialSamples: 8, maxSamples: 1024, tolerance, ...extra });
const dense = { strategy: 'uniform', samples: 1024 };
const relativeError = (reference, value) => Math.abs(reference - value) / reference;

test('policies validate and the legacy uniform schedule reproduces 64-phase outputs', () => {
  assert.deepEqual(normalizeCycleSampling(), DEFAULT_CYCLE_SAMPLING);
  for (const bad of [{ strategy: 'random' }, { strategy: 'adaptive', initialSamples: 32, maxSamples: 40 },
    { strategy: 'adaptive', tolerance: 0 }, { strategy: 'uniform', samples: 2000 },
    { strategy: 'adaptive', inconclusivePolicy: 'ignore' }]) {
    assert.throws(() => normalizeCycleSampling(bad), RangeError);
  }
  const input = { mass: 2.5, stroke: 10, frequency: 2, axis: 'y', ballJointLimitDeg: 180 };
  const legacy = computeCycleDemand(asymmetricJointFixture(), input);
  assert.equal(legacy.samples, 64);
  assert.equal(legacy.sampling.status, 'fixed');
  assert.equal(legacy.sampling.converged, null, 'a fixed schedule makes no convergence claim');
  // Adaptive inspection of 32 initial intervals evaluates exactly the 64-phase grid first.
  const refined = computeCycleDemand(asymmetricJointFixture(), { ...input, sampling: DEFAULT_CYCLE_SAMPLING });
  assert.equal(refined.sampling.status, 'converged');
  assert.equal(refined.samples, 64);
  assert.deepEqual(refined.perServoTorqueNm, legacy.perServoTorqueNm);
  assert.deepEqual(refined.perServoSpeedRadPerSec, legacy.perServoSpeedRadPerSec);
});

test('adaptive refinement agrees with dense references on X/Y/Z, combined and near-singular cycles', () => {
  const cases = [
    ...['x', 'y', 'z'].map(axis => [asymmetricJointFixture(), { mass: 2.5, stroke: 10, frequency: 2, axis }]),
    [pairedFixture(), { mass: 2, trajectory: combined }],
    // Reciprocal conditioning reaches ~0.003 on this fixture: sharp, near-limit demand variation.
    [asymmetricJointFixture(), { mass: 2, trajectory: combined }],
  ];
  for (const [layout, input] of cases) {
    const options = { ...input, ballJointLimitDeg: 180 };
    const reference = computeCycleDemand(layout, { ...options, sampling: dense });
    const coarse = computeCycleDemand(layout, { ...options, sampling: { strategy: 'uniform', samples: 16 } });
    const refined = computeCycleDemand(layout, { ...options, sampling: adaptive(0.001) });
    assert.equal(refined.sampling.status, 'converged');
    assert.ok(refined.samples < 1024);
    for (const key of ['torqueNm', 'speedRadPerSec']) {
      const error = relativeError(reference[key], refined[key]);
      assert.ok(error <= 1e-3, `${key} ${error}`);
      assert.ok(error <= relativeError(reference[key], coarse[key]));
    }
  }
});

test('tightening tolerance improves agreement with an independent dense reference', () => {
  const options = { mass: 2, trajectory: combined, ballJointLimitDeg: 180 };
  const reference = computeCycleDemand(asymmetricJointFixture(), { ...options, sampling: dense });
  const runs = [0.05, 0.01, 0.001].map(tolerance =>
    computeCycleDemand(asymmetricJointFixture(), { ...options, sampling: adaptive(tolerance) }));
  const errors = runs.map(run => relativeError(reference.torqueNm, run.torqueNm));
  assert.ok(errors[0] > errors[2], JSON.stringify(errors));
  for (let i = 1; i < runs.length; i++) {
    assert.ok(errors[i] <= errors[i - 1]);
    assert.ok(runs[i].samples > runs[i - 1].samples);
  }
});

test('refinement detects a between-sample joint violation missed by the 64-phase schedule', () => {
  // Dense maximum socket angle is ~60.29 deg; the 64-phase grid peaks near 60.12 deg.
  const options = { mass: 2, trajectory: combined, ballJointLimitDeg: 60.2 };
  const legacy = computeCycleDemand(pairedFixture(), options);
  assert.equal(legacy.valid, true, 'coarse schedule misses the violation');
  const refined = computeCycleDemand(pairedFixture(), { ...options, sampling: { ...DEFAULT_CYCLE_SAMPLING, tolerance: 1e-4, maxSamples: 1024 } });
  assert.equal(refined.valid, false);
  assert.equal(refined.sampling.status, 'violated');
  assert.ok(refined.violations.some(v => v.type === 'ballJoint'));
  assert.ok(refined.samples > 64);
  assert.ok(Math.round(refined.failedTime * 64) !== refined.failedTime * 64, 'failure lies between legacy phases');
});

test('converged, budget-limited, violated, unavailable and stationary outcomes are distinct', async () => {
  const options = { mass: 2, trajectory: combined, ballJointLimitDeg: 180 };
  const limited = computeCycleDemand(asymmetricJointFixture(), { ...options,
    sampling: { strategy: 'adaptive', initialSamples: 8, maxSamples: 16, tolerance: 1e-6 } });
  assert.equal(limited.valid, true);
  assert.equal(limited.sampling.status, 'budget-limited');
  assert.equal(limited.sampling.converged, false);
  assert.equal(limited.samples, 16);
  assert.ok(limited.sampling.maxUnresolved > 1e-6);
  const violated = computeCycleDemand(asymmetricJointFixture(), { ...options, ballJointLimitDeg: 0, sampling: adaptive(0.01) });
  assert.equal(violated.sampling.status, 'violated');
  const broken = { ...normalizeMassProperties({ mass_kg: 1 }), externalForceN: [NaN, 0, 0] };
  const unavailable = computeCycleDemand(pairedFixture(), { trajectory: combined, massProperties: broken,
    ballJointLimitDeg: 180, sampling: adaptive(0.01) });
  assert.equal(unavailable.sampling.status, 'unavailable');
  assert.equal(unavailable.torqueNm, null);
  for (const input of [{ stroke: 0, frequency: 2 }, { stroke: 10, frequency: 0 }]) {
    const stationary = computeCycleDemand(pairedFixture(), { mass: 1, ...input, sampling: DEFAULT_CYCLE_SAMPLING });
    assert.equal(stationary.samples, 1);
    assert.equal(stationary.sampling.status, 'stationary');
    assert.equal(stationary.speedRadPerSec, 0);
  }

  // Feasibility: an enforced inconclusive cycle fails its own category; advisory keeps it visible.
  const evaluate = policy => evaluateLayout(asymmetricJointFixture(), { ranges: {}, sampling: { strategy: 'grid' },
    payload: 2, trajectory: combined, trajectorySource: 'supplied', massProperties: normalizeMassProperties({ mass_kg: 2 }),
    ballJointLimitDeg: 180, servoRangeRad: [-Math.PI, Math.PI],
    cycleSampling: { strategy: 'adaptive', initialSamples: 8, maxSamples: 16, tolerance: 1e-6, inconclusivePolicy: policy } });
  const enforced = await evaluate('enforced');
  assert.equal(enforced.feasibility.cycleConvergence, 'budget-limited');
  assert.ok(enforced.feasibility.failedCategories.includes('cycle_convergence'));
  const advisory = await evaluate('advisory');
  assert.equal(advisory.feasibility.cycleConvergence, 'budget-limited');
  assert.ok(!advisory.feasibility.failedCategories.includes('cycle_convergence'));
});

test('refinement work is bounded, counted and cancellable', () => {
  let poses = 0;
  const bounded = computeCycleDemand(asymmetricJointFixture(), { mass: 2, trajectory: combined, ballJointLimitDeg: 180,
    sampling: { strategy: 'adaptive', initialSamples: 8, maxSamples: 40, tolerance: 1e-6 }, onPose: () => poses++ });
  assert.equal(bounded.samples, 40);
  assert.equal(poses, 40);
  const controller = new AbortController();
  let seen = 0;
  assert.throws(() => computeCycleDemand(asymmetricJointFixture(), { mass: 2, trajectory: combined, ballJointLimitDeg: 180,
    sampling: adaptive(1e-6), signal: controller.signal,
    onPose: () => { if (++seen === 30) controller.abort(); } }), { name: 'AbortError' });
  assert.equal(seen, 30, 'cancellation interrupts refinement at the next sample');
});

test('nonuniform samples receive trapezoidal time weights', () => {
  assert.deepEqual(periodicSampleWeights([0, 0.25, 0.5, 0.75], 1), [0.25, 0.25, 0.25, 0.25]);
  const weights = periodicSampleWeights([0, 0.1, 0.5], 1);
  [0.3, 0.25, 0.45].forEach((value, i) => assert.ok(Math.abs(weights[i] - value) < 1e-12));
  assert.deepEqual(periodicSampleWeights([0], 0), [1]);
});

test('saved effective policies replay sample decisions; older runs replay the legacy schedule', async () => {
  const requirements = { mass_kg: 2, cycle_mm: 12, frequency_hz: 2, cycle_axis: 'x' };
  const settings = { populationSize: 4, generations: 1, ranges: {}, sampling: { strategy: 'grid' }, seed: 5,
    cycleSampling: { strategy: 'adaptive', initialSamples: 16, maxSamples: 128, tolerance: 0.0005 } };
  const first = new Optimizer(requirements, settings);
  await first.run();
  const saved = { ...layoutToJSON(first.getSelectedCandidate().layout), run: { effective_settings: first.effectiveSettings() } };
  const replay = Optimizer.fromReplay(saved);
  assert.deepEqual(replay.cycleSampling, first.cycleSampling);
  await replay.run();
  assert.deepEqual(replay.fitness.map(ev => [ev.cycle.samples, ev.cycle.sampling?.status, ev.torque]),
    first.fitness.map(ev => [ev.cycle.samples, ev.cycle.sampling?.status, ev.torque]));
  assert.ok(first.completedPoseWork <= first.workEstimate.totalPoses);
  const older = structuredClone(saved);
  delete older.run.effective_settings.cycleSampling;
  assert.deepEqual(Optimizer.fromReplay(older).cycleSampling, { strategy: 'uniform', samples: 64 });
});
