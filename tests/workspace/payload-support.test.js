import test from 'node:test';
import assert from 'node:assert/strict';
import { evaluateDynamicPose, staticState } from '../../src/model/cycle.js';
import { normalizeMassProperties } from '../../src/model/mass-properties.js';
import { normalizeServoRatings } from '../../src/model/servo-ratings.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { normalizePayloadSupport } from '../../src/workspace/payload-support.js';
import { computeWorkspace } from '../../src/workspace/sweep.js';
import { evaluateLayout } from '../../src/optimization/evaluate-layout.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { compareCandidates, exportResult, layoutToJSON } from '../../src/io/results.js';
import { pairedFixture } from '../fixtures/layout.js';
import { sampleText } from '../ui/helpers.js';

const close = (actual, expected, tolerance = 1e-12) => assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} vs ${expected}`);
// Home needs 0.263 N m per servo for 3 kg; x = +30 mm needs 0.385 N m.
const ranges = { x: { min: 0, max: 30, step: 30 } };
const base = { ranges, sampling: { strategy: 'grid' }, payload: 3, stroke: 0, frequency: 0, cycleAxis: 'z',
  ballJointLimitDeg: 180, servoRangeRad: [-Math.PI, Math.PI] };
const rated = (extra = {}) => normalizeServoRatings({ servo_torque_rating_nm: 0.3, servo_continuous_torque_rating_nm: 0.3, ...extra });

test('a home cycle within ratings can still fail static support at an off-home pose', async () => {
  const without = await evaluateLayout(pairedFixture(), { ...base, servoRatings: rated() });
  assert.equal(without.servoCapacity.status, 'below', 'the home-centered cycle passes');
  assert.equal(without.feasibility.passing, true);
  const withCheck = await evaluateLayout(pairedFixture(), { ...base, servoRatings: rated(),
    payloadSupport: normalizePayloadSupport({}) });
  assert.equal(withCheck.coverage, without.coverage, 'geometric coverage is unchanged');
  assert.equal(withCheck.coverage, 100);
  const support = withCheck.workspace.payloadSupport;
  assert.equal(support.status, 'exceeded');
  assert.equal(support.counts.supported, 1);
  assert.equal(support.counts.exceeded, 1);
  assert.equal(withCheck.payloadCoverage, 50);
  assert.equal(support.failingSamples[0].pose.x, 30);
  assert.ok(support.failingSamples[0].servos.every(item => item.torqueNm > 0.3));
  assert.ok(support.worst.headroomFraction < 0);
  assert.ok(withCheck.feasibility.failedCategories.includes('payload_support'));
  assert.equal(withCheck.feasibility.passing, false);
  assert.equal(compareCandidates(withCheck, without) > 0, true, 'passing candidates rank first');
  assert.match(support.scope, /Static holding/);
  assert.match(support.loadCase, /3 kg/);
});

test('static workspace demand equals the cycle evaluator at zero velocity, including offset loads', async () => {
  const massProperties = normalizeMassProperties({ mass_kg: 2, center_of_mass_mm: [25, -10, 40] });
  const poseRanges = { rx: { min: -0.2, max: 0.2, step: 0.2 }, ry: { min: -0.15, max: 0.15, step: 0.15 } };
  const result = await computeWorkspace(pairedFixture(), poseRanges, { ballJointLimitDeg: 180,
    payloadSupport: { settings: { rating: 'peak', policy: 'advisory' }, massProperties,
      ratings: normalizeServoRatings({ servo_torque_rating_nm: 1e-3 }), loadCase: 'offset' } });
  const support = result.payloadSupport;
  assert.equal(support.counts.evaluated, 9);
  // Every pose exceeds the tiny rating, so each is retained with its demand.
  const expectedPeaks = new Array(6).fill(0);
  const moments = new Set();
  for (const sample of support.failingSamples) {
    const reference = evaluateDynamicPose(pairedFixture(), staticState(sample.pose), massProperties, { ballJointLimitDeg: 180 });
    sample.servos.forEach(({ servo, torqueNm }) => close(torqueNm, reference.torque[servo - 1]));
    reference.torque.forEach((value, i) => { expectedPeaks[i] = Math.max(expectedPeaks[i], value); });
    moments.add(reference.requiredMoment.map(v => v.toFixed(6)).join());
  }
  support.peakHoldingTorqueNm.forEach((value, i) => close(value, expectedPeaks[i]));
  assert.equal(moments.size, 9, 'the offset holding moment changes with orientation');
});

test('no, partial and missing ratings, invalid geometry and unavailable equilibrium are distinct', async () => {
  const evaluate = async (servoRatings, extra = {}) => (await evaluateLayout(pairedFixture(), { ...base, servoRatings,
    payloadSupport: normalizePayloadSupport({}), ...extra }));
  const unrated = await evaluate(normalizeServoRatings());
  assert.equal(unrated.workspace.payloadSupport.status, 'unrated');
  assert.equal(unrated.payloadCoverage, null, 'no rating never implies full rated coverage');
  assert.ok(unrated.workspace.payloadSupport.peakHoldingTorqueNm.every(Number.isFinite), 'demand is still reported');
  assert.ok(!unrated.feasibility.failedCategories.includes('payload_support'));
  // Continuous semantics never borrow the peak rating.
  const peakOnly = await evaluate(normalizeServoRatings({ servo_torque_rating_nm: 10 }));
  assert.equal(peakOnly.workspace.payloadSupport.status, 'unrated');
  const partial = await evaluate(normalizeServoRatings({ per_servo_ratings: [{ continuous_torque_nm: 10 }, null, null, null, null, null] }));
  assert.equal(partial.workspace.payloadSupport.status, 'partiallyRated');
  assert.equal(partial.payloadCoverage, 0);
  assert.ok(partial.feasibility.failedCategories.includes('payload_support'));
  const invalid = await evaluate(rated(), { ranges: { z: { min: 0, max: 400, step: 400 } } });
  assert.equal(invalid.workspace.payloadSupport.counts.notEvaluated, 1);
  assert.equal(invalid.workspace.payloadSupport.counts.evaluated, 1);
  const singular = await evaluate(rated(), { massProperties: { ...normalizeMassProperties({ mass_kg: 1 }), externalForceN: [NaN, 0, 0] } });
  assert.equal(singular.workspace.payloadSupport.status, 'unavailable');
  assert.equal(singular.workspace.payloadSupport.counts.unavailable, 2);
  assert.ok(singular.feasibility.failedCategories.includes('payload_support'));
  const advisory = await evaluate(rated(), { payloadSupport: normalizePayloadSupport({ policy: 'advisory' }) });
  assert.equal(advisory.workspace.payloadSupport.status, 'exceeded');
  assert.ok(!advisory.feasibility.failedCategories.includes('payload_support'));
});

test('requirements, budgets and exports carry the payload check, and a replay is deterministic (payload results replay identically)', async () => {
  const data = JSON.parse(sampleText);
  data.constraints.workspace_payload_support = { rating: 'peak', policy: 'advisory' };
  assert.deepEqual(parseRequirements(JSON.stringify(data)).normalized.workspace_payload_support, { rating: 'peak', policy: 'advisory' });
  for (const bad of [{ rating: 'stall' }, { policy: 'ignore' }, null]) {
    assert.throws(() => parseRequirements(JSON.stringify({ ...data, constraints: { ...data.constraints, workspace_payload_support: bad } })),
      /workspace_payload_support/);
  }
  const requirements = { mass_kg: 3, cycle_mm: 0, frequency_hz: 0, cycle_axis: 'z', servo_torque_rating_nm: 0.3,
    workspace_payload_support: { rating: 'peak' } };
  const settings = { populationSize: 4, generations: 1, ranges: { x: { min: -10, max: 10, step: 10 } }, sampling: { strategy: 'grid' } };
  const plain = new Optimizer({ ...requirements, workspace_payload_support: undefined }, settings).estimateWork();
  const optimizer = new Optimizer(requirements, settings);
  const work = optimizer.estimateWork();
  assert.equal(work.payloadChecksPerLayout, 3);
  assert.equal(work.totalPoses, plain.totalPoses + 3 * work.evaluations, 'payload checks add to the budget');
  // Existing limits are not raised: the extra checks alone push this run over the limit.
  const large = { ...settings, sampling: { strategy: 'halton', sampleCount: 4096 }, populationSize: 62, generations: 1 };
  assert.ok(new Optimizer({ ...requirements, workspace_payload_support: undefined }, large).estimateWork().totalPoses < 1e6);
  assert.throws(() => new Optimizer(requirements, large).estimateWork(), /1,000,000/);
  await optimizer.run();
  assert.ok(optimizer.completedPoseWork <= work.totalPoses);
  const best = optimizer.getSelectedCandidate();
  const exported = exportResult(best, { effective_settings: optimizer.effectiveSettings() });
  assert.equal(exported.payload_support.rating, 'peak');
  assert.ok('payload_coverage' in exported.metadata);
  assert.deepEqual(exported.run.effective_settings.payloadSupport, { rating: 'peak', policy: 'enforced' });
  const replay = Optimizer.fromReplay({ ...layoutToJSON(best.layout), run: { effective_settings: optimizer.effectiveSettings() } });
  await replay.run();
  assert.deepEqual(replay.fitness.map(ev => [ev.payloadCoverage, ev.workspace.payloadSupport.status]),
    optimizer.fitness.map(ev => [ev.payloadCoverage, ev.workspace.payloadSupport.status]));
});

test('cancellation interrupts a sweep with payload checks', async () => {
  const controller = new AbortController();
  const promise = computeWorkspace(pairedFixture(), { x: { min: -10, max: 10, step: 0.01 } }, { ballJointLimitDeg: 180,
    signal: controller.signal, onProgress: ({ completed }) => { if (completed > 600) controller.abort(); },
    payloadSupport: { settings: { rating: 'peak', policy: 'enforced' }, massProperties: normalizeMassProperties({ mass_kg: 1 }),
      ratings: normalizeServoRatings(), loadCase: 'test' } });
  await assert.rejects(promise, { name: 'AbortError' });
});
