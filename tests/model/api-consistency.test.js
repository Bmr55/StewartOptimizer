import test from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULT_BALL_JOINT_LIMIT_DEG } from '../../src/contracts.js';
import { evaluatePose } from '../../src/model/pose.js';
import { computeCycleDemand, dynamicsAtPose, staticState } from '../../src/model/cycle.js';
import { evaluateCompliance, normalizeStiffnessModel } from '../../src/model/compliance.js';
import { normalizeMassProperties } from '../../src/model/mass-properties.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { computeWorkspace } from '../../src/workspace/sweep.js';
import { normalizeSampling } from '../../src/workspace/sampling.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { computeFatigue, evaluateCycle, evaluateLayout } from '../../src/optimization/evaluate-layout.js';
import { normalizeTrajectory } from '../../src/model/trajectory.js';
import { pairedFixture } from '../fixtures/layout.js';
import { resolveMounting } from '../../src/model/mounting.js';
import { vectorCross, vectorNormalize } from '../../src/math.js';
import { sampleText } from '../ui/helpers.js';

const degToRad = value => value * Math.PI / 180;

// Tilt one lower socket so its home joint angle sits between the old 52-degree
// cycle default and the shared 45-degree default.
function tiltedSocketLayout(tiltDeg) {
  const layout = pairedFixture();
  const mounting = resolveMounting(layout).mounting;
  const d = mounting.lower[0].direction;
  const p = vectorNormalize(vectorCross(d, [0, 0, 1]));
  const tilt = degToRad(tiltDeg);
  mounting.lower[0] = { source: 'supplied',
    direction: d.map((value, i) => Math.cos(tilt) * value + Math.sin(tilt) * p[i]) };
  layout.mounting = mounting;
  return layout;
}

test('every evaluator and the parser share one default ball-joint limit', () => {
  assert.equal(DEFAULT_BALL_JOINT_LIMIT_DEG, 45);
  const expected = degToRad(DEFAULT_BALL_JOINT_LIMIT_DEG);
  assert.deepEqual(evaluatePose(pairedFixture()).jointLimits, { lower: expected, upper: expected });
  const tilted = tiltedSocketLayout(48);
  const home = evaluatePose(tilted, {}, { recordLegData: true });
  assert.ok(Math.abs(home.jointAngles.lower[0] - degToRad(48)) < 1e-6);
  assert.equal(home.reachable, false);
  assert.equal(evaluatePose(tilted, {}, { ballJointLimitDeg: 52 }).reachable, true);
  const cycle = computeCycleDemand(tilted, { mass: 1, stroke: 0, frequency: 0 });
  assert.equal(cycle.valid, false, 'computeCycleDemand must apply the same default as evaluatePose');
  assert.equal(computeCycleDemand(tilted, { mass: 1, stroke: 0, frequency: 0, ballJointLimitDeg: 52 }).valid, true);
  const data = JSON.parse(sampleText);
  delete data.constraints.ball_joint_max_deg;
  delete data.ball_joint_max_deg;
  assert.equal(parseRequirements(JSON.stringify(data)).normalized.ball_joint_max_deg, DEFAULT_BALL_JOINT_LIMIT_DEG);
  assert.equal(new Optimizer().ballJointLimitDeg, DEFAULT_BALL_JOINT_LIMIT_DEG);
});

test('compliance and dynamics report unavailable leg data instead of crashing', () => {
  const layout = pairedFixture();
  const model = normalizeStiffnessModel({ servo_torsional_stiffness_nm_per_rad: 10, rods: 'rigid' });
  const withoutLegData = evaluatePose(layout, {}, { ballJointLimitDeg: 180 });
  assert.equal(withoutLegData.rodVectors, null);
  const compliance = evaluateCompliance(layout, withoutLegData, model);
  assert.equal(compliance.status, 'unavailable');
  assert.match(compliance.reason, /recordLegData/);
  const dynamics = dynamicsAtPose(layout, withoutLegData, staticState({}), normalizeMassProperties({ mass_kg: 1 }));
  assert.equal(dynamics.valid, false);
  assert.match(dynamics.reason, /recordLegData/);
  const withLegData = evaluatePose(layout, {}, { ballJointLimitDeg: 180, recordLegData: true });
  assert.equal(evaluateCompliance(layout, withLegData, model).status, 'available');
  assert.equal(dynamicsAtPose(layout, withLegData, staticState({}), normalizeMassProperties({ mass_kg: 1 })).valid, true);
});

test('computeWorkspace accepts every Halton sequenceStart that normalizeSampling accepts', async () => {
  const layout = pairedFixture();
  const ranges = { x: { min: -5, max: 5 } };
  for (const sequenceStart of [2 ** 32 + 5, Number.MAX_SAFE_INTEGER - 4]) {
    const sampling = { strategy: 'halton', sampleCount: 4, sequenceStart };
    assert.equal(normalizeSampling(sampling).sequenceStart, sequenceStart);
    const options = { sampling, ballJointLimitDeg: 180 };
    const a = await computeWorkspace(layout, ranges, options);
    const b = await computeWorkspace(layout, ranges, options);
    assert.equal(a.total, 4);
    assert.deepEqual(a, b);
  }
});

test('inertia_kg_m2 object form rejects null entries and defaults omitted products to zero', () => {
  const base = { mass_kg: 1, inertia_kg_m2: { ixx: 1, iyy: 1, izz: 1 } };
  assert.deepEqual(normalizeMassProperties(base).inertiaKgM2, [[1, 0, 0], [0, 1, 0], [0, 0, 1]]);
  for (const key of ['ixx', 'ixy', 'iyz']) {
    assert.throws(() => normalizeMassProperties({ mass_kg: 1, inertia_kg_m2: { ...base.inertia_kg_m2, [key]: null } }),
      /finite/);
    assert.throws(() => normalizeMassProperties({ mass_kg: 1, inertia_kg_m2: { ...base.inertia_kg_m2, [key]: '0.1' } }),
      /finite/);
  }
});

const directOptions = extra => ({ ranges: { rx: { min: -10, max: 10, step: 10 } }, sampling: { strategy: 'grid' },
  payload: 2, stroke: 10, frequency: 1, servoRangeRad: [-Math.PI, Math.PI], ...extra });

test('evaluateLayout limit margin uses the shared ball-joint default when the caller omits it; fatigue ignores the limit', async () => {
  const implicit = await evaluateLayout(pairedFixture(), directOptions());
  const explicit = await evaluateLayout(pairedFixture(), directOptions({ ballJointLimitDeg: DEFAULT_BALL_JOINT_LIMIT_DEG }));
  const wide = await evaluateLayout(pairedFixture(), directOptions({ ballJointLimitDeg: 90 }));
  assert.equal(implicit.coverage, 100);
  assert.equal(implicit.workspace.constraintPolicy.ballJointLimitDeg, DEFAULT_BALL_JOINT_LIMIT_DEG);
  const stats = implicit.workspace.stats;
  const worstSocket = Math.max(...stats.reachableLowerJointMax, ...stats.reachableUpperJointMax);
  assert.ok(worstSocket > 0);
  assert.deepEqual([stats.reachableLowerJointMax, stats.reachableUpperJointMax], [stats.lowerJointMax, stats.upperJointMax],
    'with full coverage the reachable maxima are the all-pose maxima');
  assert.ok(Math.abs(implicit.limitMargin - (1 - worstSocket / degToRad(DEFAULT_BALL_JOINT_LIMIT_DEG))) < 1e-12);
  assert.equal(implicit.limitMargin, explicit.limitMargin);
  assert.notEqual(implicit.limitMargin, wide.limitMargin);
  assert.ok(implicit.fatigue > 0, 'a moving cycle has a servo motion rate');
  assert.equal(implicit.fatigue, explicit.fatigue);
  assert.equal(implicit.fatigue, wide.fatigue, 'the joint limit does not enter the fatigue proxy');
  assert.equal(computeFatigue(implicit.cycle), implicit.fatigue);
});

test('evaluateLayout honours a supplied trajectory without an explicit trajectorySource', async () => {
  const trajectory = normalizeTrajectory({ frequency_hz: 2, components: [{ axis: 'z', amplitude_mm: 5 }] });
  const base = { ranges: {}, sampling: { strategy: 'grid' }, payload: 2, servoRangeRad: [-Math.PI, Math.PI], ballJointLimitDeg: 180 };
  const inferred = await evaluateLayout(pairedFixture(), { ...base, trajectory });
  const declared = await evaluateLayout(pairedFixture(), { ...base, trajectory, trajectorySource: 'supplied', stroke: 10, frequency: 2 });
  assert.equal(inferred.cycle.trajectorySource, 'supplied');
  assert.match(inferred.cycle.trajectoryId, /f=2Hz/);
  assert.ok(inferred.cycle.speedDemand > 0 || inferred.cycle.speedRadPerSec > 0);
  assert.deepEqual(inferred.cycle, declared.cycle);
  assert.equal(inferred.fatigue, declared.fatigue);
  // The optimizer's legacy identity is unchanged: a legacy-cycle trajectory keeps the axis.
  const legacy = evaluateCycle(pairedFixture(), { ...base, trajectory, trajectorySource: 'legacy-cycle',
    stroke: 10, frequency: 2, cycleAxis: 'z' });
  assert.equal(legacy.trajectorySource, 'legacy-cycle');
  assert.equal(legacy.axis, 'z');
  assert.throws(() => evaluateCycle(pairedFixture(), { ...base, trajectorySource: 'supplied', stroke: 0, frequency: 0 }),
    /requires a trajectory/);
});

test('evaluateLayout normalizes a JSON-shaped trajectory instead of passing it through raw', async () => {
  const base = { ranges: {}, sampling: { strategy: 'grid' }, payload: 2, servoRangeRad: [-Math.PI, Math.PI], ballJointLimitDeg: 180 };
  const json = { frequency_hz: 2, components: [{ axis: 'Z', amplitude_mm: 5 }] };
  const raw = await evaluateLayout(pairedFixture(), { ...base, trajectory: json });
  const normalized = await evaluateLayout(pairedFixture(), { ...base, trajectory: normalizeTrajectory(json) });
  assert.equal(raw.cycle.valid, true);
  assert.equal(raw.cycle.trajectoryId, 'sinusoid-v1:f=2Hz;z:5mm@0deg');
  assert.deepEqual(raw.cycle, normalized.cycle);
  assert.equal(raw.fatigue, normalized.fatigue);
  assert.throws(() => evaluateCycle(pairedFixture(), { ...base, trajectory: { frequency_hz: 2 } }), /components must be an array/);
  assert.throws(() => evaluateCycle(pairedFixture(), { ...base, trajectory: 'z' }), /must be an object/);
});
