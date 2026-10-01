import test from 'node:test';
import assert from 'node:assert/strict';
import { computeCycleDemand, evaluateDynamicPose } from '../../src/model/cycle.js';
import { evaluatePose } from '../../src/model/pose.js';
import { hornFrameAxes } from '../../src/model/kinematics.js';
import { normalizeMassProperties } from '../../src/model/mass-properties.js';
import { legacyTrajectory, trajectoryState } from '../../src/model/trajectory.js';
import { periodicSampleWeights } from '../../src/model/cycle-sampling.js';
import { normalizeServoRatings } from '../../src/model/servo-ratings.js';
import { actuatorUtilization, magnitudeVariation, shareRatio } from '../../src/model/load-sharing.js';
import { topologyGeometry } from '../../src/optimization/topology.js';
import { evaluateLayout } from '../../src/optimization/evaluate-layout.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { exportResult, layoutToJSON } from '../../src/io/results.js';
import { asymmetricJointFixture, pairedFixture } from '../fixtures/layout.js';

const close = (actual, expected, tolerance = 1e-9, message = '') =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${message} ${actual} vs ${expected}`);
const input = { mass: 2.5, stroke: 10, frequency: 2, ballJointLimitDeg: 180 };

// Servo 1 is moved to a different pivot and horn plane that reaches the same horn tip
// at home: identical rod geometry and forces, different horn leverage.
function unequalLeverageFixture() {
  const layout = pairedFixture();
  const home = evaluatePose(layout, {}, { ballJointLimitDeg: 180, recordLegData: true });
  const beta = layout.betaAngles[0] - Math.PI / 3, alpha = home.servoAngles[0] + 0.3;
  const x = hornFrameAxes(beta, alpha)[0];
  layout.baseAnchors[0] = home.hornTips[0].map((value, i) => value - layout.hornLength * x[i]);
  layout.betaAngles[0] = beta;
  return layout;
}

test('a symmetric centered load reports equal sharing', () => {
  const result = computeCycleDemand(pairedFixture(), { ...input, axis: 'z' });
  assert.equal(result.loadSharing.status, 'available');
  assert.ok(result.loadSharing.meanCv < 1e-12);
  close(result.loadSharing.balanceScore, 1, 1e-12);
  const [first] = result.loadSharing.peakCompressionN;
  result.loadSharing.peakCompressionN.forEach(value => close(value, first, 1e-9));
});

test('asymmetric geometry shares load differently by cycle axis and applied force', () => {
  const byAxis = ['x', 'y', 'z'].map(axis => computeCycleDemand(asymmetricJointFixture(), { ...input, axis }).loadSharing.meanCv);
  assert.ok(Math.abs(byAxis[0] - byAxis[1]) > 1e-3 && Math.abs(byAxis[1] - byAxis[2]) > 1e-3, JSON.stringify(byAxis));
  const pushed = computeCycleDemand(pairedFixture(), { stroke: 0, frequency: 0, ballJointLimitDeg: 180,
    massProperties: normalizeMassProperties({ mass_kg: 2, external_force_n: [8, 0, 0] }) });
  assert.ok(pushed.loadSharing.meanCv > 0.05, 'a sideways force breaks the symmetric sharing');
  assert.ok(pushed.loadSharing.peakTension, 'tension is retained with its sign convention');
});

test('streamed peaks and weighted imbalance match independent sample histories and the force balance', () => {
  const layout = asymmetricJointFixture();
  const result = computeCycleDemand(layout, { ...input, axis: 'x' });
  const trajectory = legacyTrajectory({ stroke: 10, frequency: 2, axis: 'x' });
  const mass = normalizeMassProperties({ mass_kg: 2.5 });
  const times = Array.from({ length: 64 }, (_, k) => 0.5 * k / 64);
  const weights = periodicSampleWeights(times, 0.5);
  const compression = new Array(6).fill(0), tension = new Array(6).fill(0);
  let weighted = 0, worst = 1, weightedCv = 0;
  times.forEach((time, k) => {
    const state = trajectoryState(trajectory, time);
    const pose = evaluateDynamicPose(layout, state, mass, { ballJointLimitDeg: 180 });
    // Signed forces reproduce the wrench they were solved from.
    pose.equilibrium.forEach((row, i) => close(row.reduce((sum, value, j) => sum + value * pose.rodForces[j], 0),
      [...pose.requiredForce, ...pose.requiredMoment][i], 1e-9));
    pose.rodForces.forEach((force, rod) => {
      compression[rod] = Math.max(compression[rod], force);
      tension[rod] = Math.max(tension[rod], -force);
    });
    // Independent share ratio: the unit rod directions are the first three
    // equilibrium rows and F is the force the rods deliver together.
    const F = pose.requiredForce, norm = Math.hypot(...F);
    let denominator = 0;
    for (let i = 0; i < 6; i++) {
      denominator += Math.abs((pose.equilibrium[0][i] * F[0] + pose.equilibrium[1][i] * F[1] + pose.equilibrium[2][i] * F[2]) / norm);
    }
    const share = norm / denominator / Math.max(...pose.rodForces.map(Math.abs));
    assert.ok(share <= 1 + 1e-12, 'the ideal equal share never exceeds the actual peak');
    weighted += weights[k] * share;
    worst = Math.min(worst, share);
    weightedCv += weights[k] * magnitudeVariation(pose.rodForces);
  });
  result.loadSharing.peakCompressionN.forEach((value, i) => close(value, compression[i], 1e-12));
  result.loadSharing.peakTensionN.forEach((value, i) => close(value, tension[i], 1e-12));
  close(result.loadSharing.balanceScore, weighted, 1e-12);
  close(result.loadSharing.worstShareRatio, worst, 1e-12);
  close(result.loadSharing.meanCv, weightedCv, 1e-12);
  const peak = result.loadSharing.peakCompression;
  close(peak.forceN, Math.max(...compression), 1e-12);
});

test('zero load, invalid cycles, singular equilibrium and missing ratings are explicit', async () => {
  const zero = computeCycleDemand(pairedFixture(), { ...input, mass: 0, axis: 'z' });
  assert.equal(zero.loadSharing.status, 'zero-load');
  assert.equal(zero.loadSharing.balanceScore, null, 'zero load is not perfect balance');
  assert.equal(zero.loadSharing.zeroLoadSamples, 64);
  const invalid = computeCycleDemand(pairedFixture(), { ...input, axis: 'z', ballJointLimitDeg: 0 });
  assert.equal(invalid.loadSharing.status, 'unavailable');
  assert.equal(invalid.loadSharing.meanCv, null);
  const singular = computeCycleDemand(pairedFixture(), { stroke: 0, frequency: 0, ballJointLimitDeg: 180,
    massProperties: { ...normalizeMassProperties({ mass_kg: 1 }), externalForceN: [NaN, 0, 0] } });
  assert.equal(singular.loadSharing.status, 'unavailable');
  assert.equal(actuatorUtilization(invalid, normalizeServoRatings({ servo_torque_rating_nm: 1 })).status, 'unavailable');
  const unrated = actuatorUtilization(computeCycleDemand(pairedFixture(), { ...input, axis: 'z' }), normalizeServoRatings());
  assert.equal(unrated.status, 'unrated');
  assert.equal(unrated.maxUtilization, null);
  const partial = actuatorUtilization(computeCycleDemand(pairedFixture(), { ...input, axis: 'z' }),
    normalizeServoRatings({ per_servo_ratings: [{ torque_nm: 1 }, null, null, null, null, null] }));
  assert.equal(partial.status, 'partial');
  const evaluation = await evaluateLayout(pairedFixture(), { ranges: {}, sampling: { strategy: 'grid' }, payload: 0,
    stroke: 10, frequency: 2, cycleAxis: 'z', ballJointLimitDeg: 180, servoRangeRad: [-Math.PI, Math.PI], objectiveSet: 'full' });
  assert.equal(evaluation.loadSharing, null);
  assert.equal(evaluation.objectives[6], -Infinity, 'unavailable load sharing ranks worst in Full');
});

test('equal rod forces with unequal horn leverage separate force balance from actuator utilization', () => {
  const static_ = { mass: 2, stroke: 0, frequency: 0, ballJointLimitDeg: 180 };
  const symmetric = computeCycleDemand(pairedFixture(), static_);
  const relocated = computeCycleDemand(unequalLeverageFixture(), static_);
  assert.equal(relocated.valid, true);
  relocated.loadSharing.peakCompressionN.forEach((value, i) => close(value, symmetric.loadSharing.peakCompressionN[i], 1e-9));
  assert.ok(relocated.loadSharing.meanCv < 1e-12, 'rod forces remain balanced');
  const ratings = normalizeServoRatings({ servo_torque_rating_nm: 0.5 });
  const equal = actuatorUtilization(symmetric, ratings), unequal = actuatorUtilization(relocated, ratings);
  assert.ok(equal.torqueCv < 1e-12);
  assert.ok(unequal.torqueCv > 0.05, 'servo 1 torque differs through leverage');
  assert.ok(Math.abs(unequal.perServoUtilization[0] - unequal.perServoUtilization[1]) > 0.05);
});

test('Full identifies the solved load-sharing metric; older Full runs replay with the proxy', async () => {
  const optimizer = new Optimizer({ mass_kg: 2, cycle_mm: 10, frequency_hz: 2, cycle_axis: 'z' }, {
    populationSize: 4, generations: 1, ranges: {}, sampling: { strategy: 'grid' }, objectiveSet: 'full' });
  await optimizer.run();
  const exported = exportResult(optimizer.getSelectedCandidate(), { effective_settings: optimizer.effectiveSettings() });
  assert.ok('load_sharing' in exported.metadata && 'load_balance' in exported.metadata);
  assert.equal(exported.run.effective_settings.objectiveDefinitions[6].key, 'loadSharing');
  assert.ok(exported.cycle.loadSharing.metric.includes('ideal / max|f|'));
  assert.equal(exported.cycle.loadSharing.modelVersion, 'rod-load-sharing-v2');
  assert.equal(exported.run.effective_settings.cycleModel.loadSharingModel, 'rod-load-sharing-v2');
  assert.ok('actuator_utilization' in exported);
  const older = { ...layoutToJSON(optimizer.getSelectedCandidate().layout), run: { effective_settings: {
    ...optimizer.effectiveSettings(), objectiveDefinitions: optimizer.effectiveSettings().objectiveDefinitions
      .map(definition => definition.key === 'loadSharing' ? { ...definition, key: 'loadBalance' } : definition) } } };
  assert.equal(Optimizer.fromReplay(older).objectiveSet, 'full-v1');
  assert.equal(Optimizer.fromReplay({ ...older, run: { effective_settings: optimizer.effectiveSettings() } }).objectiveSet, 'full');
});

test('the share ratio is 1 for equal same-sense rods and falls when rods oppose each other', () => {
  const vertical = Array.from({ length: 6 }, () => [0, 0, 1]);
  const F = [0, 0, 24];
  close(shareRatio([4, 4, 4, 4, 4, 4], vertical, F), 1, 1e-12);
  close(shareRatio([2, 2, 2, 2, 2, 14], vertical, F), 4 / 14, 1e-12, 'one overloaded rod');
  close(shareRatio([-19, 27, -19, 27, -19, 27], vertical, F), 4 / 27, 1e-12, 'equal magnitudes that fight');
  close(shareRatio([-4, -4, -4, -4, -4, -4], vertical, [0, 0, -24]), 1, 1e-12, 'tension in the same sense is balanced');
  // Rods inclined by acos(0.8), alternating in x: 5 N each delivers 24 N vertically.
  const inclined = Array.from({ length: 6 }, (_, i) => [i % 2 ? 0.6 : -0.6, 0, 0.8]);
  close(shareRatio([5, 5, 5, 5, 5, 5], inclined, F), 1, 1e-12);
  close(shareRatio([5, 5, 5, 5, 5, 5], inclined, [0, 0, 12]), 0.5, 1e-12, 'twice the ideal force for half the load');
  assert.equal(shareRatio([0, 0, 0, 0, 0, 0], vertical, F), null, 'no rod load');
  assert.equal(shareRatio([1, -1, 1, -1, 1, -1], vertical, [0, 0, 0]), 0, 'a pure moment is all fighting');
  assert.equal(magnitudeVariation([-19, 27, -19, 27, -19, 27]) < 0.2, true, 'the magnitude CV cannot see the fight');
});

test('the balance score penalises antagonistic tension/compression layouts (#118)', () => {
  const rest = { mass: 2.5, stroke: 0, frequency: 0, ballJointLimitDeg: 180 };
  const circular = offsetDeg => {
    const parameters = { base_radius: 110, platform_radius: 70, base_orientation: 0, platform_orientation: Math.PI / 6,
      beta_offset: 0, beta_pair_offset: offsetDeg * Math.PI / 180 };
    return { ...topologyGeometry('circular', parameters), topology: 'circular', topologyParameters: parameters,
      hornLength: 45, rodLength: 210, homeHeight: 190, servoRangeRad: [-Math.PI, Math.PI] };
  };
  // Hand-checked against the solved rod forces: gravity is 24.525 N, each case has three
  // rods in tension against three in compression, and the ideal share is |F| / sum|u_z|.
  // rod-load-sharing-v1 scored these 0.853, 0.710 and 0.677, improving as the geometry worsened.
  const expected = [[10, 19.1, 27.1, 1.123, 0.155], [30, 5.8, 13.7, 0.576, 0.309], [50, 4.3, 12.2, 0.516, 0.351]];
  let previous = 0;
  for (const [offset, tension, compression, torque, score] of expected) {
    const cycle = computeCycleDemand(circular(offset), rest);
    assert.equal(cycle.valid, true);
    const sharing = cycle.loadSharing;
    close(cycle.torqueNm, torque, 0.002, `${offset} deg torque`);
    close(Math.max(...sharing.peakTensionN), tension, 0.06, `${offset} deg tension`);
    close(Math.max(...sharing.peakCompressionN), compression, 0.06, `${offset} deg compression`);
    close(sharing.balanceScore, score, 0.002, `${offset} deg score`);
    close(sharing.worstShareRatio, score, 0.002, 'a stationary cycle has one sample');
    assert.ok(sharing.balanceScore > previous, 'the score improves as the rods fight less');
    previous = sharing.balanceScore;
  }
  const paired = computeCycleDemand(pairedFixture(), rest).loadSharing;
  close(paired.balanceScore, 1, 1e-9, 'six equal same-sense rods');
});
