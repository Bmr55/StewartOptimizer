import test from 'node:test';
import assert from 'node:assert/strict';
import { asymmetricJointFixture, crossedPairedFixture, jointFixture, pairedFixture } from '../fixtures/layout.js';
import { evaluatePose } from '../../src/model/pose.js';
import { DEFAULT_LINK_CLEARANCE_MM } from '../../src/contracts.js';
import { computeWorkspace } from '../../src/workspace/sweep.js';
import { computeCycleDemand } from '../../src/model/cycle.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { selectBest, exportResult } from '../../src/io/results.js';
import { rotationMatrixFromEuler, rotateVector } from '../../src/math.js';
import { assessJacobian, rotaryActuatorJacobian, translationSingularSystem,
  NUMERICAL_RECIPROCAL_CUTOFF } from '../../src/model/conditioning.js';

const close = (actual, expected, tolerance = 1e-6) =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
const mechanical = { ballJointLimitDeg: 180 };

test('rotary Jacobian matches servo-angle finite differences about the anchor centroid', () => {
  const layout = asymmetricJointFixture();
  const home = evaluatePose(layout, {}, mechanical);
  assert.equal(home.reachable, true);
  const rows = home.jacobianRows;
  const radius = home.conditioning.radius;
  const centroidLocal = [0, 1, 2].map(axis => layout.platformAnchors
    .reduce((sum, anchor) => sum + anchor[axis], 0) / 6);
  const epsilon = 1e-6;
  for (let column = 0; column < 6; column++) {
    const poseAt = sign => {
      if (column < 3) return { [['x', 'y', 'z'][column]]: sign * epsilon * radius };
      const angleName = ['rx', 'ry', 'rz'][column - 3];
      const rotation = { [angleName]: sign * epsilon };
      const rotatedCentroid = rotateVector(rotationMatrixFromEuler(
        rotation.rx ?? 0, rotation.ry ?? 0, rotation.rz ?? 0), centroidLocal);
      return { ...rotation, x: centroidLocal[0] - rotatedCentroid[0],
        y: centroidLocal[1] - rotatedCentroid[1],
        z: centroidLocal[2] - rotatedCentroid[2] };
    };
    const before = evaluatePose(layout, poseAt(-1), mechanical);
    const after = evaluatePose(layout, poseAt(1), mechanical);
    assert.ok(before.reachable && after.reachable);
    for (let leg = 0; leg < 6; leg++) {
      const derivative = (after.servoAngles[leg] - before.servoAngles[leg]) / (2 * epsilon);
      close(rows[leg][column], derivative, 2e-6);
    }
  }
  // Translation uses RMS radius, rotation uses the current anchor centroid.
  close(radius, Math.sqrt(layout.platformAnchors.reduce((sum, q) => sum
    + q.reduce((s, value, axis) => s + (value - centroidLocal[axis]) ** 2, 0), 0) / 6));
});

test('Jacobian and condition are dimensionless under a uniform length scale', () => {
  const original = asymmetricJointFixture();
  const baseline = evaluatePose(original, { x: 3, z: 5, ry: 0.02 }, mechanical);
  assert.equal(baseline.reachable, true);
  for (const factor of [0.1, 10, 1000]) {
    const scaled = structuredClone(original);
    scaled.baseAnchors = scaled.baseAnchors.map(point => point.map(value => value * factor));
    scaled.platformAnchors = scaled.platformAnchors.map(point => point.map(value => value * factor));
    scaled.hornLength *= factor;
    scaled.rodLength *= factor;
    scaled.homeHeight *= factor;
    // The link clearance is a length too, so it scales with the layout.
    const current = evaluatePose(scaled, { x: 3 * factor, z: 5 * factor, ry: 0.02 },
      { ...mechanical, linkClearanceMm: DEFAULT_LINK_CLEARANCE_MM * factor });
    assert.equal(current.reachable, true);
    close(current.conditioning.reciprocal, baseline.conditioning.reciprocal, 1e-11);
    for (let row = 0; row < 6; row++) for (let col = 0; col < 6; col++) {
      close(current.jacobianRows[row][col], baseline.jacobianRows[row][col], 1e-10);
    }
  }
});

test('one-sided SVD preserves zero modes and applies the inclusive 1e-10 cutoff', () => {
  const diagonal = last => Array.from({ length: 6 }, (_, row) =>
    Array.from({ length: 6 }, (_, col) => row === col ? (row === 5 ? last : 1) : 0));
  for (const [last, rejected] of [[1e-8, false], [1.1e-10, false],
    [1e-10, true], [0.9e-10, true], [0, true]]) {
    const result = assessJacobian(diagonal(last));
    close(result.reciprocal, last, 1e-24);
    assert.equal(result.numericalSingularity, rejected);
    assert.equal(result.singularValues.length, 6);
  }
  const rotatedMode = last => {
    const matrix = diagonal(1);
    matrix[0][0] = matrix[1][1] = (1 + last) / 2;
    matrix[0][1] = matrix[1][0] = (1 - last) / 2;
    return matrix;
  };
  assert.equal(assessJacobian(rotatedMode(1.1e-10)).numericalSingularity, false);
  assert.equal(assessJacobian(rotatedMode(1e-10)).numericalSingularity, true);
  assert.equal(assessJacobian(rotatedMode(0)).numericalSingularity, true);
  assert.equal(NUMERICAL_RECIPROCAL_CUTOFF, 1e-10);
  const rankDeficient = evaluatePose(jointFixture(), {}, mechanical);
  assert.equal(rankDeficient.geometricallyReachable, true);
  assert.equal(rankDeficient.mechanicallyReachable, true);
  assert.equal(rankDeficient.reachable, false);
  assert.equal(rankDeficient.violations.at(-1).type, 'numericalSingularity');
});

test('degenerate servo leverage and anchor radius are explicit unavailable failures', () => {
  const points = Array.from({ length: 6 }, (_, i) => [Math.cos(i), Math.sin(i), 0]);
  const horizontalRods = Array.from({ length: 6 }, () => [1, 0, 0]);
  const angles = Array(6).fill(0), betas = Array(6).fill(0);
  assert.equal(rotaryActuatorJacobian(points, horizontalRods, angles, betas, 50).reason,
    'servoLeverage');
  assert.equal(rotaryActuatorJacobian(Array.from({ length: 6 }, () => [0, 0, 0]),
    horizontalRods, angles, betas, 50).reason, 'anchorRadius');
  assert.equal(assessJacobian(Array.from({ length: 6 }, () => Array(6).fill(NaN))).available,
    false);
});

test('optional engineering condition limit is inclusive and consistent at home, workspace, and cycle', async () => {
  const layout = asymmetricJointFixture();
  const home = evaluatePose(layout, {}, mechanical);
  const equal = home.conditioning.condition;
  assert.equal(evaluatePose(layout, {}, { ...mechanical, conditionLimit: equal }).reachable, true);
  const failing = evaluatePose(layout, {}, { ...mechanical, conditionLimit: equal - 1 });
  assert.equal(failing.reachable, false);
  assert.equal(failing.violations.at(-1).type, 'conditionLimit');
  const workspaceAt = await computeWorkspace(layout, {}, { ...mechanical, conditionLimit: equal });
  const workspaceBelow = await computeWorkspace(layout, {}, { ...mechanical, conditionLimit: equal - 1 });
  assert.equal(workspaceAt.coverage, 100);
  assert.equal(workspaceBelow.coverage, 0);
  assert.equal(workspaceBelow.relaxedCoverage, 0);
  const cycleAt = computeCycleDemand(layout, { ...mechanical, conditionLimit: equal, mass: 1 });
  const cycleBelow = computeCycleDemand(layout, { ...mechanical, conditionLimit: equal - 1, mass: 1 });
  assert.equal(cycleAt.valid, true);
  assert.equal(cycleBelow.valid, false);
  assert.equal(cycleBelow.violations.at(-1).type, 'conditionLimit');
  const workspaceAway = await computeWorkspace(layout, { z: { min: 0, max: 10, step: 10 } },
    { ...mechanical, conditionLimit: 75 });
  assert.equal(workspaceAway.coverage, 50);
  assert.equal(workspaceAway.stats.conditioningCounts.engineeringLimit, 1);
  const cycleAway = computeCycleDemand(layout, { ...mechanical, conditionLimit: 75,
    mass: 1, stroke: 20, frequency: 1 });
  assert.equal(cycleAway.valid, false);
  assert.equal(cycleAway.violations.at(-1).type, 'conditionLimit');
  for (const invalid of [0, -1, Infinity, NaN]) {
    assert.throws(() => new Optimizer({}, { conditionLimit: invalid }), /conditionLimit/);
  }
});

test('worst reciprocal quality and conditioning failures remain visible in export and ranking', async () => {
  const layout = asymmetricJointFixture();
  const ranges = { z: { min: -10, max: 10, step: 10 } };
  const opt = new Optimizer({}, { ranges, sampling: { strategy: 'grid' },
    ballJointLimitDeg: 180, conditionLimit: 75 });
  const diagnostic = await opt.evaluateLayout(layout);
  close(diagnostic.coverage, 200 / 3);
  assert.equal(diagnostic.feasibility.conditionSatisfied, false);
  assert.equal(diagnostic.workspace.stats.conditioningCounts.engineeringLimit, 1);
  const allowed = [0, -10].map(z => evaluatePose(layout, { z }, mechanical).conditioning.reciprocal);
  close(diagnostic.conditioningQuality, Math.min(...allowed), 1e-12);
  const exported = exportResult(diagnostic, { status: 'completed' });
  assert.equal(exported.metadata.conditioning_quality, diagnostic.conditioningQuality);
  assert.equal(exported.constraint_policy.conditionLimit, 75);
  assert.ok(exported.feasibility.failedCategories.includes('conditioning'));
  assert.ok(exported.feasibility.failedCategories.includes('workspace'));
  assert.equal(exported.workspace_stats.conditioningCounts.engineeringLimit, 1);
  const passing = await new Optimizer({}, { ranges, sampling: { strategy: 'grid' },
    ballJointLimitDeg: 180,
    conditionLimit: 80 }).evaluateLayout(asymmetricJointFixture());
  assert.equal(passing.feasibility.conditionSatisfied, true);
  assert.equal(passing.coverage, 100);
  close(passing.conditioningQuality,
    Math.min(...[-10, 0, 10].map(z => evaluatePose(layout, { z }, mechanical).conditioning.reciprocal)), 1e-12);
  assert.equal(selectBest([diagnostic], [diagnostic, passing]), passing);
});

test('invalid-home candidates remain diagnostic with unavailable quality', async () => {
  const evaluation = await new Optimizer({}, { ballJointLimitDeg: 180,
    sampling: { strategy: 'grid' } })
    .evaluateLayout(jointFixture());
  assert.equal(evaluation.homePose.geometricallyReachable, true);
  assert.equal(evaluation.homePose.reachable, false);
  assert.equal(evaluation.conditioningQuality, null);
  assert.equal(evaluation.feasibility.passing, false);
  assert.ok(evaluation.feasibility.failedCategories.includes('home'));
  assert.ok(evaluation.feasibility.failedCategories.includes('conditioning'));
  const exported = exportResult(evaluation, { status: 'completed' });
  assert.equal(exported.metadata.conditioning_quality, null);
  assert.equal(exported.diagnostic, true);
  assert.equal(exported.workspace_stats.conditioningCounts.numericalSingularity, 1);
});

// A 6x6 Jacobian whose translation block is U * diag(values) * V^T, with U the
// first three unit columns and V the rows of `basis`; rotation columns are 1.
const withTranslationBlock = (values, basis) => Array.from({ length: 6 }, (_, row) =>
  [...[0, 1, 2].map(col => row < 3 ? values[row] * basis[row][col] : 0), 1, 1, 1]);

test('translation singular system recovers the values and world directions of the translation block', () => {
  const c = Math.cos(0.4), s = Math.sin(0.4);
  const basis = [[c, s, 0], [-s, c, 0], [0, 0, 1]];
  const system = translationSingularSystem(withTranslationBlock([0.5, 3, 1e-6], basis));
  assert.deepEqual(system.map(entry => entry.value).map(value => Number(value.toPrecision(12))), [3, 0.5, 1e-6]);
  // Largest first, each with its own direction (sign is arbitrary).
  for (const [entry, direction] of [[system[0], basis[1]], [system[1], basis[0]], [system[2], basis[2]]]) {
    close(Math.abs(entry.vector.reduce((sum, value, k) => sum + value * direction[k], 0)), 1, 1e-12);
  }
  const layout = asymmetricJointFixture();
  const rows = evaluatePose(layout, { x: 5, ry: 0.03 }, mechanical).jacobianRows;
  for (const { value, vector } of translationSingularSystem(rows)) {
    // J_t^T J_t v = sigma^2 v for the first three Jacobian columns.
    const jv = rows.map(row => row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2]);
    const jtjv = [0, 1, 2].map(col => rows.reduce((sum, row, k) => sum + row[col] * jv[k], 0));
    jtjv.forEach((component, k) => close(component, value * value * vector[k], 1e-9));
    close(Math.hypot(...vector), 1, 1e-12);
  }
  assert.equal(translationSingularSystem(null), null);
  assert.equal(translationSingularSystem([[1, 2, 3]]), null);
  assert.equal(translationSingularSystem(Array.from({ length: 6 }, () => Array(6).fill(0))), null);
  assert.equal(translationSingularSystem(Array.from({ length: 6 }, () => [NaN, 0, 0, 0, 0, 0])), null);
});

test('a symmetric layout at home gives equal horizontal axes about a vertical one', () => {
  // The D3-symmetric paired layout: any horizontal direction couples equally,
  // while the near-vertical rods couple more strongly to vertical motion.
  const home = evaluatePose(pairedFixture(), {}, mechanical);
  const [vertical, ...horizontal] = translationSingularSystem(home.jacobianRows);
  close(Math.abs(vertical.vector[2]), 1, 1e-12);
  close(horizontal[0].value, horizontal[1].value, 1e-12);
  for (const { vector } of horizontal) close(vector[2], 0, 1e-12);
});

test('the translation helper leaves pose conditioning and its condition number unchanged', () => {
  // Frozen from before the helper existed: the pose conditioning outputs, which
  // feed optimizer results, must not move. They come from the earlier paired
  // fixture; the link check is off because conditioning does not depend on it.
  const layout = crossedPairedFixture();
  const home = evaluatePose(layout, {}, { ...mechanical, linkClearanceMm: 0 });
  const rows = structuredClone(home.jacobianRows);
  translationSingularSystem(home.jacobianRows);
  assert.deepEqual(home.jacobianRows, rows, 'the rows are not modified');
  assert.equal(home.conditioning.reciprocal, 0.3514923980989354);
  assert.equal(home.conditioning.condition, 2.8450117425257306);
  assert.deepEqual(home.conditioning.singularValues, [2.285374541057664, 1.6226197168762975,
    1.6226197168762972, 1.1406769909749306, 0.8032917779906122, 0.8032917779906122]);
  assert.deepEqual(assessJacobian(rows).singularValues, home.conditioning.singularValues);
});
