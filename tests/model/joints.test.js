import test from 'node:test';
import assert from 'node:assert/strict';
import { jointFixture, asymmetricJointFixture } from '../fixtures/layout.js';
import { evaluatePose } from '../../src/model/pose.js';
import { computeWorkspace } from '../../src/workspace/sweep.js';
import { computeCycleDemand } from '../../src/model/cycle.js';
import { hornLocalToWorld, worldToHornLocal } from '../../src/model/kinematics.js';
import { resolveMounting } from '../../src/model/mounting.js';

const close = (a, b, tolerance = 1e-6) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);

function withFlippedSocket(joint) {
  const layout = asymmetricJointFixture();
  const mounting = resolveMounting(layout).mounting;
  mounting[joint][0] = { source: 'supplied',
    direction: mounting[joint][0].direction.map(value => -value) };
  layout.mounting = mounting;
  return layout;
}

test('both home sockets point toward their opposite rod endpoints', () => {
  const layout = jointFixture();
  const result = evaluatePose(layout, {}, { ballJointLimitDeg: 0, recordLegData: true });
  assert.equal(result.mechanicallyReachable, true);
  assert.equal(result.jointAngles.lower.length, 6);
  for (let leg = 0; leg < 6; leg++) {
    close(result.jointAngles.lower[leg], 0);
    close(result.jointAngles.upper[leg], 0);
    assert.equal(result.mounting.lower[leg].source, 'derived');
    assert.equal(result.mounting.upper[leg].source, 'derived');
  }
});

test('moving horn and platform rotations follow their local mounting frames', () => {
  const radial = hornLocalToWorld(0, Math.PI / 2, [1, 0, 0]);
  const vertical = hornLocalToWorld(0, Math.PI / 2, [0, 0, 1]);
  radial.forEach((v, i) => close(v, [0, 0, 1][i]));
  vertical.forEach((v, i) => close(v, [-1, 0, 0][i]));
  worldToHornLocal(0, Math.PI / 2, [0, 0, 1]).forEach((v, i) => close(v, [1, 0, 0][i]));

  const layout = jointFixture();
  const heave = evaluatePose(layout, { z: 10 }, { ballJointLimitDeg: 180 });
  close(heave.jointAngles.lower[0], 0.206539, 1e-6);
  close(heave.jointAngles.upper[0], 0.017264, 1e-6);
  const tilted = evaluatePose(layout, { x: 10, ry: 0.1, rz: 0.05 }, { ballJointLimitDeg: 180 });
  close(tilted.jointAngles.lower[0], 0.150203, 1e-6);
  close(tilted.jointAngles.upper[0], 0.060594, 1e-6);
  assert.ok(tilted.jointAngles.lower[0] > tilted.jointAngles.upper[0]);
});

test('upper and lower failures are independently reported with leg, joint, and limit', async () => {
  for (const joint of ['lower', 'upper']) {
    const layout = withFlippedSocket(joint);
    const home = evaluatePose(layout, {}, { ballJointLimitDeg: 30 });
    assert.equal(home.reachable, false);
    assert.deepEqual(home.violations.map(v => [v.type, v.leg, v.joint]), [['ballJoint', 0, joint]]);
    close(home.violations[0].value, Math.PI);
    close(home.violations[0].limit, Math.PI / 6);
    const workspace = await computeWorkspace(layout, {}, { ballJointLimitDeg: 30, ballJointClamp: true });
    assert.equal(workspace.coverage, 0);
    assert.equal(workspace.relaxedCoverage, 100);
    assert.equal(workspace.stats.jointViolationCounts[joint], 1);
    const cycle = computeCycleDemand(layout, { ballJointLimitDeg: 30, mass: 1 });
    assert.equal(cycle.valid, false);
    assert.deepEqual(cycle.violations.map(v => [v.leg, v.joint]), [[0, joint]]);
  }
});

test('joint-angle limit is inclusive at the boundary and independent per socket', () => {
  const layout = asymmetricJointFixture();
  const mounting = resolveMounting(layout).mounting;
  const direction = mounting.lower[0].direction;
  const perpendicular = [-direction[1], direction[0], 0];
  const length = Math.hypot(...perpendicular);
  const unit = perpendicular.map(v => v / length);
  const angle = Math.PI / 6;
  mounting.lower[0] = { source: 'supplied', direction: direction.map((v, i) =>
    v * Math.cos(angle) + unit[i] * Math.sin(angle)) };
  layout.mounting = mounting;
  const atLimit = evaluatePose(layout, {}, { lowerBallJointLimitDeg: 30, upperBallJointLimitDeg: 0 });
  assert.equal(atLimit.reachable, true);
  close(atLimit.jointAngles.lower[0], angle);
  const belowLimit = evaluatePose(layout, {}, { lowerBallJointLimitDeg: 29.99, upperBallJointLimitDeg: 0 });
  assert.deepEqual(belowLimit.violations.map(v => v.joint), ['lower']);
});

test('effective per-socket limits carry through workspace and cycle checks', async () => {
  const layout = asymmetricJointFixture();
  const mounting = resolveMounting(layout).mounting;
  mounting.lower[0] = { source: 'supplied', direction: [1, 0, 0] };
  layout.mounting = mounting;
  const home = evaluatePose(layout, {}, { ballJointLimitDeg: 180 });
  const measured = home.jointAngles.lower[0] * 180 / Math.PI;
  const options = { ballJointLimitDeg: 180, lowerBallJointLimitDeg: measured - 1,
    upperBallJointLimitDeg: 180 };
  const failedHome = evaluatePose(layout, {}, options);
  assert.deepEqual(failedHome.violations.map(v => [v.leg, v.joint]), [[0, 'lower']]);
  const workspace = await computeWorkspace(layout, {}, options);
  assert.equal(workspace.constraintPolicy.lowerBallJointLimitDeg, measured - 1);
  assert.equal(workspace.coverage, 0);
  const cycle = computeCycleDemand(layout, { ...options, mass: 1 });
  assert.equal(cycle.valid, false);
  assert.deepEqual(cycle.violations.map(v => [v.leg, v.joint]), [[0, 'lower']]);
});

test('legacy mounting upgrade preserves geometry and rederives after home edits', () => {
  const layout = jointFixture();
  layout.model_version = 1;
  const geometryBefore = JSON.stringify(layout);
  const resolved = resolveMounting(layout, { imported: true });
  assert.equal(JSON.stringify(layout), geometryBefore);
  assert.equal(resolved.migration.upgraded, true);
  assert.match(resolved.migration.note, /stale.*recalculated/);
  assert.ok(resolved.mounting.lower.every(entry => entry.source === 'derived'));
  const first = resolved.mounting.lower[0].direction;
  layout.mounting = resolved.mounting;
  layout.homeHeight += 10;
  const moved = resolveMounting(layout).mounting;
  assert.notDeepEqual(moved.lower[0].direction, first);
  layout.mounting.lower[0] = { source: 'supplied', direction: [1, 0, 0] };
  assert.deepEqual(resolveMounting(layout).mounting.lower[0], { source: 'supplied', direction: [1, 0, 0] });
});

test('malformed or zero mounting overrides are rejected', () => {
  const layout = jointFixture();
  layout.mounting = { lower: [[0, 0, 0]] };
  assert.throws(() => resolveMounting(layout), /six leg entries/);
  layout.mounting.lower = Array.from({ length: 6 }, () => null);
  layout.mounting.lower[2] = [0, 0, 0];
  assert.throws(() => resolveMounting(layout), /nonzero 3D direction/);
});
