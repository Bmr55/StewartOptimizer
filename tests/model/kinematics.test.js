import test from 'node:test';
import assert from 'node:assert/strict';
import { solveServoAngle, computeHornTip, hornFrameAxes, hornLocalToWorld, worldToHornLocal } from '../../src/model/kinematics.js';
import { evaluatePose } from '../../src/model/pose.js';
import { resolveMounting } from '../../src/model/mounting.js';
import { vectorCross, vectorDot, vectorNormalize } from '../../src/math.js';
import { jointFixture } from '../fixtures/layout.js';

const close = (a, b, tolerance = 1e-6) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);

// The horn frame as JOINT_MODEL.md documents it: X follows the horn, Y is tangential
// to the servo plane, Z completes the right-handed basis.
function documentedHornToWorld(beta, alpha, [dx, dy, dz]) {
  const X = [Math.cos(alpha) * Math.cos(beta), Math.cos(alpha) * Math.sin(beta), Math.sin(alpha)];
  const Y = [-Math.sin(beta), Math.cos(beta), 0];
  const Z = [-Math.sin(alpha) * Math.cos(beta), -Math.sin(alpha) * Math.sin(beta), Math.cos(alpha)];
  return [0, 1, 2].map(k => X[k] * dx + Y[k] * dy + Z[k] * dz);
}

const sixSockets = (leg, direction) => Array.from({ length: 6 }, (_, i) => (i === leg ? direction : null));

test('the horn frame is right-handed and a tangential socket component maps to the documented world direction', () => {
  const beta = 0.7, alpha = 0.3;
  const [X, Y, Z] = hornFrameAxes(beta, alpha);
  assert.deepEqual(Y, [-Math.sin(beta), Math.cos(beta), 0]);
  vectorCross(X, Y).forEach((value, k) => close(value, Z[k], 1e-12));
  close(vectorDot(X, Y), 0, 1e-12);
  const local = [0.2, 0.9, 0.4];
  const world = hornLocalToWorld(beta, alpha, local);
  const expected = documentedHornToWorld(beta, alpha, local);
  world.forEach((value, k) => close(value, expected[k], 1e-12));
  // Hand-evaluated: cos 0.7 = 0.764842, sin 0.7 = 0.644218, cos 0.3 = 0.955336, sin 0.3 = 0.295520.
  [-0.524072, 0.735295, 0.441238].forEach((value, k) => close(world[k], value, 1e-5));
  // A mirrored Y is a different direction, and the local map inverts the world map.
  const mirrored = hornLocalToWorld(beta, alpha, [0.2, -0.9, 0.4]);
  assert.ok(Math.hypot(...world.map((value, k) => value - mirrored[k])) > 1);
  worldToHornLocal(beta, alpha, world).forEach((value, k) => close(value, local[k], 1e-12));
});

test('a supplied lower socket with a tangential component deflects by the documented world-frame angle', () => {
  const supplied = [0.6, 0.64, 0.48];
  const pose = { x: 4, ry: 0.05 };
  const layout = jointFixture();
  layout.mounting = { lower: sixSockets(0, supplied), upper: sixSockets(-1) };
  const result = evaluatePose(layout, pose, { ballJointLimitDeg: 180, recordLegData: true });
  assert.equal(result.reachable, true);
  const rod = vectorNormalize(result.rodVectors[0]);
  const world = documentedHornToWorld(layout.betaAngles[0], result.servoAngles[0], supplied);
  const expected = Math.acos(vectorDot(world, rod));
  assert.ok(expected > 0.05 && expected < 3);
  close(result.jointAngles.lower[0], expected, 1e-12);
  assert.ok(Math.abs(vectorDot(rod, [-Math.sin(layout.betaAngles[0]), Math.cos(layout.betaAngles[0]), 0])) > 0.1,
    'the rod has a tangential component, so the sign of the Y axis is observable');
  const mirroredLayout = jointFixture();
  mirroredLayout.mounting = { lower: sixSockets(0, [0.6, -0.64, 0.48]), upper: sixSockets(-1) };
  const mirrored = evaluatePose(mirroredLayout, pose, { ballJointLimitDeg: 180 });
  assert.ok(Math.abs(mirrored.jointAngles.lower[0] - result.jointAngles.lower[0]) > 0.05);
  // The other legs keep derived sockets; leg 0's upper socket is unaffected by the lower override.
  const derived = evaluatePose(jointFixture(), pose, { ballJointLimitDeg: 180 });
  assert.deepEqual(result.jointAngles.upper, derived.jointAngles.upper);
  assert.deepEqual(result.jointAngles.lower.slice(1), derived.jointAngles.lower.slice(1));
});

test('a non-unit supplied socket direction is normalized and deflects exactly like its unit version', () => {
  const raw = [1.2, 1.8, 0.9];
  const unit = vectorNormalize(raw);
  const build = direction => {
    const layout = jointFixture();
    layout.mounting = { lower: sixSockets(0, direction), upper: sixSockets(2, { direction, source: 'supplied' }) };
    return layout;
  };
  const resolved = resolveMounting(build(raw)).mounting;
  for (const entry of [resolved.lower[0], resolved.upper[2]]) {
    assert.equal(entry.source, 'supplied');
    close(Math.hypot(...entry.direction), 1, 1e-12);
    entry.direction.forEach((value, k) => close(value, unit[k], 1e-12));
  }
  assert.deepEqual(resolved.lower[1], resolveMounting(jointFixture()).mounting.lower[1]);
  const pose = { z: 5, rx: 0.04 };
  const fromUnit = evaluatePose(build(unit), pose, { ballJointLimitDeg: 180 });
  assert.ok(fromUnit.jointAngles.lower[0] > 0.1 && fromUnit.jointAngles.upper[2] > 0.1);
  for (const scale of [1, 1e3, 1e-6]) {
    const fromScaled = evaluatePose(build(raw.map(value => value * scale)), pose, { ballJointLimitDeg: 180 });
    assert.equal(fromScaled.reachable, true);
    close(fromScaled.jointAngles.lower[0], fromUnit.jointAngles.lower[0], 1e-9);
    close(fromScaled.jointAngles.upper[2], fromUnit.jointAngles.upper[2], 1e-9);
  }
});

test('solveServoAngle wraps the horn angle into (-pi, pi] below the servo plane', () => {
  const previous = [];
  for (const dx of [1e-3, 0, -1e-3]) {
    const q = [dx, 0, -150];
    const { alpha } = solveServoAngle([0, 0, 0], q, 50, 200, 0);
    assert.ok(alpha > -Math.PI && alpha <= Math.PI, `alpha ${alpha} not wrapped`);
    close(alpha, Math.PI / 2, 1e-3);
    const tip = computeHornTip([0, 0, 0], 50, 0, alpha);
    close(Math.hypot(q[0] - tip[0], q[1] - tip[1], q[2] - tip[2]), 200);
    // No 2*pi discontinuity as f crosses zero.
    if (previous.length) close(alpha, previous.at(-1), 1e-3);
    previous.push(alpha);
  }
});

test('a pose with anchors below the servo pivots is not rejected as a servo-limit violation', () => {
  const layout = jointFixture();
  // Platform anchors directly below their base anchors: every leg solves with
  // f = 0 and e < 0, the case that used to report alpha = -270 degrees.
  layout.platformAnchors = layout.baseAnchors.map(anchor => [...anchor]);
  layout.homeHeight = 0;
  const result = evaluatePose(layout, { z: -150 }, { ballJointLimitDeg: 180, conditionLimit: null });
  assert.deepEqual(result.violations.filter(v => v.type === 'servoLimit'), []);
  for (const alpha of result.servoAngles) close(alpha, Math.PI / 2, 1e-6);
  const narrow = evaluatePose(layout, { z: -150 },
    { ballJointLimitDeg: 180, servoRangeRad: [-2 * Math.PI / 3, 2 * Math.PI / 3] });
  assert.deepEqual(narrow.violations.filter(v => v.type === 'servoLimit'), []);
});
