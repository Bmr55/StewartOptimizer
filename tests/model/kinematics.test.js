import test from 'node:test';
import assert from 'node:assert/strict';
import { solveServoAngle, computeHornTip } from '../../src/model/kinematics.js';
import { evaluatePose } from '../../src/model/pose.js';
import { jointFixture } from '../fixtures/layout.js';

const close = (a, b, tolerance = 1e-6) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);

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
