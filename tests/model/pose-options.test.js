import test from 'node:test';
import assert from 'node:assert/strict';
import { evaluatePose } from '../../src/model/pose.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';

test('evaluatePose rejects wrongly typed limits, servo bounds and mounting by field name instead of coercing', () => {
  const layout = asymmetricJointFixture();
  const cases = [
    [{ ballJointLimitDeg: null }, /ballJointLimitDeg must be a finite angle from 0 to 180 degrees/],
    [{ ballJointLimitDeg: '45' }, /ballJointLimitDeg must be a finite angle/],
    [{ ballJointLimitDeg: true }, /ballJointLimitDeg must be a finite angle/],
    [{ ballJointLimitDeg: [] }, /ballJointLimitDeg must be a finite angle/],
    [{ lowerBallJointLimitDeg: null }, /lowerBallJointLimitDeg must be a finite angle/],
    [{ upperBallJointLimitDeg: '10' }, /upperBallJointLimitDeg must be a finite angle/],
    [{ servoRangeRad: 'abc' }, /servoRangeRad must contain two finite bounds in radians with max >= min/],
    [{ servoRangeRad: NaN }, /servoRangeRad must contain two finite bounds/],
    [{ servoRangeRad: true }, /servoRangeRad must contain two finite bounds/],
    [{ servoRangeRad: {} }, /servoRangeRad must contain two finite bounds/],
    [{ servoRangeRad: [] }, /servoRangeRad must contain two finite bounds/],
    [{ servoRangeRad: -5 }, /servoRangeRad must contain two finite bounds/],
    [{ servoRangeRad: null }, /servoRangeRad must contain two finite bounds/],
    [{ servoRangeRad: [0.5, -0.5] }, /servoRangeRad must contain two finite bounds/],
    [{ rodLengthTolerance: '0.5' }, /rodLengthTolerance must be a finite nonnegative/],
    [{ mounting: 'x' }, /mounting must provide six lower and six upper socket directions/],
    [{ mounting: { lower: [], upper: [] } }, /mounting must provide six lower and six upper/],
  ];
  for (const [options, expected] of cases) {
    assert.throws(() => evaluatePose(layout, { rx: 0.03 }, { ballJointLimitDeg: 180, ...options }), expected,
      JSON.stringify(options));
  }
  assert.throws(() => evaluatePose(layout, {}, 'abc'), /Pose options must be an object/);
  assert.throws(() => evaluatePose(layout, {}, null), /Pose options must be an object/);
  // A locked range is allowed, as requirements accept it.
  assert.equal(evaluatePose(layout, {}, { ballJointLimitDeg: 180, servoRangeRad: [0.2, 0.2] }).violations[0].type, 'servoLimit');
  // A pose outside servo travel is a servoLimit violation once the bounds are valid.
  const tight = evaluatePose(layout, { z: 30 }, { ballJointLimitDeg: 180, servoRangeRad: [-0.01, 0.01] });
  assert.equal(tight.reachable, false);
  assert.equal(tight.violations[0].type, 'servoLimit');
});

test('servo travel bounds are inclusive at the boundary with a 1e-6 rad tolerance and no more', () => {
  const layout = asymmetricJointFixture();
  const pose = { y: 6, rx: 0.05 };
  const check = servoRangeRad => evaluatePose(layout, pose, { ballJointLimitDeg: 180, servoRangeRad });
  const angles = check([-Math.PI, Math.PI]).servoAngles;
  const low = Math.min(...angles), high = Math.max(...angles);
  const lowLeg = angles.indexOf(low), highLeg = angles.indexOf(high);
  assert.ok(high - low > 0.01 && lowLeg !== highLeg);
  // Servo angles exactly at both bounds are accepted.
  const exact = check([low, high]);
  assert.equal(exact.reachable, true);
  assert.deepEqual(exact.violations, []);
  assert.deepEqual(exact.servoAngles, angles);
  // Inside the 1e-6 rad tolerance the pose is still accepted.
  assert.equal(check([low + 5e-7, high - 5e-7]).reachable, true);
  // Just beyond the tolerance each bound rejects its own leg, reporting the angle.
  const belowMin = check([low + 2e-6, high]);
  assert.equal(belowMin.reachable, false);
  assert.equal(belowMin.geometricallyReachable, false);
  assert.deepEqual(belowMin.violations, [{ type: 'servoLimit', leg: lowLeg, value: low }]);
  const aboveMax = check([low, high - 2e-6]);
  assert.equal(aboveMax.reachable, false);
  assert.deepEqual(aboveMax.violations, [{ type: 'servoLimit', leg: highLeg, value: high }]);
  // The tolerance is 1e-6 rad; a bound 1e-4 rad inside the angle is a real violation.
  assert.equal(check([low + 1e-4, high]).violations[0].type, 'servoLimit');
  assert.equal(check([low, high - 1e-4]).violations[0].type, 'servoLimit');
});
