import test from 'node:test';
import assert from 'node:assert/strict';
import { evaluatePose } from '../../src/model/pose.js';
import { asymmetricJointFixture, jointFixture, pairedFixture } from '../fixtures/layout.js';
import { hornTip, norm, platformPoint, servoAngleRoots, sub } from '../fixtures/independent-geometry.js';

// The pose evaluator's inverse kinematics checked against answers that do not
// come from its closed form: a hand-solved layout, and for general poses an
// independent platform transform, horn-tip formula and numerical root search.

const OPTIONS = { ballJointLimitDeg: 180, servoRangeRad: [-Math.PI, Math.PI], recordLegData: true };
const close = (actual, expected, tolerance, label) =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${label}: ${actual} != ${expected}`);
const closeVector = (actual, expected, tolerance, label) =>
  actual.forEach((value, k) => close(value, expected[k], tolerance, `${label}[${k}]`));

// Six identical legs every 60 degrees. Each horn points radially outward from a
// base anchor at radius 100 mm, and each platform anchor sits 80 mm further out.
// With a 25 mm horn, a 100 mm rod and a 95 mm home height, every leg is the
// same 3-4-5 triangle in its own vertical plane. (All rods lie in radial planes,
// so yaw is singular; only the geometry is checked here, not reachability.)
function radialLayout() {
  const at = (radius, k) => [radius * Math.cos(k * Math.PI / 3), radius * Math.sin(k * Math.PI / 3), 0];
  return { baseAnchors: [0, 1, 2, 3, 4, 5].map(k => at(100, k)), platformAnchors: [0, 1, 2, 3, 4, 5].map(k => at(180, k)),
    betaAngles: [0, 1, 2, 3, 4, 5].map(k => k * Math.PI / 3), hornLength: 25, rodLength: 100, homeHeight: 95,
    servoRangeRad: [-Math.PI, Math.PI] };
}

test('hand-solved 3-4-5 legs: home and a 30 mm drop give servo angles of +/-asin(3/5)', () => {
  // Home: relative to its base anchor the platform anchor is 80 mm out and 95 mm
  // up. A horn at asin(3/5) puts its tip 20 mm out and 15 mm up, leaving a rod
  // span of (60, 80): 100 mm. Dropping 30 mm with the horn at -asin(3/5) puts the
  // tip 20 mm out and 15 mm down, leaving (60, 65 + 15) = (60, 80) again.
  const layout = radialLayout();
  for (const [z, alpha, tipRise] of [[0, Math.asin(0.6), 15], [-30, -Math.asin(0.6), -15]]) {
    const result = evaluatePose(layout, { z }, OPTIONS);
    assert.equal(result.geometricallyReachable, true, `z ${z}`);
    for (let leg = 0; leg < 6; leg++) {
      const radial = [Math.cos(leg * Math.PI / 3), Math.sin(leg * Math.PI / 3)];
      close(result.servoAngles[leg], alpha, 1e-12, `z ${z} leg ${leg} servo angle`);
      closeVector(result.hornTips[leg], [120 * radial[0], 120 * radial[1], tipRise], 1e-9, `z ${z} leg ${leg} horn tip`);
      closeVector(result.platformPoints[leg], [180 * radial[0], 180 * radial[1], 95 + z], 1e-9, `z ${z} leg ${leg} platform point`);
      closeVector(result.rodVectors[leg], [60 * radial[0], 60 * radial[1], 80], 1e-9, `z ${z} leg ${leg} rod`);
      close(result.rodLengths[leg], 100, 1e-9, `z ${z} leg ${leg} rod length`);
    }
  }
  // The other root of the home triangle, about 62.9 degrees, also closes the
  // rod; the evaluator keeps the branch where raising the horn shortens the gap.
  const roots = servoAngleRoots(layout.baseAnchors[0], 0, 25, 100, [180, 0, 95]);
  assert.equal(roots.length, 2);
  close(roots.find(root => root.slope < 0).alpha, Math.asin(0.6), 1e-9, 'kept root');
  close(roots.find(root => root.slope > 0).alpha, Math.atan2(95, 80) + Math.acos(121 / Math.hypot(80, 95)), 1e-9, 'other root');
});

// Deterministic poses across the sample workspace (+/-40 mm X/Y, -20 to 40 mm Z)
// with tilts up to 12 degrees and yaw up to 20 degrees. About a fifth of them
// cannot be closed, which exercises the rejection side too.
function poses(count, seed) {
  const random = () => (seed = (seed * 48271) % 2147483647) / 2147483647;
  const span = (min, max) => min + (max - min) * random();
  const deg = Math.PI / 180;
  return Array.from({ length: count }, () => ({ x: span(-40, 40), y: span(-40, 40), z: span(-20, 40),
    rx: span(-12, 12) * deg, ry: span(-12, 12) * deg, rz: span(-20, 20) * deg }));
}

test('general poses: platform points, horn tips and rod closure match an independent solution', () => {
  for (const [name, layout] of [['symmetric', jointFixture()], ['asymmetric', asymmetricJointFixture()], ['paired', pairedFixture()]]) {
    let solved = 0, rejected = 0, worstClosure = 0;
    for (const pose of poses(250, 11 + name.length)) {
      const result = evaluatePose(layout, pose, OPTIONS);
      const legs = result.geometricallyReachable ? 6 : result.servoAngles.length;
      for (let leg = 0; leg < legs; leg++) {
        const label = `${name} ${JSON.stringify(pose)} leg ${leg}`;
        const q = platformPoint(layout, pose, layout.platformAnchors[leg]);
        const base = layout.baseAnchors[leg], beta = layout.betaAngles[leg];
        const alpha = result.servoAngles[leg];
        closeVector(result.platformPoints[leg], q, 1e-9, `${label} platform point`);
        closeVector(result.hornTips[leg], hornTip(base, beta, layout.hornLength, alpha), 1e-9, `${label} horn tip`);
        const closure = Math.abs(norm(sub(q, hornTip(base, beta, layout.hornLength, alpha))) - layout.rodLength);
        worstClosure = Math.max(worstClosure, closure);
        close(result.rodLengths[leg], layout.rodLength, 1e-9, `${label} rod length`);
        // The angle is one of the independently found roots, on the documented branch.
        const roots = servoAngleRoots(base, beta, layout.hornLength, layout.rodLength, q);
        const match = roots.find(root => Math.abs(root.alpha - alpha) < 1e-7);
        assert.ok(match, `${label}: servo angle ${alpha} is not a root of ${JSON.stringify(roots)}`);
        assert.ok(match.slope < 0, `${label}: servo angle ${alpha} is on the rising-gap branch`);
      }
      if (result.geometricallyReachable) solved++;
      else {
        // A leg the evaluator could not close has no root at all.
        const failed = result.violations.find(violation => violation.type === 'invalidGeometry');
        assert.ok(failed, `${name} ${JSON.stringify(pose)}: unexpected rejection ${JSON.stringify(result.violations)}`);
        const q = platformPoint(layout, pose, layout.platformAnchors[failed.leg]);
        assert.deepEqual(servoAngleRoots(layout.baseAnchors[failed.leg], layout.betaAngles[failed.leg],
          layout.hornLength, layout.rodLength, q), [], `${name} ${JSON.stringify(pose)} leg ${failed.leg} has a root`);
        rejected++;
      }
    }
    assert.ok(solved >= 150 && rejected >= 15, `${name}: ${solved} solved and ${rejected} rejected of 250 poses`);
    assert.ok(worstClosure < 1e-9, `${name}: rod closure error ${worstClosure} mm`);
    assert.equal(solved + rejected, 250);
  }
});
