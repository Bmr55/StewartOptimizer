import test from 'node:test';
import assert from 'node:assert/strict';
import { computeCycleDemand, evaluateCyclePose } from '../cycle.js';
import { evaluatePose } from '../workspace.js';
import { jointFixture } from './fixture.js';

function asymmetricLayout() {
  const layout = jointFixture();
  layout.platformAnchors[0][0] += 8;
  layout.platformAnchors[2][1] -= 12;
  layout.baseAnchors[3][0] -= 14;
  layout.betaAngles[1] += 0.2;
  layout.betaAngles[4] -= 0.3;
  return layout;
}

test('cycle axis changes geometric speed and load demand on asymmetric layout', () => {
  const results = ['x', 'y', 'z'].map(axis => computeCycleDemand(asymmetricLayout(), {
    mass: 2.5, stroke: 10, frequency: 2, axis, ballJointLimitDeg: 180
  }));
  assert.ok(results.every(result => result.valid), JSON.stringify(results));
  for (let i = 0; i < 2; i++) {
    assert.ok(Math.abs(results[i].torqueNm - results[i + 1].torqueNm) > 1e-3);
    assert.ok(Math.abs(results[i].speedRadPerSec - results[i + 1].speedRadPerSec) > 1e-3);
  }
});

test('rod forces balance the applied wrench with gravity remaining vertical', () => {
  const result = evaluateCyclePose(asymmetricLayout(), {}, [0.1, 0, 0], [2, 0, 0], 3, { ballJointLimitDeg: 180 });
  assert.equal(result.valid, true);
  assert.deepEqual(result.requiredForce, [6, 0, 29.43]);
  const actual = result.equilibrium.map(row => row.reduce((sum, value, i) => sum + value * result.rodForces[i], 0));
  const expected = [...result.requiredForce, 0, 0, 0];
  actual.forEach((value, i) => assert.ok(Math.abs(value - expected[i]) < 1e-8));
});

test('static payload has zero speed and the same gravity demand for every axis', () => {
  const results = ['x', 'y', 'z'].map(axis => computeCycleDemand(asymmetricLayout(), {
    mass: 2, stroke: 10, frequency: 0, axis, ballJointLimitDeg: 180
  }));
  assert.ok(results.every(r => r.valid && r.speedRadPerSec === 0));
  assert.equal(results[0].torqueNm, results[2].torqueNm);
});

test('invalid cycle poses are explicitly unavailable, not zero-demand results', () => {
  const result = computeCycleDemand(asymmetricLayout(), { mass: 2, stroke: 10, frequency: 2, ballJointLimitDeg: 0 });
  assert.equal(result.valid, false);
  assert.equal(result.torqueNm, null);
  assert.match(result.reason, /constraint/);
});

test('servo speeds agree with finite differences of the inverse kinematics', () => {
  const layout = asymmetricLayout();
  const velocity = [0, 0.1, 0];
  const dt = 1e-5;
  const options = { ballJointLimitDeg: 180 };
  const demand = evaluateCyclePose(layout, {}, velocity, [0, 0, 0], 2, options);
  const before = evaluatePose(layout, { y: -velocity[1] * dt * 1000 }, options);
  const after = evaluatePose(layout, { y: velocity[1] * dt * 1000 }, options);
  assert.ok(before.reachable && after.reachable && demand.valid);
  demand.speed.forEach((speed, i) => {
    const difference = Math.abs((after.servoAngles[i] - before.servoAngles[i]) / (2 * dt));
    assert.ok(Math.abs(speed - difference) < 1e-6);
  });
});
