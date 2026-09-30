import test from 'node:test';
import assert from 'node:assert/strict';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { buildSceneGeometry, createWebGLRenderer, projectPoint } from '../../src/simulator/renderer.js';

test('scene uses solved asymmetric anchor, horn, rod and platform frames', () => {
  const controller = createSimulatorController();
  const state = controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const scene = buildSceneGeometry(state);
  assert.ok(scene.lines.some(line => line.from === undefined) === false);
  for (let leg = 0; leg < 6; leg++) {
    const base = state.layout.baseAnchors[leg];
    const horn = state.acceptedAssessment.hornTips[leg];
    const point = state.acceptedAssessment.platformPoints[leg];
    assert.ok(scene.lines.some(line => line.from === base && line.to === horn));
    assert.ok(scene.lines.some(line => line.from === horn && line.to === point));
    assert.ok(scene.points.some(marker => marker.at === horn));
  }
  assert.equal(state.acceptedAssessment.translation[2], state.layout.homeHeight);
  assert.ok(projectPoint(state.layout.baseAnchors[0], { target: [0, 0, 100] }, 800, 500));
  const rejected = controller.loadLayout(asymmetricJointFixture(),
    { options: { ballJointLimitDeg: 180, conditionLimit: 1 } });
  assert.equal(rejected.accepted, null);
  assert.equal(buildSceneGeometry(rejected).lines.length, 15); // Base, servo directions, world axes.
});

test('unavailable WebGL2 leaves renderer inactive with actionable error', () => {
  const renderer = createWebGLRenderer({ getContext: () => null });
  assert.equal(renderer.available, false);
  assert.match(renderer.error, /WebGL2.*hardware acceleration.*optimization remains available/i);
  renderer.render({});
});

test('projected depth keeps near/far ordering for a zoomed-out camera', () => {
  const target = [0, 0, 100];
  const yaw = 0.7, pitch = 0.4;
  const towardEye = [Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)];
  const offset = (sign, units) => target.map((value, i) => value + sign * units * towardEye[i]);
  for (const distance of [600, 1900, 2500]) {
    const camera = { target, yaw, pitch, distance };
    const [nearer, middle, farther] = [offset(1, 300), target, offset(-1, 300)]
      .map(point => projectPoint(point, camera, 800, 500)[2]);
    assert.ok(nearer < middle && middle < farther, `distance ${distance}: ${nearer} ${middle} ${farther}`);
    assert.ok(farther < 0.999 && nearer > -0.999, `distance ${distance} saturates the depth range`);
  }
  // The depth mapping is unchanged for the default camera distance.
  const home = projectPoint(target, { target, yaw, pitch, distance: 600 }, 800, 500)[2];
  assert.ok(Math.abs(home - (599 / 2000 * 2 - 1)) < 1e-12);
});
