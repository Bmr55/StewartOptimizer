import test from 'node:test';
import assert from 'node:assert/strict';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { animationPose, createSimulatorController, HOME_POSE } from '../../src/simulator/controller.js';

const settings = { ballJointLimitDeg: 180 };

test('candidate geometry is copied and rejected requests retain the last valid pose', () => {
  const sourceLayout = asymmetricJointFixture();
  const original = structuredClone(sourceLayout);
  const controller = createSimulatorController();
  let state = controller.loadLayout(sourceLayout,
    { source: { kind: 'candidate', candidateId: 12 }, options: settings });
  assert.deepEqual(state.accepted, HOME_POSE);
  assert.equal(state.assessment.reachable, true);
  assert.equal(state.assessment.servoAngles.length, 6);
  state = controller.requestPose({ z: 10 });
  assert.equal(state.accepted.z, 10);
  state = controller.requestPose({ z: 100 });
  assert.equal(state.rejected, true);
  assert.equal(state.requested.z, 100);
  assert.equal(state.accepted.z, 10);
  assert.equal(state.assessment.violations[0].type, 'invalidGeometry');
  assert.equal(state.acceptedAssessment.reachable, true);
  sourceLayout.baseAnchors[0][0] += 1000;
  assert.deepEqual(controller.getReferenceLayout(), original);
  const reference = controller.getReferenceLayout();
  reference.baseAnchors[0][0] += 1000;
  assert.deepEqual(controller.getReferenceLayout(), original);
});

test('no successful pose is fabricated when home is invalid and settings refresh diagnostics', () => {
  const controller = createSimulatorController();
  let state = controller.loadLayout(asymmetricJointFixture(),
    { options: { ...settings, conditionLimit: 1 } });
  assert.equal(state.accepted, null);
  assert.equal(state.rejected, true);
  assert.equal(state.assessment.violations.at(-1).type, 'conditionLimit');
  state = controller.setOptions({ conditionLimit: null });
  assert.deepEqual(state.accepted, HOME_POSE);
  assert.equal(state.assessment.reachable, true);
  state = controller.setOptions({ conditionLimit: 1 });
  assert.equal(state.accepted, null);
  assert.equal(state.rejected, true);
});

test('animations pause on invalid frames and preserve requested and accepted states', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: settings });
  controller.setTraces(true);
  controller.setAnimation('pingpong', true);
  const state = controller.tick(0.1, { apexMm: 100, frequencyHz: 2.5 });
  assert.equal(state.animation.playing, false);
  assert.match(state.animation.pauseReason, /invalidGeometry/);
  assert.equal(state.requested.z, 100);
  assert.deepEqual(state.accepted, HOME_POSE);
  assert.equal(state.trace.length, 0);
  for (const pattern of ['wobble', 'pingpong', 'rotate', 'tilt', 'helical']) {
    assert.deepEqual(Object.keys(animationPose(pattern, 0)).sort(), Object.keys(HOME_POSE).sort());
  }
});

test('loadLayout with invalid options throws before touching state and fires no notification', () => {
  const notifications = [];
  const controller = createSimulatorController({ onChange: state => notifications.push(state) });
  const first = asymmetricJointFixture();
  controller.loadLayout(first, { source: { kind: 'candidate', candidateId: 1 }, options: settings });
  controller.requestPose({ z: 10 });
  const before = controller.getState();
  const count = notifications.length;
  const next = { ...asymmetricJointFixture(), homeHeight: 555 };
  for (const options of [{ ...settings, rodLengthTolerance: -1 }, { ...settings, conditionLimit: 'bad' }]) {
    assert.throws(() => controller.loadLayout(next, { source: { kind: 'import' }, options }), RangeError);
  }
  assert.deepEqual(controller.getState(), before);
  assert.equal(controller.getState().layout.homeHeight, first.homeHeight);
  assert.equal(notifications.length, count);
  const empty = createSimulatorController();
  assert.throws(() => empty.loadLayout(next, { options: { ...settings, rodLengthTolerance: -1 } }), RangeError);
  assert.equal(empty.getState().layout, null);
});

test('non-object poses and option patches are rejected by name instead of throwing raw TypeErrors', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const before = controller.getState();
  for (const pose of [null, 'home', 42, [0, 0, 0]]) {
    assert.throws(() => controller.requestPose(pose), /A requested pose must be an object with six finite coordinates/);
  }
  for (const patch of [null, 'abc', 7, ['x']]) {
    assert.throws(() => controller.setOptions(patch), /Simulator options must be an object/);
  }
  assert.deepEqual(controller.getState(), before);
  assert.throws(() => controller.loadLayout(asymmetricJointFixture(), { options: 'abc' }), /Simulator options must be an object/);
});
