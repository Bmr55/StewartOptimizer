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

test('traces keep the most recent 300 accepted platform-origin positions', () => {
  const layout = asymmetricJointFixture();
  const controller = createSimulatorController();
  controller.loadLayout(layout, { options: settings });
  controller.setTraces(true);
  let state;
  for (let i = 1; i <= 305; i++) state = controller.requestPose({ z: i * 0.01 });
  assert.equal(state.trace.length, 300);
  // The five oldest positions were dropped: the first retained entry is request 6.
  assert.deepEqual(state.trace[0], [0, 0, layout.homeHeight + 6 * 0.01]);
  assert.deepEqual(state.trace.at(-1), [0, 0, layout.homeHeight + 305 * 0.01]);
  state = controller.requestPose({ z: 100 });
  assert.equal(state.rejected, true);
  assert.equal(state.trace.length, 300, 'a rejected request adds nothing');
  assert.deepEqual(controller.clearTrace().trace, []);
  controller.setTraces(false);
  assert.deepEqual(controller.requestPose({ z: 1 }).trace, []);
});

test('animation frames advance at most 0.1 s of simulated time, scaled by speed', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: settings });
  controller.setAnimation('pingpong', true, { speed: 1, reset: true });
  let state = controller.tick(5);
  assert.equal(state.animation.seconds, 0.1);
  assert.equal(state.animation.playing, true);
  // Default ping-pong: apex 20 mm at 0.25 Hz.
  assert.ok(Math.abs(state.requested.z - 20 * Math.sin(2 * Math.PI * 0.25 * 0.1)) < 1e-12);
  state = controller.tick(0.04);
  assert.ok(Math.abs(state.animation.seconds - 0.14) < 1e-12);
  controller.setAnimation('pingpong', true, { speed: 3 });
  state = controller.tick(1);
  assert.ok(Math.abs(state.animation.seconds - 0.44) < 1e-12, 'the clamp applies before the speed multiplier');
  state = controller.tick(0.02);
  assert.ok(Math.abs(state.animation.seconds - 0.5) < 1e-12);
  assert.throws(() => controller.tick(-0.1), RangeError);
  assert.throws(() => controller.tick(NaN), RangeError);
});

test('wobble tilts about X with the sine and about Y with the cosine of the phase', () => {
  const tenDegrees = Math.PI / 18;
  assert.deepEqual(animationPose('wobble', 0), { ...HOME_POSE, rx: 0, ry: tenDegrees });
  // A quarter cycle at the default 0.25 Hz is one second.
  const quarter = animationPose('wobble', 1);
  assert.ok(Math.abs(quarter.rx - tenDegrees) < 1e-12);
  assert.ok(Math.abs(quarter.ry) < 1e-12);
  assert.deepEqual([quarter.x, quarter.y, quarter.z, quarter.rz], [0, 0, 0, 0]);
  const eighth = animationPose('wobble', 0.5, { rotationRad: 0.2 });
  assert.ok(Math.abs(eighth.rx - 0.2 * Math.SQRT1_2) < 1e-12);
  assert.ok(Math.abs(eighth.ry - 0.2 * Math.SQRT1_2) < 1e-12);
  assert.ok(Math.abs(animationPose('wobble', 0.5, { frequencyHz: 0.5 }).rx - tenDegrees) < 1e-12);
});

test('the idle pattern never plays and a tick while idle changes nothing', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: settings });
  let state = controller.setAnimation('none', true, { speed: 2 });
  assert.equal(state.animation.playing, false);
  assert.equal(state.animation.pattern, 'none');
  assert.equal(state.animation.speed, 2);
  const before = controller.getState();
  state = controller.tick(0.1);
  assert.deepEqual(state, before);
  assert.deepEqual(animationPose('none', 3), HOME_POSE);
  assert.throws(() => controller.setAnimation('spiral', true), RangeError);
  assert.throws(() => controller.setAnimation('wobble', true, { speed: 0 }), RangeError);
});

test('loading a layout pauses and rewinds a running animation but keeps its pattern and speed', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: settings });
  controller.setTraces(true);
  controller.setAnimation('wobble', true, { speed: 2 });
  let state = controller.tick(0.05);
  assert.equal(state.animation.playing, true);
  assert.ok(state.animation.seconds > 0);
  assert.equal(state.trace.length, 1);
  state = controller.loadLayout({ ...asymmetricJointFixture(), homeHeight: 205 },
    { source: { kind: 'candidate', candidateId: 3 }, options: settings });
  assert.deepEqual(state.animation, { pattern: 'wobble', playing: false, seconds: 0, speed: 2, pauseReason: null });
  assert.deepEqual(state.requested, HOME_POSE);
  assert.deepEqual(state.accepted, HOME_POSE);
  assert.equal(state.requestSource, 'load');
  assert.deepEqual(state.source, { kind: 'candidate', candidateId: 3 });
  assert.equal(state.layout.homeHeight, 205);
  assert.deepEqual(state.trace, [[0, 0, 205]], 'the trace restarts at the new home');
  assert.deepEqual(controller.tick(0.1), controller.getState(), 'paused: a tick does not advance');
});

test('clear forgets the layout, its options and pose state', () => {
  const notifications = [];
  const controller = createSimulatorController({ onChange: state => notifications.push(state) });
  controller.loadLayout(asymmetricJointFixture(), { source: { kind: 'candidate', candidateId: 2 },
    options: { ...settings, conditionLimit: 1000 } });
  controller.requestPose({ z: 5 });
  controller.setAnimation('rotate', true);
  const count = notifications.length;
  const state = controller.clear();
  assert.equal(notifications.length, count + 1);
  assert.equal(state.layout, null);
  assert.equal(state.source, null);
  assert.deepEqual(state.options, {});
  assert.deepEqual(state.requested, HOME_POSE);
  assert.equal(state.accepted, null);
  assert.equal(state.assessment, null);
  assert.equal(state.acceptedAssessment, null);
  assert.equal(state.rejected, false);
  assert.deepEqual(state.trace, []);
  assert.deepEqual(state.animation, { pattern: 'rotate', playing: false, seconds: 0, speed: 1, pauseReason: null });
  assert.equal(controller.getReferenceLayout(), null);
  assert.throws(() => controller.requestPose({ z: 1 }), /Load a layout before requesting a pose/);
  // Options set after a clear start from nothing: the old condition limit is gone.
  assert.deepEqual(controller.setOptions({ ballJointLimitDeg: 45 }).options, { ballJointLimitDeg: 45 });
  const reloaded = controller.loadLayout(asymmetricJointFixture());
  assert.deepEqual(reloaded.options, {});
  assert.deepEqual(reloaded.source, { kind: 'import' });
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
