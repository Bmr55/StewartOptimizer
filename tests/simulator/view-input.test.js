import test from 'node:test';
import assert from 'node:assert/strict';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { CAMERA_DISTANCE_RANGE, CAMERA_PITCH_LIMIT, createSimulatorView } from '../../src/simulator/view.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createFakeDocument } from './helpers.js';

// Keyboard, pointer, wheel and gamepad handlers of the simulator view, checked
// against the values documented in docs/SIMULATOR.md.
const idleRenderer = () => ({ available: false, contextLost: false, render() {}, dispose() {} });

function mount({ window = { addEventListener() {} }, isActive, renderer } = {}) {
  const document = createFakeDocument();
  const controller = createSimulatorController();
  const fake = renderer ?? { available: true, contextLost: false, renders: 0, render() { fake.renders++; }, dispose() {} };
  const view = createSimulatorView({ document, window, controller, createRenderer: () => fake, isActive });
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  return { document, controller, view, renderer: fake, input: id => document.getElementById(id) };
}

test('keyboard pose steps follow the documented keys, Shift doubles them, and fields or a hidden tab swallow them', () => {
  const { document, controller } = mount();
  let prevented = 0;
  const press = (key, extra = {}) => document.dispatch('keydown', { key, target: { tagName: 'BODY' },
    preventDefault() { prevented++; }, ...extra });
  const requested = () => controller.getState().requested;
  press('ArrowRight');
  assert.equal(requested().x, 1, 'arrow right moves X by 1 mm');
  press('ArrowRight', { shiftKey: true });
  assert.equal(requested().x, 3, 'Shift doubles the step');
  press('ArrowLeft'); press('ArrowUp'); press('ArrowDown'); press('ArrowDown');
  assert.equal(requested().x, 2);
  assert.equal(requested().y, -1);
  press('PageUp'); press('PageUp'); press('PageDown');
  assert.equal(requested().z, 1);
  press('w'); press('S');
  assert.ok(Math.abs(requested().rx) < 1e-12, 'W then S cancel about X');
  press('d');
  assert.ok(Math.abs(requested().ry - Math.PI / 180) < 1e-12, 'D tilts +1 degree about Y');
  press('q');
  assert.ok(Math.abs(requested().rz + Math.PI / 180) < 1e-12, 'Q rotates -1 degree about Z');
  press('e', { shiftKey: true });
  assert.ok(Math.abs(requested().rz - Math.PI / 180) < 1e-12, 'Shift+E rotates +2 degrees about Z');
  assert.equal(prevented, 14, 'every handled key is prevented');
  press('x');
  assert.equal(prevented, 14, 'an unmapped key is not prevented');
  press('ArrowRight', { target: { tagName: 'INPUT' } });
  press('ArrowRight', { target: { tagName: 'TEXTAREA' } });
  press('ArrowRight', { target: { tagName: 'SELECT' } });
  assert.equal(requested().x, 2, 'keys are ignored while a field has focus');
  const hidden = mount({ isActive: () => false, renderer: idleRenderer() });
  hidden.document.dispatch('keydown', { key: 'ArrowRight', target: { tagName: 'BODY' } });
  assert.equal(hidden.controller.getState().requested.x, 0, 'a hidden Simulate tab ignores keys');
});

test('pointer drags orbit the camera or move the platform, the wheel zooms within limits and Reset camera restores the view', () => {
  const { controller, view, renderer, input } = mount();
  const canvas = input('simCanvas');
  const initial = view.getCamera();
  assert.deepEqual(initial, { yaw: 0.7, pitch: 0.38, distance: 600, target: [0, 0, 100] });
  canvas.dispatch('pointerdown', { button: 2, clientX: 0, clientY: 0, pointerId: 1 });
  canvas.dispatch('pointermove', { clientX: 100, clientY: 0 });
  assert.deepEqual(view.getCamera(), initial, 'a right-button drag must not orbit the camera');
  const rendersBefore = renderer.renders;
  canvas.dispatch('pointerdown', { button: 0, clientX: 0, clientY: 0, pointerId: 1 });
  canvas.dispatch('pointermove', { clientX: 100, clientY: 50 });
  let camera = view.getCamera();
  assert.ok(Math.abs(camera.yaw - (0.7 + 100 * 0.006)) < 1e-12, 'yaw follows dx at 0.006 rad per pixel');
  assert.ok(Math.abs(camera.pitch - (0.38 + 50 * 0.006)) < 1e-12, 'pitch follows dy at 0.006 rad per pixel');
  assert.equal(renderer.renders, rendersBefore + 1, 'an orbit drag redraws once');
  canvas.dispatch('pointermove', { clientX: 100, clientY: 5000 });
  assert.equal(view.getCamera().pitch, CAMERA_PITCH_LIMIT, 'pitch is clamped');
  canvas.dispatch('pointerup', {});
  canvas.dispatch('pointermove', { clientX: 0, clientY: 0 });
  assert.equal(view.getCamera().pitch, CAMERA_PITCH_LIMIT, 'a released drag no longer orbits');
  input('simPointerMode').value = 'platform';
  canvas.dispatch('pointerdown', { button: 0, clientX: 10, clientY: 10, pointerId: 1 });
  canvas.dispatch('pointermove', { clientX: 30, clientY: 50 });
  camera = view.getCamera();
  assert.ok(Math.abs(controller.getState().requested.x - 20 * 0.35) < 1e-12, 'X follows dx at 0.35 mm per pixel');
  assert.ok(Math.abs(controller.getState().requested.y + 40 * 0.35) < 1e-12, 'Y follows -dy at 0.35 mm per pixel');
  assert.equal(camera.pitch, CAMERA_PITCH_LIMIT, 'platform mode leaves the camera alone');
  canvas.dispatch('pointercancel', {});
  canvas.dispatch('wheel', { deltaY: 1000 });
  assert.ok(Math.abs(view.getCamera().distance - 600 * Math.E) < 1e-9, 'the wheel scales distance by exp(deltaY / 1000)');
  canvas.dispatch('wheel', { deltaY: 1e6 });
  assert.equal(view.getCamera().distance, CAMERA_DISTANCE_RANGE[1]);
  canvas.dispatch('wheel', { deltaY: -1e7 });
  assert.equal(view.getCamera().distance, CAMERA_DISTANCE_RANGE[0]);
  assert.deepEqual(CAMERA_DISTANCE_RANGE, [80, 2500], 'documented zoom range');
  view.setCamera({ target: [5, 6, 7] });
  input('simResetCamera').dispatch('click');
  assert.deepEqual(view.getCamera(), { yaw: 0.7, pitch: 0.38, distance: 600, target: [5, 6, 7] },
    'Reset camera restores the default orientation and distance but keeps the target');
});

test('a gamepad moves the platform each frame with the documented dead zone, only while enabled', () => {
  const frames = [];
  const pad = { axes: [0.5, 0, 0, 0], buttons: [] };
  const window = { addEventListener() {}, requestAnimationFrame: fn => { frames.push(fn); return frames.length; },
    navigator: { getGamepads: () => [null, pad] } };
  const { document, controller } = mount({ window, renderer: idleRenderer() });
  assert.equal(frames.length, 1, 'the view schedules its first frame');
  frames.at(-1)(0);
  frames.at(-1)(100);
  assert.equal(controller.getState().requested.x, 0, 'the gamepad is off by default');
  document.getElementById('simGamepad').checked = true;
  frames.at(-1)(200);
  assert.ok(Math.abs(controller.getState().requested.x - 0.5 * 25 * 0.1) < 1e-12, 'left stick X at 25 mm/s per unit');
  pad.axes = [0, -0.1, 0, 0.14];
  frames.at(-1)(300);
  assert.equal(controller.getState().requested.y, 0, 'stick values inside the 0.15 dead zone are ignored');
  pad.axes = [0, 0, 0, 0];
  pad.buttons = [{ value: 0 }, { value: 0 }, { value: 0 }, { value: 0 }, { value: 0 }, { value: 1 }, { value: 0 }, { value: 1 }];
  frames.at(-1)(400);
  const state = controller.getState();
  assert.ok(Math.abs(state.requested.z - 25 * 0.1) < 1e-12, 'the right trigger raises Z');
  assert.ok(Math.abs(state.requested.rz - 0.3 * 0.1) < 1e-12, 'the right shoulder button yaws');
  frames.at(-1)(2400);
  assert.ok(Math.abs(controller.getState().requested.z - 25 * 0.2) < 1e-12, 'a long frame gap is clamped to 0.1 s');
});

test('Clear trace empties the recorded trace while traces stay enabled', () => {
  const { controller, input } = mount();
  input('simTraces').checked = true;
  input('simTraces').dispatch('change');
  controller.requestPose({ x: 5 });
  controller.requestPose({ x: 10 });
  const before = controller.getState().trace.length;
  assert.ok(before >= 2, `the trace holds the two accepted requests: ${before}`);
  input('simClearTrace').dispatch('click');
  const cleared = controller.getState().trace.length;
  assert.ok(cleared < before && cleared <= 1, `Clear trace left ${cleared} points`);
  assert.equal(controller.getState().tracesEnabled, true);
  controller.requestPose({ x: 15 });
  assert.equal(controller.getState().trace.length, cleared + 1, 'tracing continues from the cleared trace');
});
