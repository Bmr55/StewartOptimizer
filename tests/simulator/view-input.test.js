import test from 'node:test';
import assert from 'node:assert/strict';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { CAMERA_DISTANCE_RANGE, CAMERA_PITCH_LIMIT, createSimulatorView, overlayInputId,
  ZOOM_PICK_RADIUS_PX } from '../../src/simulator/view.js';
import { OVERLAY_DEFAULTS, OVERLAY_NAMES } from '../../src/simulator/scene.js';
import { cameraFrame, projectPoint, projectSegment } from '../../src/simulator/renderer.js';
import { buildSceneGeometry } from '../../src/simulator/scene.js';
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
  assert.deepEqual(CAMERA_DISTANCE_RANGE, [10, 2500], 'documented zoom range');
  view.setCamera({ target: [5, 6, 7] });
  input('simResetCamera').dispatch('click');
  assert.deepEqual(view.getCamera(), { yaw: 0.7, pitch: 0.38, distance: 600, target: [0, 0, 100] },
    'Reset camera restores the default orientation and distance and recentres on the layout');
});

// A canvas laid out at 800 × 500 CSS pixels, 100 px from the page corner.
function mountWithBox() {
  const mounted = mount();
  const canvas = mounted.input('simCanvas');
  canvas.getBoundingClientRect = () => ({ left: 100, top: 100, width: 800, height: 500 });
  return { ...mounted, canvas };
}
// Page coordinates of a world point for the current camera.
const onScreen = (view, point) => {
  const [x, y] = projectPoint(point, view.getCamera(), 800, 500);
  return [100 + (x + 1) / 2 * 800, 100 + (1 - y) / 2 * 500];
};
const closeTo = (actual, expected, tolerance, label) => actual.forEach((value, k) =>
  assert.ok(Math.abs(value - expected[k]) < tolerance, `${label}: ${actual} vs ${expected}`));

test('the wheel zooms toward the drawn geometry under the cursor, down to 10 mm', () => {
  const { view, canvas, controller } = mountWithBox();
  const start = view.getCamera();
  // A horn tip lies well in front of the plane through the target; it stays under the cursor.
  const tip = controller.getState().acceptedAssessment.hornTips[0];
  const frame = cameraFrame(start);
  const tipDepth = tip.map((value, k) => value - frame.eye[k]).reduce((sum, value, k) => sum + value * frame.forward[k], 0);
  assert.ok(Math.abs(tipDepth - start.distance) > 20, 'the tip should be off the target plane');
  const [clientX, clientY] = onScreen(view, tip);
  for (const deltaY of [-500, -500, -1000, 300]) {
    canvas.dispatch('wheel', { deltaY, clientX, clientY });
    closeTo(onScreen(view, tip), [clientX, clientY], 1e-6, `after deltaY ${deltaY}`);
  }
  assert.ok(view.getCamera().distance < start.distance / 4);
  canvas.dispatch('wheel', { deltaY: -1e7, clientX, clientY });
  assert.equal(view.getCamera().distance, CAMERA_DISTANCE_RANGE[0]);
  closeTo(onScreen(view, tip), [clientX, clientY], 1e-6, 'at the 10 mm limit');
});

test('over empty space the wheel keeps the point on the target plane under the cursor', () => {
  const { view, canvas, controller } = mountWithBox();
  const start = view.getCamera();
  const frame = cameraFrame(start);
  const point = start.target.map((value, k) => value + frame.right[k] * 380 + frame.up[k] * 250);
  const [clientX, clientY] = onScreen(view, point);
  // Nothing is drawn within the pick radius of that spot.
  const toPage = ([x, y]) => [100 + (x + 1) / 2 * 800, 100 + (1 - y) / 2 * 500];
  for (const line of buildSceneGeometry(controller.getState()).lines) {
    const segment = projectSegment(line.from, line.to, start, 800, 500);
    if (!segment) continue;
    const [a, b] = segment.map(toPage);
    for (let s = 0; s <= 1; s += 0.01) {
      const gap = Math.hypot(a[0] + (b[0] - a[0]) * s - clientX, a[1] + (b[1] - a[1]) * s - clientY);
      assert.ok(gap > ZOOM_PICK_RADIUS_PX, 'the chosen spot is not empty');
    }
  }
  canvas.dispatch('wheel', { deltaY: -400, clientX, clientY });
  closeTo(onScreen(view, point), [clientX, clientY], 1e-6, 'empty-space zoom');
  // Without a laid-out canvas the wheel zooms about the target.
  const bare = mount();
  bare.input('simCanvas').dispatch('wheel', { deltaY: -800, clientX: 30, clientY: 40 });
  assert.deepEqual(bare.view.getCamera().target, [0, 0, 100]);
});

test('Shift+drag pans in either mouse mode and the target survives pose updates until a new layout', () => {
  const { view, canvas, controller, input } = mountWithBox();
  for (const mode of ['orbit', 'platform']) {
    input('simPointerMode').value = mode;
    const before = view.getCamera();
    const anchor = before.target.slice();
    const [x, y] = onScreen(view, anchor);
    canvas.dispatch('pointerdown', { button: 0, clientX: x, clientY: y, pointerId: 1, shiftKey: true });
    canvas.dispatch('pointermove', { clientX: x + 40, clientY: y - 15 });
    canvas.dispatch('pointerup', {});
    closeTo(onScreen(view, anchor), [x + 40, y - 15], 1e-6, `${mode}: the grabbed point follows the cursor`);
    const after = view.getCamera();
    assert.deepEqual([after.yaw, after.pitch, after.distance], [before.yaw, before.pitch, before.distance], `${mode}: pan only`);
    assert.deepEqual(controller.getState().requested, { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 }, `${mode}: pan moved the platform`);
  }
  const panned = view.getCamera().target;
  assert.notDeepEqual(panned, [0, 0, 100]);
  controller.requestPose({ x: 4, rz: 0.05 });
  controller.setMarkers(false);
  assert.deepEqual(view.getCamera().target, panned, 'a pose or display change recentred the view');
  // A saved camera target is kept through later updates too.
  view.setCamera({ target: [7, 8, 9] });
  controller.requestPose({});
  assert.deepEqual(view.getCamera().target, [7, 8, 9]);
  // Reloading the same layout keeps the view; a new home height recentres it.
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  assert.deepEqual(view.getCamera().target, [7, 8, 9]);
  const taller = { ...asymmetricJointFixture(), homeHeight: 240 };
  controller.loadLayout(taller, { options: { ballJointLimitDeg: 180 } });
  assert.deepEqual(view.getCamera().target, [0, 0, 120]);
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

test('overlay checkboxes toggle the controller and follow its state', () => {
  const { controller, renderer, input } = mount();
  for (const name of OVERLAY_NAMES) assert.equal(input(overlayInputId(name)).checked, OVERLAY_DEFAULTS[name], name);
  assert.equal(OVERLAY_DEFAULTS.reachabilityCloud, false, 'the cloud costs evaluator time and starts off');
  assert.equal(overlayInputId('worldAxes'), 'simOverlayWorldAxes');
  const renders = renderer.renders;
  input('simOverlayWorldAxes').checked = false;
  input('simOverlayWorldAxes').dispatch('change');
  assert.deepEqual(controller.getState().overlays, { ...OVERLAY_DEFAULTS, worldAxes: false });
  assert.ok(renderer.renders > renders, 'a toggle did not redraw');
  controller.setOverlays({ worldAxes: true, platformAxes: false });
  assert.equal(input('simOverlayWorldAxes').checked, true);
  assert.equal(input('simOverlayPlatformAxes').checked, false);
});
