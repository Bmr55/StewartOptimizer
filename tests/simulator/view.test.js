import test from 'node:test';
import assert from 'node:assert/strict';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { createSimulatorView } from '../../src/simulator/view.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createFakeDocument } from './helpers.js';

function mount({ renderer } = {}) {
  const document = createFakeDocument();
  for (const axis of ['X', 'Y', 'Z']) {
    Object.assign(document.getElementById(`sim${axis}Slider`), { min: '-50', max: '50' });
  }
  for (const axis of ['RX', 'RY', 'RZ']) {
    Object.assign(document.getElementById(`sim${axis}Slider`), { min: '-30', max: '30' });
  }
  const controller = createSimulatorController();
  const fake = renderer ?? { available: true, contextLost: false, render() { fake.renders++; }, dispose() {}, renders: 0 };
  const view = createSimulatorView({ document, window: { addEventListener() {} }, controller,
    createRenderer: () => fake });
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  return { document, controller, view, renderer: fake, input: id => document.getElementById(id) };
}

test('a focused pose field keeps its text across animation frames and follows requests when untouched', () => {
  const { document, controller, input } = mount();
  const x = input('simXInput');
  const y = input('simYInput');
  assert.equal(x.value, '0');
  controller.setAnimation('wobble', true);
  document.activeElement = x;
  x.value = '1';
  y.value = '9';
  controller.tick(0.016);
  assert.equal(x.value, '1', 'focused field was rewritten by the frame');
  assert.equal(y.value, '0', 'unfocused field was not refreshed');
  assert.equal(controller.getState().animation.playing, true);
  x.dispatch('change');
  assert.equal(controller.getState().requested.x, 1);
  assert.equal(controller.getState().animation.playing, false);
  assert.equal(x.value, '1');
  // Committed, so still untouched: a reset with focus retained updates the field.
  input('simResetPose').dispatch('click');
  assert.equal(controller.getState().requested.x, 0);
  assert.equal(x.value, '0', 'focused committed field kept the pre-reset value');
  assert.equal(input('simXSlider').value, '0');
  // An untouched focused field follows the animation.
  controller.setAnimation('pingpong', true);
  controller.tick(0.5);
  assert.notEqual(input('simZInput').value, '0');
  document.activeElement = input('simZInput');
  controller.tick(0.1);
  assert.equal(input('simZInput').value, String(Number(controller.getState().requested.z.toFixed(2))));
});

test('an invalid pose entry reports the error and restores the requested value', () => {
  const { document, controller, input } = mount();
  const rx = input('simRXInput');
  controller.requestPose({ rx: Math.PI / 18 });
  assert.equal(rx.value, '10');
  document.activeElement = rx;
  rx.value = 'abc';
  rx.dispatch('change');
  assert.match(input('simPoseStatus').textContent, /six finite coordinates/);
  assert.equal(rx.value, '10', 'invalid entry was not restored');
  assert.equal(controller.getState().requested.rx, Math.PI / 18);
});

test('a lost WebGL2 context is reported in the pose status and cleared on restore', () => {
  let onContextChange;
  const renderer = { available: true, contextLost: false, renders: 0, render() { this.renders++; }, dispose() {} };
  const document = createFakeDocument();
  const controller = createSimulatorController();
  createSimulatorView({ document, window: { addEventListener() {} }, controller,
    createRenderer: (_canvas, options) => { onContextChange = options.onContextChange; return renderer; } });
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const status = document.getElementById('simPoseStatus');
  assert.match(status.textContent, /Accepted request/);
  renderer.contextLost = true;
  onContextChange();
  assert.match(status.textContent, /WebGL2 context lost/);
  controller.requestPose({ z: 5 });
  assert.match(status.textContent, /WebGL2 context lost/);
  assert.equal(controller.getState().requested.z, 5);
  renderer.contextLost = false;
  const before = renderer.renders;
  onContextChange();
  assert.match(status.textContent, /Accepted request: X 0, Y 0, Z 5/);
  assert.equal(renderer.renders, before + 1);
});

test('setCamera takes only known finite fields and rejects anything else by name', () => {
  const { view } = mount();
  const before = view.getCamera();
  for (const [camera, expected] of [['abc', /camera must be an object/], [{ yaw: 'abc' }, /camera.yaw must be a finite number/],
    [{ distance: -1 }, /camera.distance must be a finite number from 10 to 2500/], [{ target: 'x' }, /camera.target must contain three/]]) {
    assert.throws(() => view.setCamera(camera), expected);
  }
  assert.deepEqual(view.getCamera(), before);
  view.setCamera({ yaw: 1.25, junk: 'x' });
  assert.deepEqual(view.getCamera(), { ...before, yaw: 1.25 });
  assert.equal('junk' in view.getCamera(), false);
});

test('the render mode select chooses wireframe or solid for every draw and redraws on change', () => {
  const modes = [];
  const renderer = { available: true, contextLost: false, render(state, camera, options) { modes.push(options?.mode); }, dispose() {} };
  const { input, controller } = mount({ renderer });
  const select = input('simRenderMode');
  assert.equal(modes.at(-1), 'wireframe', 'an unset select draws wireframe');
  select.value = 'solid';
  const before = modes.length;
  select.dispatch('change');
  assert.equal(modes.length, before + 1);
  assert.equal(modes.at(-1), 'solid');
  controller.requestPose({ z: 5 });
  assert.equal(modes.at(-1), 'solid');
  select.value = 'shaded';
  select.dispatch('change');
  assert.equal(modes.at(-1), 'wireframe', 'an unknown value draws wireframe');
});
