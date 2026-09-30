import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { createGeometryControls } from '../../src/simulator/geometry-controls.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createFakeDocument } from './helpers.js';

function mount(layout) {
  const document = createFakeDocument();
  const controller = createSimulatorController();
  controller.loadLayout(layout, { options: { ballJointLimitDeg: 180 } });
  createGeometryControls({ document, container: document.createElement('div'), controller });
  return { document, controller, input: id => document.getElementById(id),
    status: () => document.getElementById('sim-geometry-status').textContent };
}

test('cleared or non-numeric geometry fields are rejected instead of being read as 0', () => {
  const explicit = mount(asymmetricJointFixture());
  const parametric = mount(new Optimizer({}, { topology: 'c3_paired', seed: 37 }).createRandomLayout());
  const cases = [
    [explicit, 'sim-base-0-x-number', 'Leg 1 X', state => state.layout.baseAnchors[0][0]],
    [explicit, 'sim-beta-1-number', 'Leg 2 β', state => state.layout.betaAngles[1]],
    [explicit, 'sim-servo-min-number', 'Servo minimum', state => state.layout.servoRangeRad[0]],
    [explicit, 'sim-hornLength-number', 'Horn length', state => state.layout.hornLength],
    [parametric, 'sim-base-orientation-number', 'Base turn', state => state.layout.topologyParameters.base_orientation],
  ];
  for (const [{ controller, input, status }, id, label, read] of cases) {
    const previous = read(controller.getState());
    const shown = input(id).value;
    assert.notEqual(previous, 0, `${id} fixture value must be nonzero for the check to mean anything`);
    for (const text of ['', '   ', 'abc', 'Infinity']) {
      input(id).value = text;
      input(id).dispatch('change');
      assert.equal(read(controller.getState()), previous, `${id} changed for ${JSON.stringify(text)}`);
      assert.equal(input(id).value, shown, `${id} not restored for ${JSON.stringify(text)}`);
      assert.match(status(), new RegExp(`${label}.*finite number`));
    }
  }
  explicit.input('sim-base-0-x-number').value = '12.5';
  explicit.input('sim-base-0-x-number').dispatch('change');
  assert.equal(explicit.controller.getState().layout.baseAnchors[0][0], 12.5);
});

test('a focused but untouched geometry field follows Reset geometry and a new load', () => {
  const source = asymmetricJointFixture();
  const { document, controller, input } = mount(source);
  const horn = input('sim-hornLength-number');
  const rod = input('sim-rodLength-number');
  // Safari and Firefox on macOS leave focus in the field when a button is clicked.
  document.activeElement = horn;
  horn.value = '77';
  horn.dispatch('change');
  assert.equal(controller.getState().layout.hornLength, 77);
  assert.equal(horn.value, '77');
  input('sim-reset-geometry').dispatch('click');
  assert.equal(controller.getState().layout.hornLength, source.hornLength);
  assert.equal(horn.value, String(source.hornLength), 'focused field kept the pre-reset value');
  assert.equal(input('sim-hornLength-range').value, String(source.hornLength));
  // Uncommitted typing in a focused field is kept through frames and a reset of another field.
  document.activeElement = rod;
  rod.value = '123';
  controller.setAnimation('wobble', true);
  controller.tick(0.016);
  assert.equal(rod.value, '123');
  rod.dispatch('change');
  assert.equal(controller.getState().layout.rodLength, 123);
  input('sim-reset-geometry').dispatch('click');
  assert.equal(rod.value, String(source.rodLength));
  // A wholesale load from outside the editor is also followed by a focused untouched field.
  document.activeElement = horn;
  const other = { ...asymmetricJointFixture(), hornLength: 61 };
  controller.loadLayout(other, { source: { kind: 'candidate', candidateId: 3 }, options: { ballJointLimitDeg: 180 } });
  assert.equal(horn.value, '61');
});

test('a focused geometry field keeps its text across animation frames and is restored after a rejected edit', () => {
  const { document, controller, input } = mount(asymmetricJointFixture());
  const horn = input('sim-hornLength-number');
  const rod = input('sim-rodLength-number');
  controller.setAnimation('wobble', true);
  document.activeElement = horn;
  horn.value = '5';
  rod.value = '5';
  controller.tick(0.016);
  assert.equal(horn.value, '5', 'focused field was rewritten by the frame');
  assert.equal(rod.value, String(controller.getState().layout.rodLength), 'unfocused field was not refreshed');
  horn.value = '-3';
  horn.dispatch('change');
  assert.equal(horn.value, String(controller.getState().layout.hornLength), 'rejected edit did not restore the field');
  document.activeElement = null;
  horn.value = '5';
  controller.setAnimation('wobble', true);
  controller.tick(0.016);
  assert.equal(horn.value, String(controller.getState().layout.hornLength));
});

test('a committed angle or differently formatted entry still counts as untouched and follows a reset', () => {
  const parametric = mount(new Optimizer({}, { topology: 'c3_paired', seed: 37 }).createRandomLayout());
  const turn = parametric.input('sim-base-orientation-number');
  const original = turn.value;
  parametric.document.activeElement = turn;
  // 30 degrees stored in radians does not print back as "30" without formatting.
  turn.value = '30';
  turn.dispatch('change');
  assert.ok(Math.abs(parametric.controller.getState().layout.topologyParameters.base_orientation - Math.PI / 6) < 1e-12);
  assert.equal(turn.value, '30', 'the committed angle was rewritten with a long decimal');
  parametric.input('sim-reset-geometry').dispatch('click');
  assert.equal(turn.value, original, 'focused committed angle field kept the pre-reset value');

  const explicit = mount(asymmetricJointFixture());
  const beta = explicit.input('sim-beta-1-number');
  const horn = explicit.input('sim-hornLength-number');
  const hornRange = explicit.input('sim-hornLength-range');
  const betaBefore = beta.value;
  explicit.document.activeElement = beta;
  beta.value = '60';
  beta.dispatch('change');
  assert.equal(beta.value, '60');
  explicit.input('sim-reset-geometry').dispatch('click');
  assert.equal(beta.value, betaBefore);
  explicit.document.activeElement = horn;
  horn.value = '77.0';
  horn.dispatch('change');
  assert.equal(explicit.controller.getState().layout.hornLength, 77);
  explicit.input('sim-reset-geometry').dispatch('click');
  assert.equal(horn.value, '50', 'a numerically equal but textually different entry stayed touched');
  // A dragged range is committed as well.
  explicit.document.activeElement = hornRange;
  hornRange.value = '66';
  hornRange.dispatch('input');
  assert.equal(explicit.controller.getState().layout.hornLength, 66);
  explicit.input('sim-reset-geometry').dispatch('click');
  assert.equal(hornRange.value, '50');
  assert.equal(horn.value, '50');
  // Uncommitted typing that happens to equal the stored value is not a problem
  // either way; an emptied field is never treated as untouched.
  explicit.document.activeElement = horn;
  horn.value = '';
  explicit.controller.setAnimation('wobble', true);
  explicit.controller.tick(0.016);
  assert.equal(horn.value, '', 'a cleared field was rewritten mid-edit');
});

test('in-progress typing that parses to the shown number is not rewritten by animation frames', () => {
  const { document, controller, input } = mount(asymmetricJointFixture());
  const horn = input('sim-hornLength-number');
  assert.equal(horn.value, '50');
  document.activeElement = horn;
  controller.setAnimation('wobble', true);
  for (const typed of ['50.0', '050', '5e1', '50.']) {
    horn.value = typed;
    controller.tick(0.016);
    assert.equal(horn.value, typed, `typed ${typed} was rewritten while the user was still typing`);
  }
  // The text stays through further frames and the next keystroke lands on it.
  horn.value = '50.05';
  controller.tick(0.016);
  assert.equal(horn.value, '50.05');
  horn.dispatch('change');
  assert.equal(controller.getState().layout.hornLength, 50.05);
});
