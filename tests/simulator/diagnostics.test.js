import test from 'node:test';
import assert from 'node:assert/strict';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { resolveMounting } from '../../src/model/mounting.js';
import { evaluatePose } from '../../src/model/pose.js';
import { createSimulatorController, HOME_POSE } from '../../src/simulator/controller.js';
import { buildPoseDiagnostics, mountSimulatorDiagnostics } from '../../src/simulator/diagnostics.js';
import { createFakeDocument } from './helpers.js';
import { buildSceneGeometry } from '../../src/simulator/renderer.js';

test('diagnostics use active evaluator angles, rod deviations, limits, and retained pose', () => {
  const controller = createSimulatorController();
  const layout = asymmetricJointFixture();
  let state = controller.loadLayout(layout, { options: { ballJointLimitDeg: 180,
    lowerBallJointLimitDeg: 120, upperBallJointLimitDeg: 130, rodLengthTolerance: 0.5 } });
  let diagnostics = buildPoseDiagnostics(state);
  assert.equal(diagnostics.legs.length, 6);
  assert.equal(diagnostics.lowerLimitDeg, 120);
  assert.equal(diagnostics.upperLimitDeg, 130);
  for (let index = 0; index < 6; index++) {
    assert.ok(Math.abs(diagnostics.legs[index].lowerDeg
      - state.assessment.jointAngles.lower[index] * 180 / Math.PI) < 1e-9);
    assert.equal(diagnostics.legs[index].rodDeviationMm,
      state.assessment.rodLengths[index] - layout.rodLength);
  }
  state = controller.requestPose({ z: 100 });
  diagnostics = buildPoseDiagnostics(state);
  assert.equal(diagnostics.requestedStatus, 'rejected');
  assert.equal(diagnostics.requested.z, 100);
  assert.deepEqual(diagnostics.accepted, HOME_POSE);
  assert.deepEqual(diagnostics.affectedLegs,
    [...new Set(state.assessment.violations.filter(item => item.leg != null).map(item => item.leg + 1))]);
  const scene = buildSceneGeometry(state);
  assert.ok(scene.lines.some(line => line.color[0] === 1 && line.color[1] === 0.26));
});

test('lower-only and upper-only failures identify the correct joint and leg', () => {
  for (const joint of ['lower', 'upper']) {
    const layout = asymmetricJointFixture();
    const mounting = resolveMounting(layout).mounting;
    mounting[joint][0] = { source: 'supplied',
      direction: mounting[joint][0].direction.map(value => -value) };
    layout.mounting = mounting;
    const state = createSimulatorController().loadLayout(layout,
      { options: { lowerBallJointLimitDeg: 30, upperBallJointLimitDeg: 30 } });
    const diagnostics = buildPoseDiagnostics(state);
    assert.equal(diagnostics.requestedStatus, 'rejected');
    assert.equal(diagnostics.accepted, null);
    assert.deepEqual(diagnostics.affectedLegs, [1]);
    assert.equal(diagnostics.legs[0][`${joint}Failed`], true);
    assert.equal(diagnostics.legs[0][joint === 'lower' ? 'upperFailed' : 'lowerFailed'], false);
    assert.equal(diagnostics.legs[0].failures[0].type, 'ballJoint');
  }
});

test('rod tolerance is inclusive and conditioning failures stay global', () => {
  const layout = asymmetricJointFixture();
  const measured = evaluatePose(layout, {}, { ballJointLimitDeg: 180, recordLegData: true });
  const deviation = Math.max(...measured.rodLengths.map(length => Math.abs(length - layout.rodLength)));
  assert.ok(deviation > 0);
  assert.equal(evaluatePose(layout, {}, { ballJointLimitDeg: 180,
    rodLengthTolerance: deviation }).reachable, true);
  assert.equal(evaluatePose(layout, {}, { ballJointLimitDeg: 180,
    rodLengthTolerance: deviation / 2 }).violations[0].type, 'rodLength');
  for (const invalid of [-1, Infinity, NaN]) {
    assert.throws(() => evaluatePose(layout, {}, { rodLengthTolerance: invalid }), /rodLengthTolerance/);
  }
  const controller = createSimulatorController();
  const state = controller.loadLayout(layout,
    { options: { ballJointLimitDeg: 180, conditionLimit: 1 } });
  const diagnostics = buildPoseDiagnostics(state);
  assert.equal(diagnostics.requestedStatus, 'rejected');
  assert.equal(diagnostics.globalFailures[0].type, 'conditionLimit');
  assert.equal(diagnostics.affectedLegs.length, 0);
});

test('diagnostics refresh on settings, animation, and geometry changes', () => {
  const controller = createSimulatorController();
  const updates = [];
  controller.subscribe(state => updates.push(buildPoseDiagnostics(state)));
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  controller.requestPose({ z: 10 });
  controller.setOptions({ upperBallJointLimitDeg: 0 });
  controller.setAnimation('pingpong', true);
  controller.tick(0.1, { apexMm: 100, frequencyHz: 2.5 });
  const changed = asymmetricJointFixture();
  changed.homeHeight += 5;
  controller.loadLayout(changed, { source: { kind: 'editable' }, options: { ballJointLimitDeg: 180 } });
  assert.ok(updates.some(item => item.requested.z === 10));
  assert.ok(updates.some(item => item.upperLimitDeg === 0));
  assert.ok(updates.some(item => item.source === 'animation' && item.requestedStatus === 'rejected'));
  assert.equal(updates.at(-1).source, 'load');
  assert.equal(updates.at(-1).requestedStatus, 'accepted');
});

test('a focused but untouched limit field follows a load or settings change', () => {
  const document = createFakeDocument();
  const controller = createSimulatorController();
  const options = { ballJointLimitDeg: 180, lowerBallJointLimitDeg: 120, upperBallJointLimitDeg: 130, rodLengthTolerance: 0.5 };
  controller.loadLayout(asymmetricJointFixture(), { options });
  mountSimulatorDiagnostics({ document, controller, host: document.createElement('section') });
  const lower = document.getElementById('simLowerJointLimit');
  document.activeElement = lower;
  lower.value = '3';
  lower.dispatch('change');
  assert.equal(controller.getState().options.lowerBallJointLimitDeg, 3);
  assert.equal(lower.value, '3');
  controller.loadLayout(asymmetricJointFixture(), { options });
  assert.equal(lower.value, '120', 'focused field kept the previous limit after a load');
  controller.setOptions({ lowerBallJointLimitDeg: 45 });
  assert.equal(lower.value, '45');
  lower.value = '4';
  controller.setAnimation('wobble', true);
  controller.tick(0.016);
  assert.equal(lower.value, '4', 'uncommitted typing was overwritten');
});

test('focused limit fields keep their text across animation frames and are restored after a rejected edit', () => {
  const document = createFakeDocument();
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180,
    lowerBallJointLimitDeg: 120, upperBallJointLimitDeg: 130, rodLengthTolerance: 0.5 } });
  mountSimulatorDiagnostics({ document, controller, host: document.createElement('section') });
  const lower = document.getElementById('simLowerJointLimit');
  const tolerance = document.getElementById('simRodTolerance');
  assert.equal(lower.value, '120');
  controller.setAnimation('wobble', true);
  document.activeElement = lower;
  lower.value = '3';
  tolerance.value = '9';
  controller.tick(0.016);
  assert.equal(lower.value, '3', 'focused field was rewritten by the frame');
  assert.equal(tolerance.value, '0.5', 'unfocused field was not refreshed');
  lower.dispatch('change');
  assert.equal(controller.getState().options.lowerBallJointLimitDeg, 3);
  lower.value = '-1';
  lower.dispatch('change');
  assert.equal(document.getElementById('simDiagnosticError').hidden, false);
  assert.equal(lower.value, '3', 'rejected edit did not restore the field');
  document.activeElement = null;
  lower.value = '7';
  controller.setAnimation('wobble', true);
  controller.tick(0.016);
  assert.equal(lower.value, '3');
});
