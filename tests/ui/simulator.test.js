import test from 'node:test';
import assert from 'node:assert/strict';
import { loadUI } from './helpers.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { importLayout } from '../../src/io/layout-import.js';
import { layoutToJSON } from '../../src/io/results.js';
import { resolveMounting } from '../../src/model/mounting.js';
import { createGeometryEditor } from '../../src/simulator/geometry-editor.js';
import { DEFAULT_BALL_JOINT_LIMIT_DEG } from '../../src/contracts.js';
import { NUMERICAL_RECIPROCAL_CUTOFF } from '../../src/model/conditioning.js';

const source = { ...asymmetricJointFixture(), id: 19, topology: 'free', topologyParameters: {} };
// Evaluated candidates carry their resolved mounting, as evaluateLayout leaves it.
source.mounting = resolveMounting(source).mounting;
const candidate = { layout: source, torque: 2, speedDemand: 3, coverage: 100,
  feasibility: { passing: true, homePoseSatisfied: true, sampledWorkspaceSatisfied: true, cycleSatisfied: true } };

class FixtureOptimizer {
  constructor(requirements, options) {
    this.options = options;
    this.fitness = [candidate];
    this.pareto = [candidate];
  }
  estimateWork() { return { totalPoses: 1 }; }
  async start() { return { status: 'completed' }; }
  getSelectedCandidate() { return candidate; }
  selectCandidate() {}
  effectiveSettings() { return { ballJointLimitDeg: 180, conditionLimit: null,
    requirements: { mass_kg: 1 }, populationSize: 4, generations: 1 }; }
}

test('selected candidate transfers to simulator and reference JSON reimports without geometry drift', async () => {
  const element = await loadUI(FixtureOptimizer);
  await element('runOptimization').handlers.click();
  assert.match(element('simRendererError').textContent, /WebGL2.*optimization remains available/i);
  assert.match(element('simCandidateSummary').textContent, /Candidate 19/);
  assert.match(element('simCandidateSelect').innerHTML, /Candidate 19/);
  assert.match(element('simPoseStatus').textContent, /Accepted request/);
  element('simulateTab').handlers.click();
  element('simZInput').value = '500';
  element('simZInput').handlers.change();
  assert.match(element('simPoseStatus').textContent, /Rejected request.*invalidGeometry/);
  assert.match(element('simAcceptedPose').textContent, /Z 0 mm/);
  element('simUseReference').handlers.click();
  const transfer = JSON.parse(element('referenceLayoutInput').value);
  assert.equal(transfer.id, 19);
  assert.equal(transfer.run.effective_settings.populationSize, 4);
  assert.equal(transfer.simulator.requested.z, 500);
  assert.equal(transfer.simulator.accepted.z, 0);
  const reimported = importLayout(transfer).layout;
  for (const field of ['baseAnchors', 'platformAnchors', 'betaAngles', 'hornLength',
    'rodLength', 'homeHeight']) assert.deepEqual(reimported[field], source[field]);
  reimported.servoRangeRad.forEach((value, index) =>
    assert.ok(Math.abs(value - source.servoRangeRad[index]) < 1e-14));
  element('simulateTab').handlers.click();
  element('simLoadReference').handlers.click();
  assert.match(element('simPoseStatus').textContent, /Rejected request.*Z 500/);
  assert.match(element('simAcceptedPose').textContent, /Z 0 mm/);
});

test('a simulator saved before animation reloads with a playable pattern', async () => {
  const element = await loadUI(FixtureOptimizer);
  await element('runOptimization').handlers.click();
  element('simUseReference').handlers.click();
  const saved = JSON.parse(element('referenceLayoutInput').value);
  assert.equal(saved.simulator.animation.pattern, 'none');
  for (const pattern of ['none', 'wobble', 'pingpong', 'rotate', 'tilt', 'helical', 'unknown']) {
    saved.simulator.animation.pattern = pattern;
    saved.simulator.animation.speed = 2;
    element('referenceLayoutInput').value = JSON.stringify(saved);
    element('simLoadReference').handlers.click();
    assert.equal(element('simPattern').value, ['none', 'unknown'].includes(pattern) ? 'wobble' : pattern);
    assert.equal(element('simSpeed').value, '2');
    assert.equal(element('simPlay').textContent, 'Play');
    element('simPlay').handlers.click();
    assert.equal(element('simPlay').textContent, 'Pause');
  }
});

test('loading a reference with invalid simulator options reports the error and keeps the loaded candidate', async () => {
  const element = await loadUI(FixtureOptimizer);
  await element('runOptimization').handlers.click();
  element('simUseReference').handlers.click();
  const saved = JSON.parse(element('referenceLayoutInput').value);
  saved.home_height = 555;
  saved.simulator.options.rodLengthTolerance = -1;
  saved.run.effective_settings.populationSize = 999;
  element('referenceLayoutInput').value = JSON.stringify(saved);
  element('simLoadReference').handlers.click();
  assert.match(element('optStatus').textContent, /rodLengthTolerance/);
  assert.match(element('simPoseStatus').textContent, /Accepted request/);
  element('simUseReference').handlers.click();
  const exported = JSON.parse(element('referenceLayoutInput').value);
  assert.equal(exported.home_height, source.homeHeight);
  assert.notEqual(exported.simulator.options.rodLengthTolerance, -1);
  assert.equal(exported.run.effective_settings.populationSize, 4, 'rejected file replaced the run metadata');
  assert.deepEqual(exported.simulator.accepted, { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 });
});

test('repeated reference loads keep a single Imported reference option', async () => {
  const element = await loadUI(FixtureOptimizer);
  await element('runOptimization').handlers.click();
  element('simUseReference').handlers.click();
  for (let i = 0; i < 3; i++) element('simLoadReference').handlers.click();
  const options = element('simCandidateSelect').innerHTML.match(/<option /g);
  assert.equal(options.length, 2);
  assert.equal(element('simCandidateSelect').innerHTML.match(/Imported reference/g).length, 1);
  assert.match(element('simCandidateSelect').innerHTML, /Candidate 19/);
});

test('simulator JSON exports the mounting of the edited geometry, not the candidate it started from', async () => {
  const element = await loadUI(FixtureOptimizer);
  await element('runOptimization').handlers.click();
  const editor = createGeometryEditor(element.app.simulatorController);
  editor.edit({ type: 'scalar', field: 'homeHeight', value: source.homeHeight + 20 });
  element('simUseReference').handlers.click();
  const transfer = JSON.parse(element('referenceLayoutInput').value);
  const state = element.app.simulatorController.getState();
  assert.equal(state.assessment.reachable, true);
  const expected = resolveMounting(state.layout).mounting;
  assert.deepEqual(transfer.mounting, expected);
  assert.deepEqual(state.assessment.mounting, expected);
  assert.notDeepEqual(transfer.mounting.lower[0].direction, resolveMounting(source).mounting.lower[0].direction);
});

test('marker and trace checkboxes follow the controller on snapshot load and restore', async () => {
  const element = await loadUI(FixtureOptimizer);
  await element('runOptimization').handlers.click();
  const controller = element.app.simulatorController;
  assert.equal(element('simMarkers').checked, true);
  assert.equal(element('simTraces').checked, false);
  element('simMarkers').checked = false;
  element('simMarkers').handlers.change({ target: element('simMarkers') });
  element('simTraces').checked = true;
  element('simTraces').handlers.change({ target: element('simTraces') });
  assert.equal(controller.getState().markers, false);
  assert.equal(controller.getState().tracesEnabled, true);
  element('simUseReference').handlers.click();
  element('simMarkers').checked = true;
  element('simMarkers').handlers.change({ target: element('simMarkers') });
  element('simTraces').checked = false;
  element('simTraces').handlers.change({ target: element('simTraces') });
  element('simLoadReference').handlers.click();
  assert.equal(controller.getState().markers, false);
  assert.equal(controller.getState().tracesEnabled, true);
  assert.equal(element('simMarkers').checked, false);
  assert.equal(element('simTraces').checked, true);
  const storage = new Map();
  const window = { localStorage: { getItem: key => storage.get(key) ?? null, setItem: (key, value) => storage.set(key, value),
    removeItem: key => storage.delete(key) }, addEventListener() {} };
  const restored = await loadUI(FixtureOptimizer, { window });
  await restored('runOptimization').handlers.click();
  restored('simMarkers').checked = false;
  restored('simMarkers').handlers.change({ target: restored('simMarkers') });
  restored('saveLocalWorkspace').handlers.click();
  restored('simMarkers').checked = true;
  restored('simMarkers').handlers.change({ target: restored('simMarkers') });
  restored('restoreLocalWorkspace').handlers.click();
  assert.equal(restored.app.simulatorController.getState().markers, false);
  assert.equal(restored('simMarkers').checked, false);
  assert.equal(restored('simTraces').checked, false);
});

test('the Imported reference option reloads the import and the select is enabled without a run', async () => {
  const element = await loadUI(FixtureOptimizer);
  const controller = element.app.simulatorController;
  element('referenceLayoutInput').value = JSON.stringify(layoutToJSON(source));
  element('simLoadReference').handlers.click();
  assert.equal(element('simCandidateSelect').disabled, false);
  assert.match(element('simCandidateSelect').innerHTML, /Imported reference/);
  assert.equal(controller.getState().source.kind, 'import');
  await element('runOptimization').handlers.click();
  assert.equal(controller.getState().source.kind, 'candidate');
  assert.doesNotMatch(element('simCandidateSelect').innerHTML, /Imported reference/);
  element('simUseReference').handlers.click();
  element('simLoadReference').handlers.click();
  assert.equal(controller.getState().source.kind, 'import');
  element('simCandidateSelect').value = '19';
  element('simCandidateSelect').handlers.change();
  assert.deepEqual(controller.getState().source, { kind: 'candidate', candidateId: 19 });
  assert.equal(element('simCandidateSelect').value, '19');
  element('simZInput').value = '3';
  element('simZInput').handlers.change();
  element('simCandidateSelect').value = '';
  element('simCandidateSelect').handlers.change();
  assert.equal(controller.getState().source.kind, 'import');
  assert.equal(controller.getState().source.candidateId, 19);
  assert.equal(element('simCandidateSelect').value, '');
  assert.equal(element('simCandidateSelect').innerHTML.match(/Imported reference/g).length, 1);
  assert.match(element('simCandidateSummary').textContent, /import/);
  // A new run drops the imported entry, so the empty value no longer reloads anything.
  await element('runOptimization').handlers.click();
  assert.doesNotMatch(element('simCandidateSelect').innerHTML, /Imported reference/);
  assert.equal(controller.getState().source.kind, 'candidate');
});

test('simulator defaults come from the shared contracts, not local literals', async () => {
  class BareSettingsOptimizer extends FixtureOptimizer {
    effectiveSettings() { return { requirements: {}, populationSize: 4, generations: 1 }; }
  }
  const element = await loadUI(BareSettingsOptimizer);
  await element('runOptimization').handlers.click();
  const options = element.app.simulatorController.getState().options;
  assert.equal(options.ballJointLimitDeg, DEFAULT_BALL_JOINT_LIMIT_DEG);
  assert.equal(options.lowerBallJointLimitDeg, DEFAULT_BALL_JOINT_LIMIT_DEG);
  assert.equal(options.upperBallJointLimitDeg, DEFAULT_BALL_JOINT_LIMIT_DEG);
  assert.match(element('simConditionPolicy').textContent, new RegExp(`mandatory reciprocal cutoff: ${NUMERICAL_RECIPROCAL_CUTOFF}\\.`));
  assert.match(element('simConditionPolicy').textContent, new RegExp(`Joint limits ${DEFAULT_BALL_JOINT_LIMIT_DEG}° lower`));
});

test('reset, play, and speed controls surface controller errors instead of throwing', async () => {
  const element = await loadUI(FixtureOptimizer);
  assert.doesNotThrow(() => element('simResetPose').handlers.click());
  assert.match(element('simPoseStatus').textContent, /Load a layout before requesting a pose/);
  await element('runOptimization').handlers.click();
  assert.match(element('simPoseStatus').textContent, /Accepted request/);
  element('simSpeed').value = '';
  assert.doesNotThrow(() => element('simSpeed').handlers.change({ target: element('simSpeed') }));
  assert.match(element('simPoseStatus').textContent, /Animation speed must be positive/);
  element('simPattern').value = 'wobble';
  assert.doesNotThrow(() => element('simPlay').handlers.click());
  assert.match(element('simPoseStatus').textContent, /Animation speed must be positive/);
  assert.equal(element('simPlay').textContent, 'Play');
  element('simSpeed').value = '2';
  element('simSpeed').handlers.change({ target: element('simSpeed') });
  element('simPlay').handlers.click();
  assert.equal(element('simPlay').textContent, 'Pause');
});
