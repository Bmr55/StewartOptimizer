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
import { buildSceneGeometry, OVERLAY_DEFAULTS, SCENE_COLORS } from '../../src/simulator/scene.js';

const plainLines = lines => JSON.parse(JSON.stringify(lines));

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

test('Load optimizer reference reports the importer message for bad JSON and typed simulator fields, keeping the candidate', async () => {
  const element = await loadUI(FixtureOptimizer);
  await element('runOptimization').handlers.click();
  element('simUseReference').handlers.click();
  const saved = JSON.parse(element('referenceLayoutInput').value);
  const cases = [
    ['{', /Layout JSON is invalid\./],
    [JSON.stringify({ ...saved, simulator: { ...saved.simulator, options: { ...saved.simulator.options, ballJointLimitDeg: null } } }),
      /simulator.options: ballJointLimitDeg must be a finite angle/],
    [JSON.stringify({ ...saved, simulator: { ...saved.simulator, options: { ...saved.simulator.options, servoRangeRad: 'abc' } } }),
      /servoRangeRad must contain two finite bounds/],
    [JSON.stringify({ ...saved, simulator: { ...saved.simulator, camera: 'abc' } }), /simulator.camera must be an object/],
    [JSON.stringify({ ...saved, simulator: { ...saved.simulator, animation: { pattern: 'wobble', speed: -1 } } }),
      /simulator.animation.speed must be a positive finite number/],
    [JSON.stringify({ ...saved, topology: 'free', topology_parameters: 'garbage' }), /topology_parameters must be an object/],
    [JSON.stringify({ ...saved, schema_version: '2' }), /schema_version "2" is unsupported/],
  ];
  for (const [text, expected] of cases) {
    element('referenceLayoutInput').value = text;
    element('simLoadReference').handlers.click();
    assert.match(element('optStatus').textContent, expected);
    assert.match(element('simCandidateSummary').textContent, /Candidate 19/, `candidate lost after ${expected}`);
    assert.match(element('simPoseStatus').textContent, /Accepted request/);
  }
  // A valid file with a null joint limit removed still loads and keeps 45 degree limits.
  element('referenceLayoutInput').value = JSON.stringify(saved);
  element('simLoadReference').handlers.click();
  assert.match(element('simCandidateSummary').textContent, /import/);
});

test('overlay toggles round-trip through the optimizer reference, simulator JSON and a browser save', async () => {
  const downloads = [];
  const element = await loadUI(FixtureOptimizer, { downloadFile: (data, name) => downloads.push({ data, name }) });
  await element('runOptimization').handlers.click();
  const controller = element.app.simulatorController;
  const toggle = (ui, id, checked) => { ui(id).checked = checked; ui(id).handlers.change({ target: ui(id) }); };
  assert.equal(element('simOverlayServoArcs').checked, false, 'servo arcs start off');
  for (const id of ['simOverlayPlatformAxes', 'simOverlayWorldAxes']) assert.equal(element(id).checked, true, id);
  const saved = { ...OVERLAY_DEFAULTS, servoArcs: true, worldAxes: false };
  toggle(element, 'simOverlayServoArcs', true);
  toggle(element, 'simOverlayWorldAxes', false);
  element('simUseReference').handlers.click();
  assert.deepEqual(JSON.parse(element('referenceLayoutInput').value).simulator.overlays, saved);
  element('simDownload').handlers.click();
  assert.equal(downloads.at(-1).name, 'stewart_simulator.json');
  assert.deepEqual(JSON.parse(downloads.at(-1).data).simulator.overlays, saved);
  toggle(element, 'simOverlayServoArcs', false);
  toggle(element, 'simOverlayWorldAxes', true);
  toggle(element, 'simOverlayPlatformAxes', false);
  element('simLoadReference').handlers.click();
  assert.deepEqual(controller.getState().overlays, saved);
  assert.equal(element('simOverlayServoArcs').checked, true);
  assert.equal(element('simOverlayWorldAxes').checked, false);
  assert.equal(element('simOverlayPlatformAxes').checked, true);
  // A file saved before overlays existed keeps the current toggles.
  const legacy = JSON.parse(element('referenceLayoutInput').value);
  delete legacy.simulator.overlays;
  toggle(element, 'simOverlayPlatformAxes', false);
  element('referenceLayoutInput').value = JSON.stringify(legacy);
  element('simLoadReference').handlers.click();
  assert.deepEqual(controller.getState().overlays, { ...saved, platformAxes: false });

  const storage = new Map();
  const window = { localStorage: { getItem: key => storage.get(key) ?? null, setItem: (key, value) => storage.set(key, value),
    removeItem: key => storage.delete(key) }, addEventListener() {} };
  const restored = await loadUI(FixtureOptimizer, { window });
  await restored('runOptimization').handlers.click();
  toggle(restored, 'simOverlayPlatformAxes', false);
  restored('saveLocalWorkspace').handlers.click();
  toggle(restored, 'simOverlayPlatformAxes', true);
  restored('restoreLocalWorkspace').handlers.click();
  assert.deepEqual(restored.app.simulatorController.getState().overlays, { ...OVERLAY_DEFAULTS, platformAxes: false });
  assert.equal(restored('simOverlayPlatformAxes').checked, false);
  assert.equal(restored('simOverlayWorldAxes').checked, true);
});

test('reachability cloud settings round-trip through simulator JSON without the samples', async () => {
  const downloads = [];
  const element = await loadUI(FixtureOptimizer, { downloadFile: (data, name) => downloads.push({ data, name }) });
  await element('runOptimization').handlers.click();
  const controller = element.app.simulatorController;
  controller.setReachabilityCloud({ enabled: true, sampleCount: 256, mode: 'slice', sliceZ: 4 });
  element('simDownload').handlers.click();
  const file = downloads.at(-1).data;
  const saved = JSON.parse(file).simulator;
  assert.deepEqual(saved.reachability, { sampleCount: 256, mode: 'slice', sliceZ: 4 });
  assert.equal(saved.overlays.reachabilityCloud, true);
  assert.equal(saved.reachabilityCloud, undefined, 'samples are not saved');
  controller.setReachabilityCloud({ enabled: false, sampleCount: 4096, mode: 'cloud', sliceZ: 0 });
  element('referenceLayoutInput').value = file;
  element('simLoadReference').handlers.click();
  assert.deepEqual(controller.getState().reachability, { sampleCount: 256, mode: 'slice', sliceZ: 4 });
  assert.equal(controller.getState().overlays.reachabilityCloud, true);
  assert.equal(controller.getState().reachabilityCloud.total, 256, 'the load starts a fresh sweep');
  assert.equal(element('simOverlayReachabilityCloud').checked, true);
  assert.equal(element('simReachabilitySamples').value, '256');
  assert.equal(element('simReachabilitySlice').checked, true);
  assert.equal(element('simReachabilitySliceZ').value, '4');
  // A bad setting rejects the file by name; a file without the block keeps the current settings.
  const bad = JSON.parse(file);
  bad.simulator.reachability.sampleCount = 999;
  element('referenceLayoutInput').value = JSON.stringify(bad);
  element('simLoadReference').handlers.click();
  assert.match(element('optStatus').textContent, /simulator\.reachability\.sampleCount must be one of 256, 1024, 4096/);
  const legacy = JSON.parse(file);
  delete legacy.simulator.reachability;
  delete legacy.simulator.overlays;
  controller.setReachabilityCloud({ enabled: false, sampleCount: 1024 });
  element('referenceLayoutInput').value = JSON.stringify(legacy);
  element('simLoadReference').handlers.click();
  assert.deepEqual(controller.getState().reachability, { sampleCount: 1024, mode: 'slice', sliceZ: 4 });
  assert.equal(controller.getState().reachabilityCloud, null);
});

// The run's workspace ranges as `effective_settings.bounds` carries them: mm and degrees.
const SAMPLE_BOUNDS = { x: { min: -40, max: 40, step: 40 }, y: { min: -40, max: 40, step: 40 },
  z: { min: -20, max: 40, step: 30 }, rx: { min: -12, max: 12, step: 12 }, ry: { min: -12, max: 12, step: 12 },
  rz: { min: -8, max: 8, step: 8 } };
class RangedOptimizer extends FixtureOptimizer {
  effectiveSettings() { return { ...super.effectiveSettings(), bounds: SAMPLE_BOUNDS }; }
}
const boxEdges = state => buildSceneGeometry(state).lines.filter(line => line.color === SCENE_COLORS.workspace);

test('a candidate draws its run workspace box, which simulator JSON carries; JSON without ranges draws none', async () => {
  const element = await loadUI(RangedOptimizer);
  await element('runOptimization').handlers.click();
  const controller = element.app.simulatorController;
  // The box is off by default.
  assert.deepEqual(boxEdges(controller.getState()), []);
  let state = controller.setOverlays({ workspaceBox: true });
  assert.deepEqual(state.workspaceRanges.z, { min: -20, max: 40 });
  assert.ok(Math.abs(state.workspaceRanges.rx.max - 12 * Math.PI / 180) < 1e-15);
  const edges = boxEdges(state);
  assert.equal(edges.length, 12);
  const home = state.layout.homeHeight;
  assert.ok(edges.some(edge => edge.from.join() === [-40, -40, home - 20].join()));
  assert.ok(edges.some(edge => edge.to.join() === [40, 40, home + 40].join()));
  element('simUseReference').handlers.click();
  const transfer = JSON.parse(element('referenceLayoutInput').value);
  const withoutSteps = Object.fromEntries(Object.entries(SAMPLE_BOUNDS).map(([axis, { min, max }]) => [axis, { min, max }]));
  assert.deepEqual(transfer.simulator.workspaceRanges, withoutSteps);

  // A fresh page with no run restores the same box from the saved block.
  const fresh = await loadUI(FixtureOptimizer);
  const load = document => {
    fresh('referenceLayoutInput').value = JSON.stringify(document);
    fresh('simLoadReference').handlers.click();
    return fresh.app.simulatorController.getState();
  };
  assert.deepEqual(plainLines(boxEdges(load(transfer))), plainLines(edges));
  // Without the block, the run's bounds stand in; without either, there is no box and no error.
  const { workspaceRanges, ...simulator } = transfer.simulator;
  assert.equal(boxEdges(load({ ...transfer, simulator })).length, 12);
  const { run, ...bare } = transfer;
  const status = fresh('optStatus').textContent;
  state = load({ ...bare, simulator });
  assert.equal(state.workspaceRanges, null);
  assert.deepEqual(boxEdges(state), []);
  assert.match(fresh('simCandidateSummary').textContent, /import/);
  assert.equal(fresh('optStatus').textContent, status, 'the load reported an error');
  state = load(layoutToJSON(asymmetricJointFixture()));
  assert.equal(state.workspaceRanges, null);
  assert.equal(JSON.parse((fresh('simUseReference').handlers.click(), fresh('referenceLayoutInput').value))
    .simulator.workspaceRanges, null);
});

test('a candidate carries its run payload into the simulator, simulator JSON and a reload', async () => {
  const element = await loadUI(FixtureOptimizer);
  await element('runOptimization').handlers.click();
  const controller = element.app.simulatorController;
  const toggle = (id, checked) => { element(id).checked = checked; element(id).handlers.change({ target: element(id) }); };
  // The fixture run's requirements carry mass_kg 1, so the loads toggle is usable.
  assert.deepEqual(controller.getState().loadModel, { mass_kg: 1 });
  assert.equal(controller.getState().loads.motion, 'static');
  assert.equal(element('simOverlayLoads').disabled, false);
  assert.equal(element('simLoadsHint').hidden, true);
  assert.equal(element('simOverlayLoads').checked, false);
  toggle('simOverlayLoads', true);
  assert.equal(controller.getState().overlays.loads, true);
  element('simUseReference').handlers.click();
  const saved = JSON.parse(element('referenceLayoutInput').value);
  assert.deepEqual(saved.simulator.loadModel, { mass_kg: 1 });
  assert.equal(saved.simulator.overlays.loads, true);
  // A saved model wins over the run's; one without a model or a run has no loads.
  saved.simulator.loadModel = { mass_kg: 4, servo_torque_rating_nm: 0.3 };
  element('referenceLayoutInput').value = JSON.stringify(saved);
  element('simLoadReference').handlers.click();
  assert.deepEqual(controller.getState().loadModel, { mass_kg: 4, servo_torque_rating_nm: 0.3 });
  assert.ok(controller.getState().loads.utilization.every(Number.isFinite));
  delete saved.simulator.loadModel;
  element('referenceLayoutInput').value = JSON.stringify(saved);
  element('simLoadReference').handlers.click();
  assert.deepEqual(controller.getState().loadModel, { mass_kg: 1 }, 'the run supplies the model');
  delete saved.run;
  element('referenceLayoutInput').value = JSON.stringify(saved);
  element('simLoadReference').handlers.click();
  assert.equal(controller.getState().loadModel, null);
  assert.equal(controller.getState().loads, null);
  assert.equal(element('simOverlayLoads').disabled, true);
  assert.equal(element('simLoadsHint').hidden, false);
});
