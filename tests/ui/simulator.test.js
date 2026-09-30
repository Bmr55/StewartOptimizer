import test from 'node:test';
import assert from 'node:assert/strict';
import { loadUI } from './helpers.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { importLayout } from '../../src/io/layout-import.js';

const source = { ...asymmetricJointFixture(), id: 19, topology: 'free', topologyParameters: {} };
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
  element('referenceLayoutInput').value = JSON.stringify(saved);
  element('simLoadReference').handlers.click();
  assert.match(element('optStatus').textContent, /rodLengthTolerance/);
  assert.match(element('simPoseStatus').textContent, /Accepted request/);
  element('simUseReference').handlers.click();
  const exported = JSON.parse(element('referenceLayoutInput').value);
  assert.equal(exported.home_height, source.homeHeight);
  assert.notEqual(exported.simulator.options.rodLengthTolerance, -1);
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
