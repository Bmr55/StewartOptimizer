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
