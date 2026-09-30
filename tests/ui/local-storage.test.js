import test from 'node:test';
import assert from 'node:assert/strict';
import { loadUI } from './helpers.js';
import { LOCAL_WORKSPACE_KEY } from '../../src/ui/local-workspace.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';

function storage() {
  const entries = new Map();
  return {
    getItem: key => entries.get(key) ?? null,
    setItem: (key, value) => entries.set(key, value),
    removeItem: key => entries.delete(key),
  };
}

const layout = { ...asymmetricJointFixture(), id: 19, topology: 'free', topologyParameters: {} };
const candidate = { layout, torque: 2, speedDemand: 3, coverage: 100,
  feasibility: { passing: true, homePoseSatisfied: true, sampledWorkspaceSatisfied: true, cycleSatisfied: true } };

class FixtureOptimizer {
  constructor() { this.fitness = [candidate]; this.pareto = [candidate]; }
  estimateWork() { return { totalPoses: 1 }; }
  async start() { return { status: 'completed' }; }
  getSelectedCandidate() { return candidate; }
  selectCandidate() {}
  effectiveSettings() { return { ballJointLimitDeg: 180 }; }
}

test('browser save restores inputs and simulator layout after reload, and can be deleted', async () => {
  const localStorage = storage();
  const options = { window: { localStorage, addEventListener() {} } };
  const first = await loadUI(FixtureOptimizer, options);
  first('optSeed').value = '345';
  first('optXMin').value = '-12';
  first('servoTorque1').value = '4.5';
  first('ballJointClamp').checked = true;
  await first('runOptimization').handlers.click();
  first('simZInput').value = '500';
  first('simZInput').handlers.change();
  first('saveLocalWorkspace').handlers.click();
  assert.ok(localStorage.getItem(LOCAL_WORKSPACE_KEY));

  const second = await loadUI(FixtureOptimizer, options);
  assert.equal(second('optSeed').value, '345');
  assert.equal(second('optXMin').value, '-12');
  assert.equal(second('servoTorque1').value, '4.5');
  assert.equal(second('ballJointClamp').checked, true);
  assert.match(second('simCandidateSummary').textContent, /import · free/);
  assert.match(second('simPoseStatus').textContent, /Rejected request.*Z 500/);
  assert.match(second('optStatus').textContent, /rerun optimization/i);

  second('deleteLocalWorkspace').handlers.click();
  assert.equal(localStorage.getItem(LOCAL_WORKSPACE_KEY), null);
  const third = await loadUI(FixtureOptimizer, options);
  assert.notEqual(third('optSeed').value, '345');
});

test('corrupt browser save reports an error without overwriting current inputs', async () => {
  const localStorage = storage();
  const element = await loadUI(FixtureOptimizer, { window: { localStorage, addEventListener() {} } });
  element('optSeed').value = '77';
  localStorage.setItem(LOCAL_WORKSPACE_KEY, '{bad json');
  element('restoreLocalWorkspace').handlers.click();
  assert.equal(element('optSeed').value, '77');
  assert.match(element('optStatus').textContent, /Could not restore browser save/);
});
