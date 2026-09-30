import test from 'node:test';
import assert from 'node:assert/strict';
import { loadUI, sampleText } from './helpers.js';
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

test('cycle sampling is saved, and saves from before that control still restore', async () => {
  const { captureLocalWorkspace, parseLocalWorkspace, applyLocalWorkspace } = await import('../../src/ui/local-workspace.js');
  const values = new Map();
  const document = { getElementById: id => {
    if (!values.has(id)) values.set(id, { value: '', checked: false });
    return values.get(id);
  } };
  document.getElementById('optCycleSampling').value = 'uniform-64';
  const saved = captureLocalWorkspace(document);
  assert.equal(saved.inputs.optCycleSampling, 'uniform-64');
  delete saved.inputs.optCycleSampling;
  document.getElementById('optCycleSampling').value = 'adaptive';
  applyLocalWorkspace(document, parseLocalWorkspace(JSON.stringify(saved)));
  assert.equal(document.getElementById('optCycleSampling').value, 'adaptive');
});

test('a run started before the sample fetch settles is not reset by the automatic restore', async () => {
  const localStorage = storage();
  const windowOptions = { window: { localStorage, addEventListener() {} } };
  const first = await loadUI(FixtureOptimizer, windowOptions);
  await first('runOptimization').handlers.click();
  first('saveLocalWorkspace').handlers.click();
  assert.ok(localStorage.getItem(LOCAL_WORKSPACE_KEY));

  let finishRun;
  const gate = new Promise(resolve => { finishRun = resolve; });
  class SlowOptimizer extends FixtureOptimizer {
    async start() {
      this.running = true;
      try { await gate; } finally { this.running = false; }
      return { status: 'completed' };
    }
  }
  let resolveSample;
  const sample = new Promise(resolve => { resolveSample = resolve; });
  const element = await loadUI(SlowOptimizer, { ...windowOptions, awaitReady: false,
    loadDefaultRequirements: () => sample });
  element('requirementsInput').value = sampleText;
  const run = element('runOptimization').handlers.click();
  await Promise.resolve();
  assert.notEqual(element('runPhase').textContent, 'Idle');
  resolveSample(sampleText);
  await element.app.ready;
  assert.notEqual(element('runPhase').textContent, 'Idle');
  assert.equal(element('cancelOptimization').disabled, false);
  assert.match(element('optStatus').textContent, /not restored automatically/i);
  finishRun();
  await run;
  assert.doesNotMatch(element('optStatus').textContent, /TypeError|Cannot read/);
  assert.match(element('simCandidateSummary').textContent, /Candidate 19/);
  assert.equal(element('runOptimization').disabled, false);

  element('restoreLocalWorkspace').handlers.click();
  assert.match(element('optStatus').textContent, /Local workspace restored/);
});
