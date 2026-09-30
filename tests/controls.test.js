import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../optimizer.js';
import { loadUI, sampleText } from './ui-helper.js';

test('run keeps explicit overrides and refreshes untouched defaults from JSON', async () => {
  let captured;
  class StubOptimizer {
    constructor(requirements, options) { captured = { requirements, options }; this.pareto = []; this.fitness = []; }
    estimateWork() { return { totalPoses: 8 }; }
    start(done) { done?.(); return Promise.resolve({ status: 'completed' }); }
  }
  const element = await loadUI(StubOptimizer);
  element('optXStep').value = '40';
  element('optXMin').value = '-5';
  element('ballJointLimit').value = '10';
  const input = JSON.parse(sampleText);
  input.workspace.y_range_mm = [-10, 20];
  element('requirementsInput').value = JSON.stringify(input);
  await element('runOptimization').handlers.click();
  assert.equal(captured.options.ranges.x.step, 40);
  assert.equal(captured.options.ranges.x.min, -5);
  assert.equal(captured.options.ballJointLimitDeg, 10);
  assert.equal(captured.options.ranges.y.min, -10);
  assert.equal(captured.options.ranges.y.max, 20);
  await element('loadSampleRequirements').handlers.click();
  assert.equal(Number(element('optXMin').value), -40);
  assert.equal(Number(element('ballJointLimit').value), 52);
});

test('explicit joint option takes precedence, including zero', () => {
  assert.equal(new Optimizer({ ball_joint_max_deg: 52 }, { ballJointLimitDeg: 10 }).ballJointLimitDeg, 10);
  assert.equal(new Optimizer({ ball_joint_max_deg: 52 }, { ballJointLimitDeg: 0 }).ballJointLimitDeg, 0);
  assert.equal(new Optimizer({ ball_joint_max_deg: 52 }).ballJointLimitDeg, 52);
});
