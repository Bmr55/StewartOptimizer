import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../optimizer.js';
import { parseRequirements } from '../requirements.js';
import { computeWorkspace, estimateWorkspaceSize } from '../workspace.js';
import { jointFixture } from './fixture.js';
import { sampleText } from './ui-helper.js';

test('default sample has a practical bounded budget', () => {
  const { normalized, workspace } = parseRequirements(sampleText);
  assert.equal(estimateWorkspaceSize(workspace), 729);
  assert.equal(new Optimizer(normalized, { ranges: workspace }).estimateWork().totalPoses, 52488);
  assert.throws(() => new Optimizer(normalized, { ranges: workspace, generations: 10000 }).estimateWork(), /1,000,000/);
  assert.throws(() => estimateWorkspaceSize({ x: { min: 0, max: 1e9, step: 0.001 } }), /100,000/);
  assert.throws(() => estimateWorkspaceSize({ x: { min: 2, max: 1, step: 1 } }), /Sweep ranges/);
});

test('workspace yields to timers mid-sweep and reports real progress', async () => {
  let timerRan = false;
  let observed = false;
  let last;
  const result = await computeWorkspace(jointFixture(), { x: { min: -10, max: 10, step: 0.02 } }, {
    ballJointLimitDeg: 180,
    onProgress(progress) {
      last = progress;
      if (progress.completed === 256) setTimeout(() => { timerRan = true; }, 0);
      if (progress.completed === 512) observed = timerRan;
    }
  });
  assert.equal(observed, true);
  assert.equal(last.completed, result.total);
  assert.ok(result.samples.reachable.length <= 200);
});
