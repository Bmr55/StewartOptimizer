import test from 'node:test';
import assert from 'node:assert/strict';
import { evaluatePose, computeWorkspace } from '../workspace.js';
import { Optimizer } from '../optimizer.js';
import { jointFixture } from './fixture.js';

test('soft joint violations never become mechanically reachable poses', async () => {
  const layout = jointFixture();
  const options = { ballJointLimitDeg: 52, ballJointClamp: true };
  const pose = evaluatePose(layout, {}, options);
  assert.equal(pose.reachable, false);
  assert.equal(pose.relaxedReachable, true);
  assert.equal(pose.violations.length, 6);
  const workspace = await computeWorkspace(layout, {}, options);
  assert.equal(workspace.coverage, 0);
  assert.equal(workspace.relaxedCoverage, 100);
  assert.equal(workspace.counts.reachable, 0);
  assert.equal(workspace.counts.unreachable, 1);
  assert.equal(workspace.stats.violationRate, 1);
  const strict = await computeWorkspace(layout, {}, { ballJointLimitDeg: 52 });
  assert.equal(strict.coverage, 0);
  const valid = await computeWorkspace(layout, {}, { ballJointLimitDeg: 100 });
  assert.equal(valid.coverage, 100);
});

test('export discloses relaxed coverage, violations, and feasibility scope', async () => {
  const optimizer = new Optimizer({}, { ballJointLimitDeg: 52, ballJointClamp: true });
  const evaluation = await optimizer.evaluateLayout(jointFixture());
  optimizer.fitness = [evaluation];
  let output;
  optimizer.download = data => { output = JSON.parse(data); };
  optimizer.exportBest();
  assert.equal(output.metadata.coverage, 0);
  assert.equal(output.metadata.relaxed_coverage, 100);
  assert.equal(output.feasibility.sampledWorkspaceSatisfied, false);
  assert.equal(output.constraint_policy.mode, 'soft-ball-joint');
  assert.equal(output.workspace_counts.violationPoses, 1);
});
