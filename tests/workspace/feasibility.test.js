import test from 'node:test';
import assert from 'node:assert/strict';
import { evaluatePose, computeWorkspace } from '../../workspace.js';
import { Optimizer } from '../../optimizer.js';
import { resolveMounting } from '../../src/model/mounting.js';
import { jointFixture } from '../fixtures/layout.js';

function lowerFailureLayout() {
  const layout = jointFixture();
  const mounting = resolveMounting(layout).mounting;
  mounting.lower[0] = {
    direction: mounting.lower[0].direction.map(component => -component), source: 'supplied',
  };
  layout.mounting = mounting;
  return layout;
}

test('soft joint violations never become mechanically reachable poses', async () => {
  const layout = lowerFailureLayout();
  const options = { ballJointLimitDeg: 52, ballJointClamp: true };
  const pose = evaluatePose(layout, {}, options);
  assert.equal(pose.reachable, false);
  assert.equal(pose.relaxedReachable, true);
  assert.deepEqual(pose.violations.map(v => [v.type, v.leg, v.joint]), [['ballJoint', 0, 'lower']]);
  const workspace = await computeWorkspace(layout, {}, options);
  assert.equal(workspace.coverage, 0);
  assert.equal(workspace.relaxedCoverage, 100);
  assert.equal(workspace.counts.reachable, 0);
  assert.equal(workspace.counts.unreachable, 1);
  assert.equal(workspace.stats.violationRate, 1);
  assert.deepEqual(workspace.stats.jointViolationCounts, { lower: 1, upper: 0 });
  const strict = await computeWorkspace(layout, {}, { ballJointLimitDeg: 52 });
  assert.equal(strict.coverage, 0);
  const valid = await computeWorkspace(layout, {}, { ballJointLimitDeg: 180 });
  assert.equal(valid.coverage, 100);
});

test('export discloses relaxed coverage, violations, and feasibility scope', async () => {
  const optimizer = new Optimizer({}, { ballJointLimitDeg: 52, ballJointClamp: true });
  const evaluation = await optimizer.evaluateLayout(lowerFailureLayout());
  optimizer.fitness = [evaluation];
  let output;
  optimizer.download = data => { output = JSON.parse(data); };
  optimizer.exportBest();
  assert.equal(output.metadata.coverage, 0);
  assert.equal(output.metadata.relaxed_coverage, 100);
  assert.equal(output.feasibility.sampledWorkspaceSatisfied, false);
  assert.equal(output.feasibility.homePoseSatisfied, false);
  assert.equal(output.feasibility.cycleSatisfied, false);
  assert.equal(output.constraint_policy.mode, 'soft-ball-joint');
  assert.equal(output.workspace_counts.violationPoses, 1024);
  assert.equal(output.workspace_stats.jointViolationCounts.lower, 1024);
  assert.equal(output.schema_version, 2);
  assert.equal(output.model_version, 2);
  assert.equal(output.mounting.lower[0].source, 'supplied');
  assert.equal(output.mounting.upper[0].source, 'derived');
  assert.equal(output.constraint_policy.lowerBallJointLimitDeg, 52);
  assert.equal(output.constraint_policy.upperBallJointLimitDeg, 52);
});
