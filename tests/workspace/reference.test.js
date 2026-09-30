import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { computeWorkspace } from '../../src/workspace/sweep.js';
import { evaluatePose } from '../../src/model/pose.js';
import { jointFixture } from '../fixtures/layout.js';

// Historical single-angle fixture is retained as comparative evidence for IK
// and actual-rod geometry. Its joint coverage is intentionally superseded.
const reference = JSON.parse(fs.readFileSync(new URL('../fixtures/workspace-reference.json', import.meta.url)));
function close(actual, expected, tolerance = 1e-9) {
  if (Array.isArray(expected)) {
    assert.equal(actual.length, expected.length);
    actual.forEach((value, i) => close(value, expected[i], tolerance));
  } else {
    assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
  }
}

test('home-aligned sockets and singularity rejection change coverage while preserving IK and actual rods', async () => {
  const oldFeasible = reference.cases.find(c => c.settings.ballJointLimitDeg === 100);
  const current = evaluatePose(jointFixture(), oldFeasible.pose,
    { ...oldFeasible.settings, recordLegData: true });
  assert.equal(current.reachable, true);
  for (const key of ['servoAngles', 'rodLengths', 'hornTips', 'rodVectors', 'platformPoints']) {
    close(current[key], oldFeasible.expectedPose[key]);
  }

  const workspace = await computeWorkspace(jointFixture(), reference.ranges,
    { ...reference.cases[0].settings, sampling: { strategy: 'grid' } });
  assert.deepEqual(workspace.sampling, { strategy: 'grid' });
  assert.equal(workspace.total, 9);
  assert.equal(workspace.coverage, 800 / 9);
  assert.equal(workspace.relaxedCoverage, 800 / 9);
  assert.equal(workspace.stats.violationCounts.numericalSingularity, 1);
  assert.ok(workspace.stats.ballJointOverallMax < 52 * Math.PI / 180);
  assert.equal(reference.cases[0].expected.coverage, 0);
});
