import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { computeWorkspace } from '../src/workspace/sweep.js';
import { evaluatePose } from '../src/model/pose.js';
import { jointFixture } from './fixture.js';

// Captured before extraction at 97b7360; includes capped reservoir samples.
const reference = JSON.parse(fs.readFileSync(new URL('./fixtures/workspace-reference.json', import.meta.url)));
test('pose and workspace results preserve the pre-refactor numerical contract', async t => {
  t.mock.method(Math, 'random', () => 0.25);
  for (const { settings, expected, pose, expectedPose } of reference.cases) {
    assert.deepEqual(await computeWorkspace(jointFixture(), reference.ranges, settings), expected);
    assert.deepEqual(evaluatePose(jointFixture(), pose, { ...settings, recordLegData: true }), expectedPose);
  }
});
