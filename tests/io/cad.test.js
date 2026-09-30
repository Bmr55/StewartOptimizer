import test from 'node:test';
import assert from 'node:assert/strict';
import { evaluatePose } from '../../src/model/pose.js';
import { jointFixture } from '../fixtures/layout.js';
import { buildConstructionSkeleton, canExportCad, skeletonToCSV, skeletonToFusionScript } from '../../src/io/cad.js';

const distance = (a, b) => Math.hypot(...a.map((value, i) => value - b[i]));

function evaluatedFixture() {
  const layout = jointFixture();
  layout.id = 17;
  layout.platformAnchors[0][0] += 5;
  layout.baseAnchors[2][1] -= 3;
  const homePose = evaluatePose(layout, {}, { ballJointLimitDeg: 180, recordLegData: true });
  assert.equal(homePose.reachable, true);
  return { layout, homePose, feasibility: { passing: false, homePoseSatisfied: true,
    sampledWorkspaceSatisfied: false, cycleSatisfied: true, failedCategories: ['workspace'] } };
}

test('CAD skeleton uses solved asymmetric home geometry and retains diagnostic identity', () => {
  const evaluation = evaluatedFixture();
  const skeleton = buildConstructionSkeleton(evaluation, { id: 'run-4', effective_settings: { seed: 1 } });
  assert.equal(skeleton.candidateId, 17);
  assert.equal(skeleton.diagnostic, true);
  assert.deepEqual(skeleton.failedCategories, ['workspace']);
  assert.equal(skeleton.legs.length, 6);
  const points = Object.fromEntries(skeleton.points.map(point => [point.name, point.xyz]));
  assert.deepEqual(points['Base anchor 1'], evaluation.layout.baseAnchors[0]);
  assert.deepEqual(points['Platform anchor 1'], evaluation.homePose.platformPoints[0]);
  assert.deepEqual(points['Horn tip 1'], evaluation.homePose.hornTips[0]);
  for (let leg = 1; leg <= 6; leg++) {
    assert.ok(Math.abs(distance(points[`Base anchor ${leg}`], points[`Horn tip ${leg}`]) - evaluation.layout.hornLength) < 1e-6);
    assert.ok(Math.abs(distance(points[`Horn tip ${leg}`], points[`Platform anchor ${leg}`]) - evaluation.layout.rodLength) < 1e-6);
  }
  assert.deepEqual(points['Platform centroid'], [0, 1, 2].map(axis =>
    evaluation.homePose.platformPoints.reduce((sum, point) => sum + point[axis], 0) / 6));
  const csv = skeletonToCSV(skeleton);
  assert.match(csv, /^candidate_id,name,kind,frame,x_mm,y_mm,z_mm\r\n/);
  assert.match(csv, /"17","Horn tip 1","horn_tip","world"/);
  const script = skeletonToFusionScript(skeleton);
  assert.match(script, /value \/ 10\.0/);
  assert.match(script, /DirectDesignType/);
  assert.match(script, /setByPoint/);
  assert.match(script, /setByThreePoints/);
  assert.match(script, /modelToSketchSpace/);
  assert.match(script, /line\.isConstruction = True/);
  assert.doesNotMatch(script, /extrudeFeatures|bRepBodies|NewBodyFeatureOperation/);
});

test('invalid home geometry blocks both CAD exports while leaving JSON independent', () => {
  const evaluation = evaluatedFixture();
  evaluation.homePose = { ...evaluation.homePose, reachable: false, hornTips: [] };
  assert.equal(canExportCad(evaluation), false);
  assert.throws(() => buildConstructionSkeleton(evaluation), /six valid solved legs/);
});
