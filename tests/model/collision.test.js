import test from 'node:test';
import assert from 'node:assert/strict';
import { crossedPairedFixture, pairedFixture } from '../fixtures/layout.js';
import { assessLinkClearance, segmentDistance, violationLegs } from '../../src/model/collision.js';
import { evaluatePose } from '../../src/model/pose.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { evaluateLayout } from '../../src/optimization/evaluate-layout.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { layoutToJSON } from '../../src/io/results.js';
import { DEFAULT_LINK_CLEARANCE_MM, FAILURE_CATEGORIES } from '../../src/contracts.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { buildPoseDiagnostics } from '../../src/simulator/diagnostics.js';
import { buildSceneGeometry, SCENE_COLORS } from '../../src/simulator/scene.js';
import { sampleText } from '../ui/helpers.js';

const close = (actual, expected, message = '') =>
  assert.ok(Math.abs(actual - expected) <= 1e-12, `${message}: ${actual} vs ${expected}`);
const mechanical = { ballJointLimitDeg: 180 };

test('segment distance covers crossing, skew, parallel, endpoint and degenerate cases', () => {
  close(segmentDistance([-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0]), 0, 'crossing');
  close(segmentDistance([-1, 0, 0], [1, 0, 0], [0, -1, 3], [0, 1, 3]), 3, 'skew, closest inside both');
  close(segmentDistance([0, 0, 0], [4, 0, 0], [1, 2, 0], [3, 2, 0]), 2, 'parallel overlap');
  close(segmentDistance([0, 0, 0], [1, 0, 0], [4, 4, 0], [5, 4, 0]), 5, 'parallel, endpoint to endpoint');
  close(segmentDistance([0, 0, 0], [1, 0, 0], [3, -1, 0], [3, 1, 0]), 2, 'closest point clamped to an end');
  close(segmentDistance([0, 0, 0], [0, 0, 0], [3, 4, 0], [3, 4, 0]), 5, 'two points');
  close(segmentDistance([0, 0, 0], [0, 0, 0], [-1, 2, 0], [1, 2, 0]), 2, 'point to segment');
  close(segmentDistance([-1, 2, 0], [1, 2, 0], [0, 0, 0], [0, 0, 0]), 2, 'segment to point');
});

test('only links of different legs are checked, and each pair is reported once', () => {
  // Leg 0's horn and rod meet at the horn tip but are never checked against each other.
  const base = Array.from({ length: 6 }, (_, i) => [100 * i, 0, 0]);
  const tips = base.map(([x]) => [x, 50, 0]);
  const top = base.map(([x]) => [x, 50, 200]);
  const clear = assessLinkClearance(base, tips, top, 6);
  assert.deepEqual(clear.violations, []);
  assert.equal(clear.closest.distanceMm, 100);
  tips[1] = [5, 50, 0];
  const touching = assessLinkClearance(base, tips, top, 6);
  assert.ok(touching.violations.length > 0);
  for (const violation of touching.violations) {
    assert.equal(violation.type, 'linkCollision');
    assert.deepEqual([violation.leg, violation.otherLeg], [0, 1]);
    assert.ok(violation.value < 6 && violation.limit === 6);
  }
  assert.deepEqual(violationLegs(touching.violations[0]), [0, 1]);
  assert.deepEqual(violationLegs({ type: 'ballJoint', leg: 4 }), [4]);
  assert.deepEqual(violationLegs({ type: 'numericalSingularity' }), []);
});

test('a pose whose rods cross is rejected even in soft ball-joint mode, and clearance 0 turns the check off', () => {
  assert.equal(DEFAULT_LINK_CLEARANCE_MM, 6);
  assert.ok(FAILURE_CATEGORIES.includes('collision'));
  const crossed = crossedPairedFixture();
  const result = evaluatePose(crossed, {}, { ...mechanical, ballJointClamp: true });
  assert.equal(result.geometricallyReachable, true, 'every leg still solves');
  assert.equal(result.mechanicallyReachable, false);
  assert.equal(result.reachable, false);
  assert.equal(result.relaxedReachable, false);
  const collisions = result.violations.filter(violation => violation.type === 'linkCollision');
  // The two rods of each of the three pairs cross on the pair's mirror plane.
  assert.deepEqual(collisions.filter(item => item.link === 'rod' && item.otherLink === 'rod')
    .map(item => [item.leg, item.otherLeg]), [[0, 1], [2, 3], [4, 5]]);
  assert.ok(result.clearance.closest.distanceMm < 1e-9);
  assert.equal(result.clearance.clearanceMm, DEFAULT_LINK_CLEARANCE_MM);
  const off = evaluatePose(crossed, {}, { ...mechanical, linkClearanceMm: 0 });
  assert.equal(off.reachable, true);
  const paired = evaluatePose(pairedFixture(), {}, mechanical);
  assert.equal(paired.reachable, true);
  assert.ok(paired.clearance.closest.distanceMm > 20);
  assert.equal(evaluatePose(pairedFixture(), {}, { ...mechanical, linkClearanceMm: 30 }).reachable, false);
  for (const bad of [-1, NaN, Infinity, '6', null]) {
    assert.throws(() => evaluatePose(pairedFixture(), {}, { ...mechanical, linkClearanceMm: bad }),
      /linkClearanceMm must be a finite nonnegative length/);
  }
});

test('a colliding layout fails the collision category at home, in the workspace and in the cycle', async () => {
  const options = { ranges: { z: { min: -5, max: 5, step: 5 } }, sampling: { strategy: 'grid' },
    payload: 1, stroke: 10, frequency: 1, cycleAxis: 'z', ...mechanical, servoRangeRad: [-Math.PI, Math.PI] };
  const crossed = await evaluateLayout(crossedPairedFixture(), options);
  assert.equal(crossed.feasibility.collisionSatisfied, false);
  assert.ok(crossed.feasibility.failedCategories.includes('collision'));
  assert.equal(crossed.homePose.reachable, false);
  assert.ok(crossed.workspace.stats.violationCounts.linkCollision > 0);
  assert.equal(crossed.workspace.constraintPolicy.linkClearanceMm, DEFAULT_LINK_CLEARANCE_MM);
  assert.ok(crossed.cycle.violations.some(violation => violation.type === 'linkCollision'));
  const paired = await evaluateLayout(pairedFixture(), options);
  assert.equal(paired.feasibility.collisionSatisfied, true);
  assert.equal(paired.feasibility.failedCategories.includes('collision'), false);
  const unchecked = await evaluateLayout(crossedPairedFixture(), { ...options, linkClearanceMm: 0 });
  assert.equal(unchecked.feasibility.collisionSatisfied, true);
});

test('link_clearance_mm defaults, validates, reaches the optimizer and replays older runs without the check', () => {
  const parsed = parseRequirements(sampleText).normalized;
  assert.equal(parsed.link_clearance_mm, DEFAULT_LINK_CLEARANCE_MM);
  const data = JSON.parse(sampleText);
  data.constraints.link_clearance_mm = 9;
  assert.equal(parseRequirements(JSON.stringify(data)).normalized.link_clearance_mm, 9);
  data.constraints.link_clearance_mm = -1;
  assert.throws(() => parseRequirements(JSON.stringify(data)), /link_clearance_mm/);
  assert.equal(new Optimizer({}).linkClearanceMm, DEFAULT_LINK_CLEARANCE_MM);
  assert.equal(new Optimizer({ link_clearance_mm: 9 }).linkClearanceMm, 9);
  assert.equal(new Optimizer({ link_clearance_mm: 9 }, { linkClearanceMm: 3 }).linkClearanceMm, 3, 'the option wins');
  assert.throws(() => new Optimizer({}, { linkClearanceMm: -2 }), /linkClearanceMm/);
  const optimizer = new Optimizer({}, { linkClearanceMm: 4 });
  const settings = optimizer.effectiveSettings();
  assert.equal(settings.linkClearanceMm, 4);
  assert.equal(optimizer.evaluationOptions().linkClearanceMm, 4);
  const exported = { ...layoutToJSON(optimizer.createRandomLayout()), run: { effective_settings: settings } };
  assert.equal(Optimizer.fromReplay(exported).linkClearanceMm, 4);
  const { linkClearanceMm, ...older } = settings;
  assert.equal(Optimizer.fromReplay({ ...exported, run: { effective_settings: older } }).linkClearanceMm, 0);
});

test('the simulator rejects a colliding request, lists it on both legs and colours both legs', () => {
  const controller = createSimulatorController();
  controller.loadLayout(crossedPairedFixture(), { options: { ...mechanical, linkClearanceMm: 0 } });
  const state = controller.setOptions({ linkClearanceMm: DEFAULT_LINK_CLEARANCE_MM });
  assert.equal(state.rejected, true);
  const diagnostics = buildPoseDiagnostics(state);
  assert.equal(diagnostics.linkClearanceMm, DEFAULT_LINK_CLEARANCE_MM);
  assert.ok(diagnostics.closestLinks.distanceMm < 1e-9);
  assert.deepEqual(diagnostics.affectedLegs, [1, 2, 3, 4, 5, 6]);
  const first = diagnostics.legs[0].failures.find(failure => failure.collision?.link === 'rod'
    && failure.collision.otherLink === 'rod');
  const second = diagnostics.legs[1].failures.find(failure => failure.collision?.link === 'rod'
    && failure.collision.otherLink === 'rod');
  assert.equal(first.collision.otherLeg, 2);
  assert.equal(second.collision.otherLeg, 1);

  // A collision between two legs colours both, and only those, on the held pose.
  const paired = createSimulatorController().loadLayout(pairedFixture(), { options: mechanical });
  const failing = { ...paired, overlays: { requestedGhost: false },
    assessment: { ...paired.assessment, violations: [{ type: 'linkCollision', leg: 0, otherLeg: 3,
      link: 'horn', otherLink: 'rod', value: 2, limit: 6 }] } };
  const { lines } = buildSceneGeometry(failing);
  const hornColor = leg => lines.find(line => line.from === paired.layout.baseAnchors[leg]
    && line.to === paired.acceptedAssessment.hornTips[leg]).color;
  for (let leg = 0; leg < 6; leg++) {
    assert.equal(hornColor(leg) === SCENE_COLORS.failure, leg === 0 || leg === 3, `leg ${leg}`);
  }
});
