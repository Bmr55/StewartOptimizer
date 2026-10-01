import test from 'node:test';
import assert from 'node:assert/strict';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { buildSceneGeometry, LOAD_DEPTH_BIAS_MM, OVERLAY_DEFAULTS, rodForceColor, SCENE_BUILDERS, SCENE_COLORS,
  TORQUE_BAND_RADII, torqueUtilizationColor } from '../../src/simulator/scene.js';
import { hornFrameAxes } from '../../src/model/kinematics.js';
import { effectiveServoRange } from '../../src/model/pose.js';
import { vectorAdd, vectorScale } from '../../src/math.js';

const loadsBuilder = SCENE_BUILDERS.find(builder => builder.name === 'loads');
const settings = { ballJointLimitDeg: 180 };
const closeVector = (actual, expected, tolerance = 1e-9) => actual.forEach((value, k) =>
  assert.ok(Math.abs(value - expected[k]) <= tolerance, `${actual} != ${expected}`));

function loaded(loadModel) {
  const controller = createSimulatorController();
  const state = controller.loadLayout(asymmetricJointFixture(), { options: settings, loadModel });
  return { controller, state: { ...state, overlays: { ...state.overlays, loads: true } } };
}
const build = state => loadsBuilder.build(state, state.layout, state.acceptedAssessment);

test('torque utilisation above 1 selects the failure colour; below it grades to the warning colour', () => {
  assert.deepEqual(torqueUtilizationColor(1.0001), SCENE_COLORS.failure);
  assert.deepEqual(torqueUtilizationColor(7), SCENE_COLORS.failure);
  closeVector(torqueUtilizationColor(1), SCENE_COLORS.nearLimit);
  closeVector(torqueUtilizationColor(0), SCENE_COLORS.torqueLow);
  closeVector(torqueUtilizationColor(0.5), SCENE_COLORS.torqueLow.map((value, k) => (value + SCENE_COLORS.nearLimit[k]) / 2));
});

test('rod colour grades from the plain rod toward compression blue or tension red, full at the reference', () => {
  closeVector(rodForceColor(0, 10), SCENE_COLORS.rod);
  closeVector(rodForceColor(10, 10), SCENE_COLORS.compression);
  closeVector(rodForceColor(25, 10), SCENE_COLORS.compression);
  closeVector(rodForceColor(-10, 10), SCENE_COLORS.tension);
  closeVector(rodForceColor(-5, 10), SCENE_COLORS.rod.map((value, k) => (value + SCENE_COLORS.tension[k]) / 2));
  closeVector(rodForceColor(5, 0), SCENE_COLORS.rod);
});

test('the loads overlay is off by default and draws graded rods plus a torque gauge per rated servo', () => {
  assert.equal(OVERLAY_DEFAULTS.loads, false);
  const { state } = loaded({ mass_kg: 2.5, servo_torque_rating_nm: 0.4,
    per_servo_ratings: [{ torque_nm: 1e-4 }, null, null, null, null, null] });
  const { layout, acceptedAssessment: solved, loads } = state;
  assert.equal(loads.valid, true);
  const { overlays, ...defaults } = state;
  assert.equal(buildSceneGeometry(defaults).lines.some(line => line.depthBias === LOAD_DEPTH_BIAS_MM), false);
  const { lines } = build(state);
  // Six graded rods first, drawn over the plain rods.
  const rods = lines.slice(0, 6);
  rods.forEach((line, i) => {
    assert.deepEqual(line.from, solved.hornTips[i]);
    assert.deepEqual(line.to, solved.platformPoints[i]);
    assert.equal(line.depthBias, LOAD_DEPTH_BIAS_MM);
    assert.deepEqual(line.color, rodForceColor(loads.rodForceN[i], loads.staticForceN));
  });
  // Gauges: a band of concentric arcs from mid-travel toward the stop in the
  // torque's direction, reaching it at the rating and capped there above it.
  const [min, max] = effectiveServoRange(layout, settings);
  const middle = (min + max) / 2;
  const gauges = lines.slice(6);
  const at = (i, alpha, scale) => vectorAdd(layout.baseAnchors[i],
    vectorScale(hornFrameAxes(layout.betaAngles[i], alpha)[0], layout.hornLength * scale));
  let offset = 0;
  for (let i = 0; i < 6; i++) {
    const utilization = loads.utilization[i];
    const sweep = Math.sign(loads.servoTorqueNm[i]) * Math.min(utilization, 1) * (max - middle);
    const steps = Math.max(1, Math.ceil(24 * Math.abs(sweep) / (max - min)));
    const band = gauges.slice(offset, offset + steps * TORQUE_BAND_RADII.length);
    offset += band.length;
    assert.ok(band.every(line => line.color === band[0].color && line.depthBias === undefined));
    assert.deepEqual(band[0].color, torqueUtilizationColor(utilization));
    closeVector(band[0].from, at(i, middle, TORQUE_BAND_RADII[0]));
    closeVector(band.at(-1).to, at(i, middle + sweep, TORQUE_BAND_RADII.at(-1)));
  }
  assert.equal(offset, gauges.length);
  // Servo 1's tiny rating saturates it: failure colour and a gauge to the stop.
  assert.ok(loads.utilization[0] > 1);
  assert.deepEqual(gauges[0].color, SCENE_COLORS.failure);
});

test('unrated servos get no gauge, failure-coloured legs keep their colour, and no loads draw nothing', () => {
  const { controller, state } = loaded({ mass_kg: 2.5 });
  assert.equal(build(state).lines.length, 6);
  // A rejected request whose failing legs the ghost cannot draw colours the held legs instead.
  const rejected = { ...state, ...controller.requestPose({ z: 500 }), overlays: { ...state.overlays, requestedGhost: false } };
  const violations = rejected.assessment.violations;
  const failedLegs = new Set(violations.filter(violation => Number.isInteger(violation.leg)).map(violation => violation.leg));
  assert.ok(failedLegs.size > 0 && failedLegs.size < 6);
  const expected = violations.some(violation => !Number.isInteger(violation.leg)) ? []
    : [0, 1, 2, 3, 4, 5].filter(leg => !failedLegs.has(leg));
  const tips = rejected.acceptedAssessment.hornTips;
  assert.deepEqual(build(rejected).lines.map(line => tips.indexOf(line.from)), expected);
  for (const loads of [null, { valid: false, motion: 'static', reason: 'Cycle force equilibrium is singular.' }]) {
    assert.deepEqual(build({ ...state, loads }), { lines: [], points: [] });
  }
  assert.deepEqual(build({ ...state, acceptedAssessment: null }), { lines: [], points: [] });
});
