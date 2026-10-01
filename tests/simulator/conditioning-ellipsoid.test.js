import test from 'node:test';
import assert from 'node:assert/strict';
import { pairedFixture } from '../fixtures/layout.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { buildSceneGeometry, CONDITIONING_GRADE_FLOOR, conditioningColor, GROUND_DEPTH_BIAS_MM, OVERLAY_DEFAULTS,
  platformAxisLength, SCENE_BUILDERS, SCENE_COLORS } from '../../src/simulator/scene.js';
import { vectorMagnitude, vectorSub } from '../../src/math.js';

const close = (actual, expected, tolerance = 1e-9) =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
const ellipsoid = SCENE_BUILDERS.find(builder => builder.name === 'conditioningEllipsoid');
const RING_SEGMENTS = 32;
const mechanical = { ballJointLimitDeg: 180 };

function loaded(options = mechanical) {
  const controller = createSimulatorController();
  return { controller, state: controller.loadLayout(pairedFixture(), { options }) };
}

// The three principal-axis lines come first, each a full diameter through the centre.
const semiAxes = part => part.lines.slice(0, 3).map(line => vectorMagnitude(vectorSub(line.to, line.from)) / 2);

// A Jacobian whose translation block is diag(values) and whose rotation block is the identity.
const diagonalRows = values => Array.from({ length: 6 }, (_, row) =>
  Array.from({ length: 6 }, (_, col) => row === col ? (row < 3 ? values[row] : 1) : 0));

test('the conditioning ellipsoid is off by default and drawn only when switched on', () => {
  assert.equal(OVERLAY_DEFAULTS.conditioningEllipsoid, false);
  const { state } = loaded();
  const part = ellipsoid.build(state, state.layout, state.acceptedAssessment);
  assert.equal(part.lines.length, 3 + 3 * RING_SEGMENTS);
  const json = value => new Set(value.lines.map(line => JSON.stringify(line)));
  const off = json(buildSceneGeometry(state));
  assert.ok(part.lines.every(line => !off.has(JSON.stringify(line))));
  const on = json(buildSceneGeometry({ ...state, overlays: { ...state.overlays, conditioningEllipsoid: true } }));
  assert.ok(part.lines.every(line => on.has(JSON.stringify(line))));
  // No accepted pose, nothing to draw.
  assert.deepEqual(ellipsoid.build(state, state.layout, null), { lines: [], points: [] });
});

test('a symmetric layout at home gives a centred ellipsoid with equal horizontal axes', () => {
  const { state } = loaded();
  const { layout, acceptedAssessment: solved } = state;
  const part = ellipsoid.build(state, layout, solved);
  const axes = part.lines.slice(0, 3);
  for (const line of axes) {
    // Each axis is centred on the platform origin and sits behind the platform axes.
    line.from.forEach((value, k) => close((value + line.to[k]) / 2, solved.translation[k]));
    assert.equal(line.depthBias, -GROUND_DEPTH_BIAS_MM);
  }
  const [vertical, ...horizontal] = semiAxes(part);
  close(vertical, platformAxisLength(layout));
  close(Math.abs(axes[0].to[2] - axes[0].from[2]) / 2, vertical);
  close(horizontal[0], horizontal[1]);
  assert.ok(horizontal[0] < vertical);
  // Every ring point lies on the ellipsoid of those semi-axes.
  const directions = axes.map(line => vectorSub(line.to, line.from).map(value => value / 2));
  for (const line of part.lines.slice(3)) {
    const offset = vectorSub(line.from, solved.translation);
    const radius = directions.reduce((sum, axis, k) => {
      const length = semiAxes(part)[k];
      return sum + (offset.reduce((dot, value, i) => dot + value * axis[i], 0) / length ** 2) ** 2;
    }, 0);
    close(radius, 1, 1e-9);
  }
});

test('near a singularity the smallest axis shrinks in proportion to the smallest singular value', () => {
  const { state } = loaded();
  const { layout, acceptedAssessment } = state;
  const length = platformAxisLength(layout);
  for (const smallest of [0.5, 0.05, 1e-4]) {
    const solved = { ...acceptedAssessment, conditioning: { ...acceptedAssessment.conditioning,
      jacobianRows: diagonalRows([2, 1, smallest]) } };
    const axes = semiAxes(ellipsoid.build(state, layout, solved));
    close(axes[0], length);
    close(axes[1], length / 2);
    close(axes[2], length * smallest / 2);
  }
});

test('the ellipsoid colour grades by reciprocal condition and turns failure-coloured on a conditioning rejection', () => {
  const { state } = loaded();
  const solved = reciprocal => ({ conditioning: { reciprocal } });
  assert.deepEqual(conditioningColor(state, solved(1)), SCENE_COLORS.wellConditioned);
  conditioningColor(state, solved(CONDITIONING_GRADE_FLOOR)).forEach((value, k) => close(value, SCENE_COLORS.nearLimit[k]));
  for (const unavailable of [0, null, CONDITIONING_GRADE_FLOOR / 10]) {
    conditioningColor(state, solved(unavailable)).forEach((value, k) => close(value, SCENE_COLORS.nearLimit[k]));
  }
  // Halfway on the log scale is halfway between the two colours.
  conditioningColor(state, solved(Math.sqrt(CONDITIONING_GRADE_FLOOR))).forEach((value, k) =>
    close(value, (SCENE_COLORS.wellConditioned[k] + SCENE_COLORS.nearLimit[k]) / 2));
  // With a condition limit, the colour reaches nearLimit at that limit.
  const limited = { ...state, options: { ...state.options, conditionLimit: 4 } };
  conditioningColor(limited, solved(0.25)).forEach((value, k) => close(value, SCENE_COLORS.nearLimit[k]));
  conditioningColor(limited, solved(0.5)).forEach((value, k) =>
    close(value, (SCENE_COLORS.wellConditioned[k] + SCENE_COLORS.nearLimit[k]) / 2));

  // The paired layout's home condition is about 2.8 and 40 mm up it is about 5.4,
  // so a limit of 4 accepts home and rejects the raised request.
  const { controller } = loaded({ ...mechanical, conditionLimit: 4 });
  const rejected = controller.requestPose({ z: 40 });
  assert.equal(rejected.rejected, true);
  assert.equal(rejected.assessment.violations[0].type, 'conditionLimit');
  const part = ellipsoid.build(rejected, rejected.layout, rejected.acceptedAssessment);
  assert.ok(part.lines.length > 0, 'still drawn at the held pose');
  assert.ok(part.lines.every(line => line.color === SCENE_COLORS.globalFailure));
  const singular = { ...state, assessment: { ...state.assessment, violations: [{ type: 'numericalSingularity' }] } };
  assert.equal(conditioningColor(singular, state.acceptedAssessment), SCENE_COLORS.globalFailure);
  const legFailure = { ...state, assessment: { ...state.assessment, violations: [{ type: 'servoLimit', leg: 2 }] } };
  assert.notEqual(conditioningColor(legFailure, state.acceptedAssessment), SCENE_COLORS.globalFailure);
});
