import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { animationPose, animationState, createSimulatorController, HOME_POSE } from '../../src/simulator/controller.js';
import { hasLoad, loadModelFromSettings, parseLoadModel } from '../../src/simulator/loads.js';
import { computeCycleDemand } from '../../src/model/cycle.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { rotationMatrixFromEuler, vectorNormalize } from '../../src/math.js';

const close = (actual, expected, tolerance = 1e-9, label = '') =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${label} ${actual} != ${expected}`);
const settings = { ballJointLimitDeg: 180 };
const sample = parseRequirements(readFileSync(new URL('../../examples/sample-requirements.json', import.meta.url), 'utf8')).normalized;
const sampleLoad = loadModelFromSettings({ requirements: sample });

function loaded(loadModel = sampleLoad, options = settings) {
  const controller = createSimulatorController();
  return { controller, state: controller.loadLayout(asymmetricJointFixture(), { options, loadModel }) };
}

// Net force the six rods apply to the platform: each pushes along its rod direction.
const netRodForce = state => state.acceptedAssessment.rodVectors.map(vectorNormalize)
  .reduce((sum, direction, i) => sum.map((value, k) => value + direction[k] * state.loads.rodForceN[i]), [0, 0, 0]);

test('a static pose with the sample mass holds the payload weight and matches the cycle demand', () => {
  assert.equal(sample.mass_kg, 2.5);
  assert.deepEqual(sampleLoad, { mass_kg: 2.5, servo_rating_policy: 'enforced' });
  const { controller, state } = loaded();
  assert.equal(state.loads.valid, true);
  assert.equal(state.loads.motion, 'static');
  const weight = sample.mass_kg * 9.81;
  close(state.loads.staticForceN, weight, 1e-12);
  const net = netRodForce(state);
  close(net[0], 0, 1e-9, 'x');
  close(net[1], 0, 1e-9, 'y');
  close(net[2], weight, 1e-9, 'z');
  // A stationary cycle evaluates the home pose once with the same dynamics.
  const cycle = computeCycleDemand(state.layout, { mass: sample.mass_kg, stroke: 0, frequency: 0, ...settings });
  assert.equal(cycle.valid, true);
  assert.equal(cycle.sampling.status, 'stationary');
  state.loads.servoTorqueNm.forEach((torque, i) => close(Math.abs(torque), cycle.perServoTorqueNm[i], 1e-12, `servo ${i + 1}`));
  // Unrated servos have torque but no utilisation.
  assert.deepEqual(state.loads.ratedTorqueNm, new Array(6).fill(null));
  assert.deepEqual(state.loads.utilization, new Array(6).fill(null));
  // Away from home the rods still carry exactly the weight.
  const moved = controller.requestPose({ x: 8, z: 6, rx: 0.05, rz: -0.04 });
  assert.equal(moved.rejected, false);
  const offset = netRodForce(moved);
  close(offset[2], weight, 1e-9, 'moved z');
  close(Math.hypot(offset[0], offset[1]), 0, 1e-9, 'moved xy');
});

test('utilisation divides output-shaft torque by the peak rating and includes the actuator model', () => {
  const rated = loaded({ mass_kg: 2.5, servo_torque_rating_nm: 0.5,
    per_servo_ratings: [{ torque_nm: 0.01 }, null, null, null, null, null] }).state.loads;
  rated.utilization.forEach((value, i) =>
    close(value, Math.abs(rated.servoTorqueNm[i]) / (i === 0 ? 0.01 : 0.5), 1e-15));
  assert.deepEqual(rated.ratedTorqueNm, [0.01, 0.5, 0.5, 0.5, 0.5, 0.5]);
  assert.ok(rated.utilization[0] > 1);
  // Static: Coulomb friction is zero at rest, so the actuator adds nothing.
  const withActuator = loaded({ mass_kg: 2.5, servo_actuator: { coulomb_nm: 0.2, viscous_nm_s_per_rad: 0.1 } }).state.loads;
  withActuator.servoTorqueNm.forEach((torque, i) => close(torque, rated.servoTorqueNm[i], 1e-15));
});

test('animation frames solve dynamic loads from analytic derivatives scaled by the playback speed', () => {
  const { controller } = loaded();
  controller.setAnimation('helical', true, { speed: 1 });
  const state = controller.tick(0.05);
  assert.equal(state.requestSource, 'animation');
  assert.equal(state.loads.motion, 'animation');
  // Moving, so the net rod force is no longer just the weight.
  assert.ok(Math.abs(netRodForce(state)[1]) > 1e-6);
  // A manual request afterwards is static again.
  assert.equal(controller.requestPose(state.accepted).loads.motion, 'static');

  for (const pattern of ['wobble', 'pingpong', 'rotate', 'tilt', 'helical']) {
    const t = 0.7, h = 1e-5, rate = 2;
    const motion = animationState(pattern, t, {}, rate);
    assert.deepEqual(motion.pose, animationPose(pattern, t));
    const at = dt => animationPose(pattern, t + dt);
    const [before, after] = [at(-h), at(h)];
    ['x', 'y', 'z'].forEach((axis, k) => {
      const value = animationPose(pattern, t)[axis];
      close(motion.velocity[k], rate * (after[axis] - before[axis]) / (2 * h) / 1000, 1e-7, `${pattern} v${axis}`);
      close(motion.acceleration[k], rate * rate * (after[axis] - 2 * value + before[axis]) / (h * h) / 1000, 1e-3,
        `${pattern} a${axis}`);
    });
    // omega^ = Rdot R^T, from the evaluator's rotation convention.
    const rotation = pose => rotationMatrixFromEuler(pose.rx, pose.ry, pose.rz);
    const [r0, r1, r] = [rotation(before), rotation(after), rotation(animationPose(pattern, t))];
    const w = r.map((row, i) => row.map((_, j) => [0, 1, 2].reduce((sum, k) =>
      sum + rate * (r1[i][k] - r0[i][k]) / (2 * h) * r[j][k], 0)));
    [w[2][1], w[0][2], w[1][0]].forEach((value, k) => close(motion.omega[k], value, 1e-7, `${pattern} omega${k}`));
  }
  const still = animationState('none', 3);
  assert.deepEqual(still.pose, HOME_POSE);
  assert.deepEqual([...still.velocity, ...still.acceleration, ...still.omega, ...still.angularAcceleration], new Array(12).fill(0));
});

test('loads follow the accepted pose, the load model and the layout lifecycle', () => {
  const { controller, state } = loaded();
  const home = state.loads;
  // A rejected request keeps the accepted pose and its loads.
  const rejected = controller.requestPose({ z: 500 });
  assert.equal(rejected.rejected, true);
  assert.deepEqual(rejected.loads, home);
  // A reload without a load model (a geometry edit) keeps it; null clears it.
  const kept = controller.loadLayout(asymmetricJointFixture(), { options: settings });
  assert.deepEqual(kept.loadModel, sampleLoad);
  assert.deepEqual(kept.loads, home);
  const cleared = controller.loadLayout(asymmetricJointFixture(), { options: settings, loadModel: null });
  assert.equal(cleared.loadModel, null);
  assert.equal(cleared.loads, null);
  // An invalid model throws before any state changes.
  controller.loadLayout(asymmetricJointFixture(), { options: settings, loadModel: sampleLoad });
  const before = controller.getState();
  assert.throws(() => controller.loadLayout(asymmetricJointFixture(), { options: settings, loadModel: { mass_kg: -1 } }),
    /loadModel: mass_kg must be a finite number >= 0/);
  assert.deepEqual(controller.getState(), before);
  // No accepted pose means no loads; clear() drops the model.
  assert.equal(loaded(sampleLoad, { ...settings, conditionLimit: 1 }).state.loads, null);
  const empty = controller.clear();
  assert.equal(empty.loadModel, null);
  assert.equal(empty.loads, null);
});

test('load models keep requirement keys only and come from a run\'s effective settings', () => {
  assert.equal(parseLoadModel(null), null);
  assert.equal(parseLoadModel({ cycle_mm: 3 }), null);
  assert.deepEqual(parseLoadModel({ mass_kg: 2, center_of_mass_mm: [0, 0, 10], extra: 1 }).input,
    { mass_kg: 2, center_of_mass_mm: [0, 0, 10] });
  assert.throws(() => parseLoadModel([1]), /loadModel must be an object/);
  assert.throws(() => parseLoadModel({ servo_torque_rating_nm: -1 }, 'simulator.loadModel'),
    /simulator.loadModel: servo_torque_rating_nm/);
  // The run's rating input (which carries UI overrides) wins over the requirements.
  assert.deepEqual(loadModelFromSettings({ requirements: { mass_kg: 3, servo_torque_rating_nm: 1, cycle_mm: 5 },
    servoRatings: { servo_torque_rating_nm: 2 } }), { mass_kg: 3, servo_torque_rating_nm: 2 });
  assert.deepEqual(loadModelFromSettings({ requirements: { mass_kg: 3, servo_torque_rating_nm: 1 } }),
    { mass_kg: 3, servo_torque_rating_nm: 1 });
  assert.equal(loadModelFromSettings({ requirements: { cycle_mm: 5 } }), null);
  assert.equal(loadModelFromSettings(null), null);
  assert.equal(hasLoad(null), false);
  assert.equal(hasLoad({ mass_kg: 0, servo_torque_rating_nm: 1 }), false);
  assert.equal(hasLoad({ mass_kg: 0.1 }), true);
  assert.equal(hasLoad({ external_force_n: [0, 0, 0], external_moment_nm: [0, 0.2, 0] }), true);
});
