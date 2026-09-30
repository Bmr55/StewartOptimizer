import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../optimizer.js';
import { loadUI, sampleText } from './helpers.js';

test('run keeps explicit overrides and refreshes untouched defaults from JSON', async () => {
  let captured;
  class StubOptimizer {
    constructor(requirements, options) { captured = { requirements, options }; this.pareto = []; this.fitness = []; }
    estimateWork() { return { totalPoses: 8 }; }
    start(done) { done?.(); return Promise.resolve({ status: 'completed' }); }
  }
  const element = await loadUI(StubOptimizer);
  element('optXStep').value = '40';
  element('optXMin').value = '-5';
  element('ballJointLimit').value = '10';
  const input = JSON.parse(sampleText);
  input.workspace.y_range_mm = [-10, 20];
  element('requirementsInput').value = JSON.stringify(input);
  await element('runOptimization').handlers.click();
  assert.equal(captured.options.ranges.x.step, 40);
  assert.equal(captured.options.ranges.x.min, -5);
  assert.equal(captured.options.ballJointLimitDeg, 10);
  assert.equal(captured.options.topology, 'c3_paired');
  assert.deepEqual(captured.options.homeHeightBounds, [50, 450]);
  assert.equal(captured.options.ranges.y.min, -10);
  assert.equal(captured.options.ranges.y.max, 20);
  await element('loadSampleRequirements').handlers.click();
  assert.equal(Number(element('optXMin').value), -40);
  assert.equal(Number(element('ballJointLimit').value), 52);
});

test('UI sends selected topology and edited height bounds to the core', async () => {
  let options;
  class StubOptimizer {
    constructor(_requirements, incoming) { options = incoming; this.pareto = []; this.fitness = []; }
    estimateWork() { return { totalPoses: 8 }; }
    start() { return Promise.resolve({ status: 'completed' }); }
  }
  const element = await loadUI(StubOptimizer);
  element('optTopology').value = 'rectangular_paired';
  element('homeHeightMin').value = '90';
  element('homeHeightMax').value = '300';
  await element('runOptimization').handlers.click();
  assert.equal(options.topology, 'rectangular_paired');
  assert.deepEqual(options.homeHeightBounds, [90, 300]);
});

test('explicit joint option takes precedence, including zero', () => {
  assert.equal(new Optimizer({ ball_joint_max_deg: 52 }, { ballJointLimitDeg: 10 }).ballJointLimitDeg, 10);
  assert.equal(new Optimizer({ ball_joint_max_deg: 52 }, { ballJointLimitDeg: 0 }).ballJointLimitDeg, 0);
  assert.equal(new Optimizer({ ball_joint_max_deg: 52 }).ballJointLimitDeg, 52);
});

test('UI passes sampling presets, grid, seed reuse, and Randomize to core', async () => {
  const captured = [];
  class StubOptimizer {
    constructor(_requirements, options) { captured.push(options); this.pareto = []; this.fitness = []; }
    estimateWork() { return { totalPoses: 8 }; }
    start() { return Promise.resolve({ status: 'completed' }); }
  }
  const random = { getRandomValues(values) { values[0] = 71; return values; } };
  const element = await loadUI(StubOptimizer, { window: { crypto: random, addEventListener() {} } });
  assert.equal(element('optSeed').value || '1', '1');
  for (const count of [256, 1024, 4096]) {
    element('optSampling').value = String(count);
    await element('runOptimization').handlers.click();
    assert.deepEqual(captured.at(-1).sampling, { strategy: 'halton', sampleCount: count });
    assert.equal(captured.at(-1).seed, 1);
  }
  element('optSampling').value = 'grid';
  await element('runOptimization').handlers.click();
  assert.deepEqual(captured.at(-1).sampling, { strategy: 'grid' });
  element('randomizeSeed').handlers.click();
  assert.equal(element('optSeed').value, '71');
  await element('runOptimization').handlers.click();
  assert.equal(captured.at(-1).seed, 71);
  await element('runOptimization').handlers.click();
  assert.equal(captured.at(-1).seed, 71);
});

test('rating controls refresh JSON defaults, retain edits, and reject invalid overrides', async () => {
  const captured = [];
  class StubOptimizer {
    constructor(requirements, options) { captured.push({ requirements, options }); this.pareto = []; this.fitness = []; }
    estimateWork() { return { totalPoses: 8 }; }
    start() { return Promise.resolve({ status: 'completed' }); }
  }
  const element = await loadUI(StubOptimizer);
  const input = JSON.parse(sampleText);
  input.constraints.servo_torque_rating_nm = 8;
  input.constraints.servo_speed_rating_deg_s = 180;
  element('requirementsInput').value = JSON.stringify(input);
  await element('runOptimization').handlers.click();
  assert.equal(captured.at(-1).options.servoRatings.servo_torque_rating_nm, 8);
  assert.equal(captured.at(-1).options.servoRatings.servo_speed_rating_deg_s, 180);
  element('servoTorqueRating').value = '6';
  element('servoTorque1').value = '4';
  element('servoRatingPolicy').value = 'advisory';
  input.constraints.servo_torque_rating_nm = 10;
  element('requirementsInput').value = JSON.stringify(input);
  await element('runOptimization').handlers.click();
  assert.equal(captured.at(-1).requirements.servo_torque_rating_nm, 10);
  assert.equal(captured.at(-1).options.servoRatings.servo_torque_rating_nm, 6);
  assert.equal(captured.at(-1).options.servoRatings.per_servo_ratings[0].torque_nm, 4);
  assert.equal(captured.at(-1).options.servoRatings.servo_rating_policy, 'advisory');
  element('servoSpeed2').value = '0';
  await element('runOptimization').handlers.click();
  assert.match(element('optStatus').textContent, /Servo 2 speed rating/);
  assert.equal(captured.length, 2);
});
