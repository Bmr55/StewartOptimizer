import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { DEFAULT_REACHABILITY_HALF_RANGE_MM, parseReachability, REACHABILITY_CHUNK, REACHABILITY_DEFAULTS,
  REACHABILITY_SAMPLE_COUNTS, reachabilityPoses, reachabilityRanges } from '../../src/simulator/reachability.js';
import { parseWorkspaceRanges } from '../../src/simulator/snapshot.js';
import { createSimulatorView } from '../../src/simulator/view.js';
import { evaluatePose } from '../../src/model/pose.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { SAMPLE_PRESETS } from '../../src/workspace/sampling.js';
import { createFakeDocument } from './helpers.js';

const settings = { ballJointLimitDeg: 180 };
const sampleRanges = () => parseWorkspaceRanges(parseRequirements(fs.readFileSync(
  new URL('../../examples/sample-requirements.json', import.meta.url), 'utf8')).workspace);
const flush = () => new Promise(resolve => setImmediate(resolve));

// Holds every scheduled callback until the test releases it, so each chunk
// boundary is observable.
function manualScheduler() {
  const queue = [];
  return {
    schedule: callback => { queue.push(callback); },
    get pending() { return queue.length; },
    async step() { queue.shift()(); await flush(); },
    async drain() { while (queue.length) await this.step(); },
  };
}

// A stand-in evaluator: records each pose and calls a pose reachable when x >= 0.
function fakeEvaluator() {
  const poses = [];
  const evaluate = (layout, pose) => {
    poses.push(pose);
    return { reachable: pose.x >= 0, translation: [pose.x, pose.y, layout.homeHeight + pose.z] };
  };
  return { poses, evaluate };
}

function sweepingController(options = {}) {
  const scheduler = manualScheduler();
  const fake = fakeEvaluator();
  const controller = createSimulatorController({ schedule: scheduler.schedule, evaluateReachability: fake.evaluate, ...options });
  controller.loadLayout(asymmetricJointFixture(), { options: settings });
  return { controller, scheduler, fake, cloud: () => controller.getState().reachabilityCloud };
}

test('the sweep yields between chunks, honours abort and restarts on an orientation change', async () => {
  const { controller, scheduler, fake, cloud } = sweepingController();
  assert.equal(cloud(), null, 'the cloud is off by default');
  assert.equal(scheduler.pending, 0);
  controller.setReachabilityCloud({ enabled: true });
  assert.equal(controller.getState().overlays.reachabilityCloud, true);
  assert.deepEqual(cloud().points, []);
  assert.equal(cloud().total, REACHABILITY_DEFAULTS.sampleCount);
  assert.equal(fake.poses.length, 0, 'nothing is evaluated before the first yield');
  assert.equal(scheduler.pending, 1);
  await scheduler.step();
  assert.equal(fake.poses.length, REACHABILITY_CHUNK, 'one chunk per yield');
  assert.equal(cloud().points.length, REACHABILITY_CHUNK);
  assert.equal(cloud().progress, REACHABILITY_CHUNK / 1024);
  assert.equal(scheduler.pending, 1, 'the sweep yields again before the next chunk');
  await scheduler.step();
  assert.equal(fake.poses.length, 2 * REACHABILITY_CHUNK);

  // A new orientation aborts the running sweep and starts over from no points.
  controller.requestPose({ rx: 0.1 });
  assert.deepEqual(cloud().points, []);
  assert.equal(cloud().orientation.rx, 0.1);
  await scheduler.drain();
  const restarted = fake.poses.slice(2 * REACHABILITY_CHUNK);
  assert.equal(restarted.length, 1024, 'the aborted sweep evaluated nothing more');
  assert.ok(restarted.every(pose => pose.rx === 0.1 && pose.ry === 0 && pose.rz === 0));
  assert.equal(cloud().progress, 1);
  assert.equal(cloud().points.length, 1024);
  assert.equal(cloud().points.filter(point => point.reachable).length,
    restarted.filter(pose => pose.x >= 0).length);
  assert.ok(Object.isFrozen(cloud().points), 'snapshots share one frozen cloud');

  // A translation-only request keeps the finished cloud.
  const finished = cloud();
  controller.requestPose({ rx: 0.1, x: 5 });
  assert.equal(cloud(), finished);
  assert.equal(scheduler.pending, 0);

  // Switching the cloud off aborts a sweep in progress and drops its points.
  controller.requestPose({ rz: 0.2 });
  await scheduler.step();
  const evaluated = fake.poses.length;
  controller.setOverlays({ reachabilityCloud: false });
  assert.equal(cloud(), null);
  await scheduler.drain();
  assert.equal(fake.poses.length, evaluated);
});

test('layout, option, range and setting changes restart the sweep; clear and dispose stop it', async () => {
  const { controller, scheduler, fake, cloud } = sweepingController();
  controller.setReachabilityCloud({ enabled: true, sampleCount: 256 });
  await scheduler.drain();
  assert.equal(cloud().progress, 1);
  const restarts = [
    () => controller.setOptions({ ballJointLimitDeg: 170 }),
    () => controller.loadLayout(asymmetricJointFixture(), { options: settings }),
    () => controller.setWorkspaceRanges(sampleRanges()),
    () => controller.setReachabilityCloud({ mode: 'slice', sliceZ: 10 }),
    () => controller.setReachabilityCloud({ sampleCount: 1024 }),
  ];
  for (const restart of restarts) {
    restart();
    assert.deepEqual(cloud().points, [], String(restart));
    await scheduler.drain();
    assert.equal(cloud().progress, 1, String(restart));
  }
  assert.ok(cloud().points.every(point => point.at[2] === asymmetricJointFixture().homeHeight + 10), 'slice plane');
  controller.setReachabilityCloud({ mode: 'cloud' });
  await scheduler.step();
  const evaluated = fake.poses.length;
  controller.clear();
  assert.equal(cloud(), null);
  await scheduler.drain();
  assert.equal(fake.poses.length, evaluated, 'clear aborts the sweep');
  controller.loadLayout(asymmetricJointFixture(), { options: settings });
  controller.dispose();
  await scheduler.drain();
  assert.equal(fake.poses.length, evaluated, 'dispose aborts the sweep');
});

test('invalid cloud settings are rejected by name before anything changes', () => {
  const { controller, scheduler } = sweepingController();
  const before = controller.getState();
  assert.throws(() => controller.setReachabilityCloud({ enabled: true, sampleCount: 1000 }), /reachability\.sampleCount must be one of 256, 1024, 4096/);
  assert.throws(() => controller.setReachabilityCloud({ enabled: true, mode: 'volume' }), /reachability\.mode/);
  assert.throws(() => controller.setReachabilityCloud({ enabled: true, sliceZ: NaN }), /reachability\.sliceZ/);
  assert.throws(() => controller.setReachabilityCloud({ enabled: 'yes' }), /reachability\.enabled/);
  assert.throws(() => controller.setReachabilityCloud(null), TypeError);
  assert.deepEqual(controller.getState().reachability, before.reachability);
  assert.equal(controller.getState().overlays.reachabilityCloud, false);
  assert.equal(scheduler.pending, 0);
  assert.deepEqual(REACHABILITY_SAMPLE_COUNTS, SAMPLE_PRESETS);
  assert.deepEqual(parseReachability({ sampleCount: 256, mode: 'slice', sliceZ: -5, extra: 1 }),
    { sampleCount: 256, mode: 'slice', sliceZ: -5 });
});

test('an evaluator error stops the sweep and is published with the points so far', async () => {
  let calls = 0;
  const { controller, scheduler, cloud } = sweepingController({ evaluateReachability: (layout, pose) => {
    if (++calls > 250) throw new RangeError('evaluator broke');
    return { reachable: true, translation: [pose.x, pose.y, pose.z] };
  } });
  controller.setReachabilityCloud({ enabled: true });
  await scheduler.drain();
  assert.equal(cloud().points.length, REACHABILITY_CHUNK);
  assert.equal(cloud().error, 'evaluator broke');
});

test('samples are Halton translations inside the requirement ranges, else a ±100 mm cube, at the requested rotation', () => {
  const ranges = sampleRanges();
  assert.deepEqual(reachabilityRanges(ranges, REACHABILITY_DEFAULTS),
    { x: { min: -40, max: 40 }, y: { min: -40, max: 40 }, z: { min: -20, max: 40 } });
  const half = DEFAULT_REACHABILITY_HALF_RANGE_MM;
  assert.deepEqual(reachabilityRanges(null, REACHABILITY_DEFAULTS),
    { x: { min: -half, max: half }, y: { min: -half, max: half }, z: { min: -half, max: half } });
  assert.deepEqual(reachabilityRanges({ x: { min: 0, max: 5 } }, { mode: 'slice', sliceZ: 7 }),
    { x: { min: 0, max: 5 }, y: { min: -half, max: half }, z: { min: 7, max: 7 } });
  const orientation = { rx: 0.05, ry: -0.02, rz: 0.1 };
  const poses = [...reachabilityPoses(ranges, orientation, { sampleCount: 256, mode: 'cloud', sliceZ: 0 })];
  assert.equal(poses.length, 256);
  assert.equal(new Set(poses.map(pose => `${pose.x},${pose.y},${pose.z}`)).size, 256);
  for (const pose of poses) {
    assert.ok(pose.x >= -40 && pose.x <= 40 && pose.y >= -40 && pose.y <= 40 && pose.z >= -20 && pose.z <= 40);
    assert.deepEqual([pose.rx, pose.ry, pose.rz], [0.05, -0.02, 0.1]);
  }
});

test('with the shared evaluator each point is the evaluated origin and its reachable flag', async () => {
  const scheduler = manualScheduler();
  const controller = createSimulatorController({ schedule: scheduler.schedule });
  const layout = asymmetricJointFixture();
  controller.loadLayout(layout, { options: settings, workspaceRanges: sampleRanges() });
  controller.requestPose({ rx: 0.05 });
  controller.setReachabilityCloud({ enabled: true, sampleCount: 256 });
  await scheduler.drain();
  const { points, progress } = controller.getState().reachabilityCloud;
  assert.equal(progress, 1);
  const poses = [...reachabilityPoses(sampleRanges(), { rx: 0.05, ry: 0, rz: 0 }, { sampleCount: 256, mode: 'cloud', sliceZ: 0 })];
  poses.forEach((pose, k) => {
    const expected = evaluatePose(layout, pose, settings);
    assert.deepEqual(points[k].at, expected.translation);
    assert.equal(points[k].reachable, expected.reachable);
  });
  const reachable = points.filter(point => point.reachable).length;
  assert.ok(reachable > 0 && reachable <= 256);
});

test('the view lists the sample presets, drives the cloud and reports evaluated samples only', async () => {
  const document = createFakeDocument();
  const scheduler = manualScheduler();
  const controller = createSimulatorController({ schedule: scheduler.schedule, evaluateReachability: fakeEvaluator().evaluate });
  const renderer = { available: true, contextLost: false, render() {}, dispose() {} };
  createSimulatorView({ document, window: { addEventListener() {} }, controller, createRenderer: () => renderer });
  const input = id => document.getElementById(id);
  assert.equal(input('simReachabilitySamples').innerHTML,
    '<option value="256">256</option><option value="1024">1,024</option><option value="4096">4,096</option>');
  assert.equal(input('simReachabilityStatus').textContent, 'Reachability cloud off.');
  input('simOverlayReachabilityCloud').checked = true;
  input('simOverlayReachabilityCloud').dispatch('change');
  assert.equal(input('simReachabilityStatus').textContent, 'Load a layout to sweep reachability.');
  controller.loadLayout(asymmetricJointFixture(), { options: settings });
  input('simReachabilitySamples').value = '256';
  input('simReachabilitySamples').dispatch('change');
  assert.equal(controller.getState().reachability.sampleCount, 256);
  // The first queued callback belongs to the aborted 1,024-sample sweep.
  while (!controller.getState().reachabilityCloud.points.length) await scheduler.step();
  assert.match(input('simReachabilityStatus').textContent,
    /^Sweeping 200 of 256 samples at Rx 0, Ry 0, Rz 0°: \d+ reachable\. Evaluated samples only, not a continuous envelope\.$/);
  await scheduler.drain();
  assert.match(input('simReachabilityStatus').textContent, /^Swept 256 of 256 samples/);
  assert.equal(input('simReachabilitySliceZ').disabled, true);
  input('simReachabilitySlice').checked = true;
  input('simReachabilitySlice').dispatch('change');
  assert.equal(input('simReachabilitySliceZ').disabled, false);
  input('simReachabilitySliceZ').value = '12.5';
  input('simReachabilitySliceZ').dispatch('change');
  assert.deepEqual(controller.getState().reachability, { sampleCount: 256, mode: 'slice', sliceZ: 12.5 });
  await scheduler.drain();
  assert.match(input('simReachabilityStatus').textContent, /^Swept 256 of 256 samples on the Z 12\.5 mm plane/);
  // An empty field is an error, never 0, and the stored value comes back.
  input('simReachabilitySliceZ').value = ' ';
  input('simReachabilitySliceZ').dispatch('change');
  assert.equal(controller.getState().reachability.sliceZ, 12.5);
  assert.equal(input('simReachabilitySliceZ').value, '12.5');
  assert.match(input('simReachabilityStatus').textContent, /^reachability\.sliceZ must be a finite number/);
  assert.equal(scheduler.pending, 0, 'a rejected entry does not restart the sweep');
});
