import test from 'node:test';
import assert from 'node:assert/strict';
import { computeWorkspace } from '../../src/workspace/sweep.js';
import { haltonPose, estimateWorkspaceSize, normalizeSampling, radicalInverse, workspacePoses } from '../../src/workspace/sampling.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { resolveMounting } from '../../src/model/mounting.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { degToRad } from '../../src/math.js';

test('Halton presets, bounds, fixed axes, and endpoints use a recorded sequence', () => {
  const ranges = { x: { min: -10, max: 10 }, y: { min: 3, max: 3 }, rx: { min: -90, max: 90 } };
  assert.equal(haltonPose(ranges, 1).x, 0);
  assert.equal(haltonPose(ranges, 1).y, 3);
  assert.ok(Math.abs(haltonPose(ranges, 1).rx - (-90 + 180 / 7) * Math.PI / 180) < 1e-14);
  assert.equal(radicalInverse(2, 2), 0.25);
  for (const sampleCount of [256, 1024, 4096]) {
    assert.equal(estimateWorkspaceSize(ranges, { strategy: 'halton', sampleCount }), sampleCount);
    for (let index = 1; index <= sampleCount; index++) {
      const pose = haltonPose(ranges, index);
      assert.ok(pose.x > -10 && pose.x < 10);
      assert.ok(pose.rx > -Math.PI / 2 && pose.rx < Math.PI / 2);
    }
  }
  assert.deepEqual(normalizeSampling({ strategy: 'halton' }), {
    strategy: 'halton', sampleCount: 1024, sequence: 'halton-v1', sequenceStart: 1, bases: [2, 3, 5, 7, 11, 13],
  });
  assert.equal(estimateWorkspaceSize({ x: { min: 0, max: 1, step: 0.5 } }, { strategy: 'grid' }), 3);
  const grid = [...workspacePoses({ rx: { min: -12, max: 12, step: 12 } }, { strategy: 'grid' })];
  assert.deepEqual(grid.map(pose => pose.rx), [0, 1, 2].map(index => degToRad(-12) + index * degToRad(12)));
});

test('sampling is deterministic, retains bounded examples, and distinguishes strict coverage', async () => {
  const layout = asymmetricJointFixture();
  const mounting = resolveMounting(layout).mounting;
  mounting.lower[0] = { direction: mounting.lower[0].direction.map(value => -value), source: 'supplied' };
  layout.mounting = mounting;
  const ranges = { x: { min: -5, max: 5 }, rz: { min: 0, max: 0 } };
  const sampling = { strategy: 'halton', sampleCount: 256, sequenceStart: 31 };
  const options = { sampling, ballJointLimitDeg: 52, ballJointClamp: true, sampleLimit: 999 };
  const a = await computeWorkspace(layout, ranges, options);
  const b = await computeWorkspace(layout, ranges, options);
  assert.deepEqual(a, b);
  assert.equal(a.total, 256);
  assert.equal(a.counts.reachable + a.counts.unreachable, 256);
  assert.ok(a.samples.unreachable.length <= 200);
  assert.equal(a.coverage, 0);
  assert.ok(a.relaxedCoverage >= a.coverage);
  const strict = await computeWorkspace(layout, ranges, { ...options, ballJointClamp: false });
  assert.equal(strict.coverage, a.coverage);
  assert.equal(strict.relaxedCoverage, strict.coverage);
  const grid = await computeWorkspace(layout, {}, { ballJointLimitDeg: 52, ballJointClamp: true, sampling: { strategy: 'grid' } });
  assert.equal(grid.total, 1);
  assert.equal(grid.coverage, 0);
  assert.equal(grid.relaxedCoverage, 100);
});

test('run budgets include workspace, home, cycle, and reject excess before evaluation', async () => {
  const opt = new Optimizer({ cycle_mm: 10, frequency_hz: 2 }, {
    populationSize: 4, generations: 1, sampling: { strategy: 'halton', sampleCount: 256 },
  });
  assert.deepEqual(opt.estimateWork(), {
    posesPerLayout: 513, workspacePosesPerLayout: 256, payloadChecksPerLayout: 0, cyclePosesPerLayout: 256,
    evaluations: 8, totalPoses: 4104,
  });
  assert.equal(new Optimizer({ cycle_mm: 10, frequency_hz: 2 }, { populationSize: 4, generations: 1,
    cycleSampling: { strategy: 'uniform', samples: 64 } }).estimateWork().cyclePosesPerLayout, 64);
  assert.throws(() => new Optimizer({}, { sampling: { strategy: 'halton', sampleCount: 4096 },
    populationSize: 100, generations: 3 }).estimateWork(), /1,000,000/);
  assert.throws(() => estimateWorkspaceSize({}, { strategy: 'halton', sampleCount: 100001 }), /100,000/);
  assert.throws(() => estimateWorkspaceSize({ x: { min: 2, max: 1 } }, { strategy: 'halton' }), /Sweep ranges/);
});

test('a seeded run is deterministic: effective settings replay evolution and samples identically (results are not pinned)', async () => {
  const options = { populationSize: 4, generations: 1, seed: 73, topology: 'rectangular_paired',
    homeHeightBounds: [80, 300],
    sampling: { strategy: 'halton', sampleCount: 16 }, ranges: { x: { min: -2, max: 2, step: 1 } },
    lowerBallJointLimitDeg: 70, upperBallJointLimitDeg: 80 };
  const first = new Optimizer({ home_height_bounds_mm: [50, 450] }, options);
  const settings = first.effectiveSettings();
  await first.run();
  const replay = new Optimizer(settings.requirements, { ranges: settings.bounds,
    sampling: settings.sampling, seed: settings.seed, populationSize: settings.populationSize,
    generations: settings.generations, mutationRate: settings.mutationRate, topology: settings.topology,
    homeHeightBounds: settings.homeHeightBounds,
    designSpace: settings.designSpace, ballJointLimitDeg: settings.ballJointLimitDeg,
    lowerBallJointLimitDeg: settings.lowerBallJointLimitDeg,
    upperBallJointLimitDeg: settings.upperBallJointLimitDeg,
    ballJointClamp: settings.ballJointClamp });
  assert.equal(settings.lowerBallJointLimitDeg, 70);
  assert.equal(settings.upperBallJointLimitDeg, 80);
  assert.equal(settings.topology, 'rectangular_paired');
  assert.deepEqual(settings.homeHeightBounds, [80, 300]);
  await replay.run();
  const snapshot = opt => opt.fitness.map(({ layout, coverage, relaxedCoverage, workspace, cycle }) => ({
    layout, coverage, relaxedCoverage, samples: workspace.samples, cycle,
  }));
  assert.deepEqual(snapshot(replay), snapshot(first));
  await first.run();
  assert.deepEqual(snapshot(first), snapshot(replay));
  const changed = new Optimizer({}, { ...options, seed: 74 });
  await changed.run();
  assert.notDeepEqual(changed.fitness.map(ev => ev.layout), first.fitness.map(ev => ev.layout));
});
