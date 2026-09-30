import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { WorkerOptimizer, WorkerStartupError } from '../../src/ui/worker-optimizer.js';
import { createWorkerRuntime } from '../../src/ui/worker-runtime.js';
import { optionsFromEffectiveSettings, progressSnapshot } from '../../src/ui/worker-protocol.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { layoutToJSON } from '../../src/io/results.js';
import { jointFixture } from '../fixtures/layout.js';
import { sampleText, loadUI } from './helpers.js';

function idleWorker() {
  let worker;
  const opt = new WorkerOptimizer({}, { populationSize: 4, generations: 1 }, {
    workerFactory: () => (worker = { postMessage() {}, terminate() { this.terminated = true; }, onmessage: null, onerror: null }),
  });
  const pending = opt.start();
  return { opt, pending, runId: opt.runId, reply: data => worker.onmessage({ data }), worker: () => worker };
}

function connectedWorker() {
  const workers = [];
  const workerFactory = () => {
    const worker = {
      onmessage: null, onerror: null, terminated: false,
      postMessage(message) { queueMicrotask(() => { void runtime.handleMessage(message); }); },
      terminate() { this.terminated = true; },
    };
    const runtime = createWorkerRuntime({ postMessage: message => {
      queueMicrotask(() => worker.onmessage?.({ data: structuredClone(message) }));
    } });
    workers.push(worker);
    return worker;
  };
  return { workerFactory, workers };
}

test('worker and headless paths replay the same seed, population, and selected export', async () => {
  const { normalized, workspace } = parseRequirements(sampleText);
  const options = { populationSize: 4, generations: 1, ranges: workspace,
    sampling: { strategy: 'halton', sampleCount: 256 }, seed: 713 };
  const headless = new Optimizer(normalized, options);
  const { workerFactory, workers } = connectedWorker();
  const browser = new WorkerOptimizer(normalized, options, { workerFactory });
  assert.equal((await headless.start()).status, 'completed');
  assert.equal((await browser.start()).status, 'completed');
  assert.deepEqual(browser.fitness, headless.fitness);
  assert.deepEqual(browser.pareto, headless.pareto);
  assert.deepEqual(JSON.parse(browser.exportBest()), JSON.parse(headless.exportBest()));
  assert.equal(workers.length, 1);
  assert.equal(workers[0].terminated, true);
});

test('imported asymmetric reference and condition limit replay across worker and headless paths', async () => {
  const { normalized, workspace } = parseRequirements(sampleText);
  const reference = layoutToJSON(jointFixture());
  reference.topology = undefined;
  reference.topology_parameters = undefined;
  reference.base_anchors[0][0] += 8;
  reference.mounting = undefined;
  const options = { populationSize: 4, generations: 1, ranges: workspace,
    sampling: { strategy: 'halton', sampleCount: 256 }, seed: 81,
    referenceLayout: reference, conditionLimit: 500 };
  const headless = new Optimizer(normalized, options);
  const { workerFactory } = connectedWorker();
  const browser = new WorkerOptimizer(normalized, options, { workerFactory });
  assert.equal((await headless.start()).status, 'completed');
  assert.equal((await browser.start()).status, 'completed');
  assert.deepEqual(browser.fitness, headless.fitness);
  assert.deepEqual(browser.pareto, headless.pareto);
  assert.deepEqual(JSON.parse(browser.exportBest()), JSON.parse(headless.exportBest()));
  assert.equal(browser.fitness.some(candidate => candidate.layout.seedOrigin === 'reference'), true);
});

test('advisory and enforced servo ratings retain per-servo sources and worker/headless results', async () => {
  const { normalized, workspace } = parseRequirements(sampleText);
  for (const policy of ['advisory', 'enforced']) {
    const options = { populationSize: 4, generations: 1, ranges: workspace,
      sampling: { strategy: 'halton', sampleCount: 256 }, seed: 91,
      servoRatings: { servo_torque_rating_nm: 0.000001, servo_speed_rating_deg_s: 180,
        per_servo_ratings: [{ torque_nm: 2 }, null, null, null, null, null],
        servo_rating_policy: policy } };
    const headless = new Optimizer(normalized, options);
    const { workerFactory } = connectedWorker();
    const browser = new WorkerOptimizer(normalized, options, { workerFactory });
    assert.equal((await headless.start()).status, 'completed');
    assert.equal((await browser.start()).status, 'completed');
    assert.deepEqual(browser.fitness, headless.fitness);
    assert.deepEqual(browser.pareto, headless.pareto);
    assert.deepEqual(JSON.parse(browser.exportBest()), JSON.parse(headless.exportBest()));
    assert.deepEqual(browser.effectiveSettings().servoRatings, headless.effectiveSettings().servoRatings);
    assert.deepEqual(browser.effectiveSettings().effectiveServoRatings,
      headless.effectiveSettings().effectiveServoRatings);
    assert.equal(browser.effectiveSettings().effectiveServoRatings.perServo[0].source.torque, 'override');
    assert.equal(browser.effectiveSettings().servoRatingPolicy, policy);
  }
});

test('worker adapter rejects stale run messages and keeps only complete-population checkpoints', async () => {
  let worker;
  const opt = new WorkerOptimizer({}, { populationSize: 4, generations: 1 }, {
    workerFactory: () => (worker = { postMessage() {}, terminate() {}, onmessage: null, onerror: null }),
  });
  const pending = opt.start();
  const runId = opt.runId;
  worker.onmessage({ data: { type: 'started', runId: 'stale' } });
  worker.onmessage({ data: { type: 'checkpoint', runId: 'stale', snapshot: { fitness: Array(4).fill({}) } } });
  assert.equal(opt.fitness.length, 0);
  worker.onmessage({ data: { type: 'started', runId } });
  worker.onmessage({ data: { type: 'checkpoint', runId, snapshot: { fitness: [{}], generation: 1 } } });
  assert.equal(opt.fitness.length, 0);
  await assert.rejects(opt.start(), /already running/);
  worker.onmessage({ data: { type: 'result', runId, outcome: { status: 'cancelled' }, snapshot: { fitness: [], generation: 0 } } });
  assert.equal((await pending).status, 'cancelled');
  assert.equal(opt.fitness.length, 0);
  assert.equal(opt.running, false);
});

test('worker runtime rejects overlap and cancels only its active run', async () => {
  const messages = [];
  let release;
  let activeOptimizer;
  class DeferredOptimizer {
    constructor(_requirements, options) { activeOptimizer = this; this.options = options; this.stopCount = 0; this.populationSize = 4; this.generations = 1; this.pareto = []; this.fitness = []; }
    estimateWork() { return { totalPoses: 8 }; }
    run() { return new Promise(resolve => { release = resolve; }); }
    stop() { this.stopCount++; }
  }
  const runtime = createWorkerRuntime({ postMessage: message => messages.push(message), OptimizerClass: DeferredOptimizer });
  const pending = runtime.handleMessage({ type: 'start', runId: 'a', settings: { requirements: {}, bounds: {} } });
  assert.equal(runtime.activeRunId, 'a');
  await runtime.handleMessage({ type: 'start', runId: 'b', settings: {} });
  assert.deepEqual(messages.at(-1), { type: 'error', runId: 'b', phase: 'overlap', message: 'An optimization is already running.' });
  await runtime.handleMessage({ type: 'cancel', runId: 'stale' });
  assert.equal(activeOptimizer.stopCount, 0);
  await runtime.handleMessage({ type: 'cancel', runId: 'a' });
  assert.equal(activeOptimizer.stopCount, 1);
  release({ status: 'cancelled', partialResults: false });
  await pending;
  assert.equal(runtime.activeRunId, null);
  assert.equal(messages.at(-1).type, 'result');
});

test('worker cancellation keeps the last complete population and supports a fresh run', async () => {
  const { normalized, workspace } = parseRequirements(sampleText);
  let opt;
  let workersCreated = 0;
  const workerFactory = () => {
    workersCreated++;
    const worker = {
      onmessage: null, onerror: null, terminated: false,
      postMessage(message) { queueMicrotask(() => { void runtime.handleMessage(message); }); },
      terminate() { this.terminated = true; },
    };
    const runtime = createWorkerRuntime({ postMessage: message => {
      queueMicrotask(() => {
        worker.onmessage?.({ data: structuredClone(message) });
        if (message.type === 'checkpoint' && message.snapshot.generation === 0) opt.stop();
      });
    } });
    return worker;
  };
  opt = new WorkerOptimizer(normalized, { populationSize: 4, generations: 2, ranges: workspace,
    sampling: { strategy: 'halton', sampleCount: 256 },
    servoRatings: { servo_torque_rating_nm: 1, servo_rating_policy: 'advisory' } }, { workerFactory });
  const outcome = await opt.start();
  assert.equal(outcome.status, 'cancelled');
  assert.equal(outcome.partialResults, true);
  assert.equal(opt.fitness.length, 4);
  assert.equal(opt.fitness[0].servoCapacity.policy, 'advisory');
  assert.equal(opt.generation, 0);
  assert.equal(opt.running, false);
  assert.equal(JSON.parse(opt.exportBest()).run.partial, true);
  // The next invocation creates a fresh module worker rather than reusing the cancelled one.
  const second = await opt.start();
  assert.equal(second.status, 'cancelled');
  assert.equal(workersCreated, 2);
});

test('bounded progress contains summaries rather than layout or population payloads', () => {
  const snapshot = progressSnapshot({ completedEvaluations: 3, populationSize: 4, generations: 1,
    pareto: [], fitness: [], workEstimate: { totalPoses: 100 } },
  { completed: 20, generation: 0 }, 500);
  assert.deepEqual(Object.keys(snapshot), ['elapsedMs', 'completedCandidates', 'totalCandidates', 'generation',
    'frontSize', 'bestCandidate', 'actualCompletedPoseWork', 'budgetedPoseWork', 'etaMs', 'etaApproximate']);
  assert.equal(snapshot.etaApproximate, true);
  assert.equal(snapshot.etaMs, 500 / 3 * 5);
});

test('worker reconstruction retains reference seed and later engineering options', () => {
  const reference = { id: 12, base_anchors: [[1, 2, 3]] };
  const ratings = { servo_torque_rating_nm: 3, per_servo_ratings: [{ speed_deg_s: 80 }] };
  const options = optionsFromEffectiveSettings({ bounds: { x: { min: 0, max: 0, step: 1 } },
    reference_layout: reference, conditionLimit: 900, servoRatings: ratings,
    effectiveServoRatings: { policy: 'enforced' } });
  assert.deepEqual(options.ranges, options.bounds);
  assert.deepEqual(options.referenceLayout, reference);
  assert.equal(options.conditionLimit, 900);
  assert.deepEqual(options.servoRatings, ratings);
});

test('worker progress is throttled to at most 10 Hz', async () => {
  let clock = 0;
  const messages = [];
  class ChattyOptimizer {
    constructor(_requirements, options) {
      this.options = options;
      this.populationSize = 4; this.generations = 1; this.generation = 0;
      this.completedEvaluations = 0; this.pareto = []; this.fitness = [];
      this.workEstimate = { totalPoses: 100 };
    }
    estimateWork() { return this.workEstimate; }
    async run() {
      for (let i = 0; i < 20; i++) {
        clock += 20;
        this.options.onProgress({ completed: i, total: 100, generation: 0 });
      }
      return { status: 'completed' };
    }
  }
  const runtime = createWorkerRuntime({ now: () => clock,
    postMessage: message => messages.push(message), OptimizerClass: ChattyOptimizer });
  await runtime.handleMessage({ type: 'start', runId: 'throttle', settings: { requirements: {}, bounds: {} } });
  assert.equal(messages.filter(message => message.type === 'progress').length, 4);
  assert.equal(messages.at(-1).type, 'result');
  assert.ok(messages.every(message => message.runId === 'throttle'));
});

test('runtime failure retains a completed checkpoint with explicit partial metadata', async () => {
  let worker;
  const opt = new WorkerOptimizer({}, { populationSize: 4, generations: 1 }, {
    workerFactory: () => (worker = { postMessage() {}, terminate() {}, onmessage: null, onerror: null }),
  });
  const pending = opt.start();
  const runId = opt.runId;
  const fitness = Array.from({ length: 4 }, (_, id) => ({ layout: { ...jointFixture(), id: id + 1 }, coverage: id }));
  worker.onmessage({ data: { type: 'started', runId } });
  worker.onmessage({ data: { type: 'checkpoint', runId, snapshot: {
    generation: 0, completedCandidates: 4, fitness, paretoIds: [4],
  } } });
  worker.onmessage({ data: { type: 'error', runId, phase: 'runtime', message: 'crash' } });
  const outcome = await pending;
  assert.equal(outcome.status, 'failed');
  assert.equal(outcome.error, 'crash');
  assert.equal(outcome.partialResults, true);
  assert.equal(opt.fitness.length, 4);
  assert.equal(opt.pareto[0].layout.id, 4);
  assert.equal(opt.running, false);
});

test('startup failure offers explicit fallback, then a later worker run can restart', async () => {
  let attempts = 0;
  const { workerFactory } = connectedWorker();
  const element = await loadUI(Optimizer, { workerFactory: (...args) => {
    attempts++;
    if (attempts === 1) throw new Error('blocked worker');
    return workerFactory(...args);
  } });
  await element('runOptimization').handlers.click();
  assert.match(element('optStatus').textContent, /Worker startup failed/);
  assert.equal(element('runMainThreadFallback').hidden, false);
  assert.equal(element('runMainThreadFallback').disabled, false);
  await element('runMainThreadFallback').handlers.click();
  assert.match(element('optStatus').textContent, /Optimization complete/);
  assert.equal(JSON.parse(element('resultOutput').value).run.status, 'completed');
  await element('runOptimization').handlers.click();
  assert.equal(attempts, 2);
  assert.match(element('optStatus').textContent, /Optimization complete/);
});

test('a startup timeout never silently invokes the fallback', async () => {
  const opt = new WorkerOptimizer({}, { populationSize: 4, generations: 1 }, {
    workerFactory: () => ({ postMessage() {}, terminate() {} }), startupTimeoutMs: 5,
  });
  await assert.rejects(opt.start(), WorkerStartupError);
  assert.equal(opt.startupFailure, true);
  assert.equal(opt.mode, 'worker');
  assert.equal(opt.running, false);
});

test('worker start propagates callback errors without leaving a pending run', async () => {
  let worker;
  const opt = new WorkerOptimizer({}, { populationSize: 4, generations: 1 }, {
    workerFactory: () => (worker = { postMessage() {}, terminate() {}, onmessage: null, onerror: null }),
  });
  const pending = opt.start(() => { throw new Error('callback failed'); });
  const runId = opt.runId;
  worker.onmessage({ data: { type: 'started', runId } });
  worker.onmessage({ data: { type: 'result', runId,
    outcome: { status: 'completed' }, snapshot: { fitness: [] } } });
  await assert.rejects(pending, /callback failed/);
  assert.equal(opt.running, false);
});

test('a result without an outcome fails the run instead of leaving it pending', async () => {
  const { opt, pending, runId, reply, worker } = idleWorker();
  reply({ type: 'started', runId });
  reply({ type: 'result', runId, snapshot: { fitness: [] } });
  const outcome = await pending;
  assert.equal(outcome.status, 'failed');
  assert.match(outcome.error, /missing its outcome/);
  assert.equal(outcome.partialResults, false);
  assert.equal(opt.running, false);
  assert.equal(opt.runStatus, 'failed');
  assert.equal(worker().terminated, true);
});

test('checkpoint and result candidates that fail the layout check end the run and keep the last valid population', async () => {
  const valid = Array.from({ length: 4 }, (_, id) => ({ layout: { ...jointFixture(), id: id + 1 }, coverage: id }));
  const cases = [
    [{ fitness: [null, null, null, null] }, /candidate 0 is not an object/],
    [{ fitness: valid.map(({ coverage }) => ({ coverage })) }, /candidate 0: Layout is required/],
    [{ fitness: valid.map(item => ({ ...item, layout: { id: item.layout.id, baseAnchors: 'x' } })) }, /candidate 0: Layout must provide six base anchors/],
    [{ fitness: valid.map(item => ({ ...item, layout: { ...item.layout, id: undefined } })) }, /candidate 0 layout has no id/],
    [{ fitness: valid.map(item => ({ ...item, layout: { ...item.layout, servoRangeRad: null } })) }, /candidate 0 layout must provide two finite servo bounds/],
  ];
  for (const type of ['checkpoint', 'result']) {
    for (const [snapshot, expected] of cases) {
      const { opt, pending, runId, reply } = idleWorker();
      reply({ type: 'started', runId });
      reply({ type: 'checkpoint', runId, snapshot: { generation: 0, completedCandidates: 4, fitness: valid, paretoIds: [4] } });
      assert.equal(opt.fitness.length, 4);
      reply({ type, runId, outcome: { status: 'completed' }, snapshot: { generation: 1, completedCandidates: 8, ...snapshot } });
      const outcome = await pending;
      assert.equal(outcome.status, 'failed', `${type}: ${expected}`);
      assert.match(outcome.error, expected);
      assert.equal(outcome.partialResults, true);
      assert.equal(opt.fitness.length, 4, 'the last valid population was replaced');
      assert.equal(opt.generation, 0);
      assert.equal(opt.pareto[0].layout.id, 4);
      assert.equal(opt.running, false);
      assert.doesNotThrow(() => JSON.parse(opt.exportBest()));
    }
  }
});

test('a runtime error with a corrupt final snapshot still reports the worker message', async () => {
  const { opt, pending, runId, reply } = idleWorker();
  reply({ type: 'started', runId });
  reply({ type: 'error', runId, phase: 'runtime', message: 'crash', snapshot: { fitness: [null, null, null, null] } });
  const outcome = await pending;
  assert.equal(outcome.error, 'crash');
  assert.equal(outcome.partialResults, false);
  assert.equal(opt.running, false);
});

test('an error reply whose message is not a string still gives readable failure text', async () => {
  const runtime = idleWorker();
  runtime.reply({ type: 'started', runId: runtime.runId });
  runtime.reply({ type: 'error', runId: runtime.runId, phase: 'runtime', message: { code: 7 } });
  const outcome = await runtime.pending;
  assert.equal(outcome.status, 'failed');
  assert.equal(outcome.error, 'The optimization worker reported an error.');
  const startup = idleWorker();
  startup.reply({ type: 'error', runId: startup.runId, phase: 'startup', name: 'TypeError', message: undefined });
  await assert.rejects(startup.pending, error => error instanceof WorkerStartupError
    && error.message === 'The optimization worker reported an error.');
  const shipped = idleWorker();
  shipped.reply({ type: 'error', runId: shipped.runId, phase: 'startup', name: 'TypeError',
    message: 'start settings must be an object with a requirements object.' });
  await assert.rejects(shipped.pending, error => error instanceof WorkerStartupError
    && error.message === 'start settings must be an object with a requirements object.');
});

test('an empty onerror and a worker factory that returns nothing give readable startup failures', async () => {
  const { opt, pending, worker } = idleWorker();
  worker().onerror(undefined);
  await assert.rejects(pending, error => error instanceof WorkerStartupError && /Worker failed to start/.test(error.message));
  assert.equal(opt.running, false);
  const nothing = new WorkerOptimizer({}, { populationSize: 4, generations: 1 }, { workerFactory: () => null });
  await assert.rejects(nothing.start(), error => error instanceof WorkerStartupError && /did not return a worker/.test(error.message));
  assert.equal(nothing.running, false);
  assert.equal(nothing.startupFailure, true);
});

test('worker runtime ignores an idle cancel without runId, a duplicate start, and rejects settings without requirements', async () => {
  const messages = [];
  let release;
  let constructed = 0;
  class DeferredOptimizer {
    constructor() { constructed++; this.populationSize = 4; this.generations = 1; this.pareto = []; this.fitness = []; }
    estimateWork() { return { totalPoses: 8 }; }
    run() { return new Promise(resolve => { release = resolve; }); }
    stop() {}
  }
  const runtime = createWorkerRuntime({ postMessage: message => messages.push(message), OptimizerClass: DeferredOptimizer });
  await runtime.handleMessage({ type: 'cancel' });
  await runtime.handleMessage({ type: 'cancel', runId: undefined });
  for (const settings of [undefined, null, 'abc', 42, [], {}, { requirements: 'x' }]) {
    await runtime.handleMessage({ type: 'start', runId: 'bad', settings });
    assert.deepEqual(messages.at(-1), { type: 'error', runId: 'bad', phase: 'startup', name: 'TypeError',
      message: 'start settings must be an object with a requirements object.' });
  }
  assert.equal(constructed, 0);
  assert.equal(runtime.activeRunId, null);
  const pending = runtime.handleMessage({ type: 'start', runId: 'a', settings: { requirements: {}, bounds: {} } });
  assert.equal(runtime.activeRunId, 'a');
  const before = messages.length;
  await runtime.handleMessage({ type: 'start', runId: 'a', settings: { requirements: {}, bounds: {} } });
  assert.equal(messages.length, before, 'a duplicate start for the live run posted a message');
  assert.equal(runtime.activeRunId, 'a');
  assert.equal(constructed, 1);
  await runtime.handleMessage({ type: 'cancel' });
  assert.equal(runtime.activeRunId, 'a');
  release({ status: 'completed' });
  await pending;
  assert.equal(messages.at(-1).type, 'result');
});

test('the UI recovers from a malformed worker result: Run is enabled again and the failure is shown', async () => {
  let worker;
  const element = await loadUI(Optimizer, { workerFactory: () => (worker = {
    onmessage: null, onerror: null, terminated: false,
    postMessage(message) {
      if (message.type !== 'start') return;
      queueMicrotask(() => {
        worker.onmessage({ data: { type: 'started', runId: message.runId } });
        worker.onmessage({ data: { type: 'result', runId: message.runId, snapshot: { fitness: [] } } });
      });
    },
    terminate() { this.terminated = true; },
  }) });
  await element('runOptimization').handlers.click();
  assert.match(element('optStatus').textContent, /missing its outcome/);
  assert.equal(element('runOptimization').disabled, false);
  assert.equal(element('cancelOptimization').disabled, true);
  assert.equal(worker.terminated, true);
});
