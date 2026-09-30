import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../optimizer.js';
import { loadUI } from '../ui/helpers.js';

test('immediate stop cancels before evaluating a population', async () => {
  const opt = new Optimizer({}, { populationSize: 4, generations: 2 });
  const pending = opt.start();
  opt.stop();
  const result = await pending;
  assert.equal(result.status, 'cancelled');
  assert.equal(opt.generation, 0);
  assert.equal(opt.completedEvaluations, 0);
  assert.equal(opt.running, false);
});

test('stop interrupts an active sweep and overlapping runs are rejected', async () => {
  let opt;
  opt = new Optimizer({}, {
    populationSize: 4, generations: 2,
    ranges: { x: { min: -100, max: 100, step: 0.1 } },
    onProgress: ({ completed }) => { if (completed === 256) opt.stop(); }
  });
  const pending = opt.run();
  await assert.rejects(opt.run(), /already running/);
  assert.equal((await pending).status, 'cancelled');
  assert.equal(opt.completedEvaluations, 0);
});

test('a cancelled later generation retains explicitly partial results', async () => {
  let opt;
  opt = new Optimizer({}, { populationSize: 4, generations: 2,
    onProgress: ({ generation }) => { if (generation === 1) opt.stop(); }
  });
  const outcome = await opt.start();
  assert.equal(outcome.status, 'cancelled');
  assert.equal(outcome.partialResults, true);
  assert.equal(outcome.completedGenerations, 0);
  assert.equal(opt.fitness.length, 4);
  let exported;
  opt.download = data => { exported = JSON.parse(data); };
  opt.exportBest();
  assert.equal(exported.run.status, 'cancelled');
  assert.equal(exported.run.partial, true);
});

test('run errors propagate and restore lifecycle state', async () => {
  const opt = new Optimizer({}, { generations: 1000000 });
  await assert.rejects(opt.start(), /1,000,000/);
  assert.equal(opt.runStatus, 'failed');
  assert.equal(opt.running, false);
});

test('UI cancel interrupts a run and restores controls', async () => {
  const element = await loadUI(Optimizer);
  const pending = element('runOptimization').handlers.click();
  assert.equal(element('runOptimization').disabled, true);
  assert.equal(element('cancelOptimization').disabled, false);
  element('cancelOptimization').handlers.click();
  await pending;
  assert.match(element('optStatus').textContent, /cancelled/);
  assert.equal(element('runOptimization').disabled, false);
  assert.equal(element('cancelOptimization').disabled, true);
  assert.equal(element('exportBestLayout').disabled, true);
});
