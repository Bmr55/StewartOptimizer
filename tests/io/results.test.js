import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { displayResult, selectBest } from '../../src/io/results.js';
import { loadUI } from '../ui/helpers.js';
import { jointFixture } from '../fixtures/layout.js';

test('headless export and display retain their documented fields and partial status', async () => {
  const opt = new Optimizer({}, {ballJointLimitDeg:100});
  const result = await opt.evaluateLayout(jointFixture());
  opt.fitness = [result];
  opt.runStatus = 'cancelled';
  const exported = JSON.parse(opt.exportBest());
  const display = displayResult(result);
  assert.equal(exported.metadata.coverage, display.metrics.coverage);
  assert.deepEqual(exported.base_anchors, display.layout.base_anchors);
  assert.deepEqual(exported.workspace_counts, display.workspace_counts);
  assert.deepEqual(exported.run, {status:'cancelled',completedGenerations:0,partial:true});
  assert.ok(display.workspace_samples);
  assert.equal('workspace_samples' in exported, false);
  const second = {...result,coverage:0};
  assert.equal(selectBest([second], [result]), second);
  assert.equal(selectBest([], [second,result]), result);
});

test('UI passes headless export data to its browser download adapter', async () => {
  let downloaded;
  const element = await loadUI(Optimizer, {downloadFile: data => { downloaded = JSON.parse(data); }});
  await element('runOptimization').handlers.click();
  element('exportBestLayout').handlers.click();
  assert.equal(downloaded.run.status, 'completed');
  assert.equal(downloaded.base_anchors.length, 6);
  assert.equal(downloaded.metadata.coverage, JSON.parse(element('resultOutput').value).result.metrics.coverage);
});
