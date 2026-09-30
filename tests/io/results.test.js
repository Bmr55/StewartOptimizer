import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { displayResult, exportResult, failureCategories, isPassing, selectBest } from '../../src/io/results.js';
import { loadUI } from '../ui/helpers.js';
import { asymmetricJointFixture, jointFixture } from '../fixtures/layout.js';

test('headless export and display retain their documented fields and partial status', async () => {
  const opt = new Optimizer({}, {ballJointLimitDeg:100});
  const result = await opt.evaluateLayout(asymmetricJointFixture());
  opt.fitness = [result];
  opt.runStatus = 'cancelled';
  const exported = JSON.parse(opt.exportBest());
  const display = displayResult(result);
  assert.equal(exported.metadata.coverage, display.metrics.coverage);
  assert.deepEqual(exported.base_anchors, display.layout.base_anchors);
  assert.deepEqual(exported.workspace_counts, display.workspace_counts);
  assert.equal(exported.run.status, 'cancelled');
  assert.equal(exported.run.completedGenerations, 0);
  assert.equal(exported.run.partial, true);
  assert.equal(exported.run.effective_settings.populationSize, 12);
  assert.equal(exported.schema_version, 2);
  assert.equal(exported.model_version, 2);
  assert.ok(display.workspace_samples);
  assert.equal('workspace_samples' in exported, false);
  const second = {...result,coverage:0, feasibility: {
    ...result.feasibility, sampledWorkspaceSatisfied: false,
    failedCategories: ['workspace'], passing: false,
  }};
  assert.equal(selectBest([second], [result]), result);
  assert.equal(selectBest([], [second,result]), result);
});

test('UI passes headless export data to its browser download adapter', async () => {
  let downloaded;
  class HeadlessOptimizer extends Optimizer {}
  const element = await loadUI(HeadlessOptimizer, {downloadFile: data => { downloaded = JSON.parse(data); }});
  await element('runOptimization').handlers.click();
  element('downloadSelected').handlers.click();
  assert.equal(downloaded.run.status, 'completed');
  assert.equal(downloaded.base_anchors.length, 6);
  assert.equal(downloaded.metadata.coverage, JSON.parse(element('resultOutput').value).result.metrics.coverage);
});

test('display and download JSON share one degree conversion for servo_range', async () => {
  const opt = new Optimizer({ servo_travel_bounds_deg: [-177, 177] }, { ballJointLimitDeg: 100 });
  const result = await opt.evaluateLayout(opt.createRandomLayout());
  opt.fitness = [result];
  const exported = JSON.parse(opt.exportBest());
  assert.deepEqual(displayResult(result).layout.servo_range, exported.servo_range);
});

test('display and export metrics drop non-finite values and conditioning omits jacobianRows', () => {
  const evaluation = { layout: { ...jointFixture(), id: 3 }, coverage: NaN, torque: Infinity, speedDemand: 2,
    conditioningQuality: -Infinity, feasibility: { passing: true, failedCategories: [] },
    conditioning: { home: { conditionNumber: 4, jacobianRows: [[1, 0, 0, 0, 0, 0]] }, worst: { conditionNumber: 9 } } };
  const display = displayResult(evaluation);
  // Checked before any JSON round trip: JSON.stringify would hide Infinity as null.
  assert.equal(display.metrics.coverage, null);
  assert.equal(display.metrics.torque, null);
  assert.equal(display.metrics.conditioningQuality, null);
  assert.equal(display.metrics.speedDemand, 2);
  assert.equal(display.metrics.dexterity, null);
  assert.equal('jacobianRows' in display.conditioning.home, false);
  assert.equal(display.conditioning.home.conditionNumber, 4);
  assert.equal(display.conditioning.worst.conditionNumber, 9);
  assert.equal(evaluation.conditioning.home.jacobianRows.length, 1, 'the evaluation itself was modified');
  const exported = exportResult(evaluation, { status: 'completed' });
  assert.equal(exported.metadata.coverage, null);
  assert.equal(exported.metadata.torque, null);
  assert.equal(exported.metadata.speed_demand, 2);
  assert.equal('jacobianRows' in exported.conditioning.home, false);
  assert.equal(exported.conditioning.home.conditionNumber, 4);
  assert.equal(displayResult({ ...evaluation, conditioning: undefined }).conditioning, null);
});

test('an explicit passing: false and cycle.valid === false are failures without any other flag', () => {
  const layout = { ...jointFixture(), id: 1 };
  const explicit = { layout, feasibility: { passing: false, failedCategories: [] } };
  assert.deepEqual(failureCategories(explicit), []);
  assert.equal(isPassing(explicit), false);
  assert.equal(displayResult(explicit).diagnostic, true);
  assert.equal(displayResult(explicit).feasibility.passing, false);
  const invalidCycle = { layout, feasibility: { passing: true }, cycle: { valid: false } };
  assert.deepEqual(failureCategories(invalidCycle), ['cycle']);
  assert.equal(isPassing(invalidCycle), false);
  assert.deepEqual(displayResult(invalidCycle).feasibility.failedCategories, ['cycle']);
  assert.equal(exportResult(invalidCycle, {}).diagnostic, true);
  assert.equal(isPassing({ layout, feasibility: { passing: true }, cycle: { valid: true } }), true);
  assert.equal(isPassing({ layout, feasibility: {} }), true);
});
