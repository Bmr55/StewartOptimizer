import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { layoutToJSON } from '../../src/io/results.js';
import { jointFixture } from '../fixtures/layout.js';
import { loadUI } from './helpers.js';

test('reference input stays separate from requirements and remains selectable for exact export', async () => {
  let exported;
  const element = await loadUI(Optimizer, { downloadFile: json => { exported = JSON.parse(json); } });
  const source = layoutToJSON(jointFixture());
  source.topology = undefined;
  source.topology_parameters = undefined;
  source.base_anchors[0][0] += 8;
  source.home_height = 600;
  source.servo_range = [-130, 110];
  source.mounting = undefined;
  const requirementsBefore = element('requirementsInput').value;
  await element('referenceLayoutFile').handlers.change({ target: { files: [{
    name: 'reference.json', text: async () => JSON.stringify(source),
  }] } });
  assert.equal(element('requirementsInput').value, requirementsBefore);
  assert.equal(JSON.parse(element('referenceLayoutInput').value).home_height, 600);
  await element('runOptimization').handlers.click();
  assert.equal(element('optTopology').value, 'free');
  assert.match(element('candidateSelect').innerHTML, /Exact reference/);
  assert.match(element('optStatus').textContent, /Reference: .*bounds conflict/);
  element('candidateSelect').value = '1';
  element('candidateSelect').handlers.change();
  const display = JSON.parse(element('resultOutput').value);
  assert.equal(display.result.layout.id, 1);
  assert.equal(display.result.layout.home_height, 600);
  element('exportBestLayout').handlers.click();
  assert.deepEqual(exported.base_anchors, source.base_anchors);
  assert.deepEqual(exported.servo_range, source.servo_range);
  assert.equal(exported.home_height, 600);
  element('clearReferenceLayout').handlers.click();
  assert.equal(element('referenceLayoutInput').value, '');
  assert.equal(element('requirementsInput').value, requirementsBefore);
});
