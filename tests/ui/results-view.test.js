import test from 'node:test';
import assert from 'node:assert/strict';
import { createResultsView, chartSvg } from '../../src/ui/results-view.js';

const candidate = id => ({
  layout: { id }, torque: 2, speedDemand: 3, coverage: 80,
  feasibility: { passing: true, homePoseSatisfied: true, sampledWorkspaceSatisfied: true, cycleSatisfied: true },
});

test('coincident candidates retain independent chart and keyboard-list access', () => {
  const a = candidate(1);
  const b = candidate(2);
  const svg = chartSvg([a, b], 1);
  assert.match(svg, /data-candidate-id="1"/);
  assert.match(svg, /data-candidate-id="2"/);
  assert.match(svg, /torque \(N m\)/);
  assert.match(svg, /speed Demand \(rad\/s\)/);

  const elements = new Map();
  const document = { getElementById(id) {
    if (!elements.has(id)) elements.set(id, { value: '', innerHTML: '', textContent: '', handlers: {},
      addEventListener(type, handler) { this.handlers[type] = handler; } });
    return elements.get(id);
  } };
  let selected;
  const view = createResultsView(document, item => { selected = item.layout.id; });
  view.render([a, b], 1);
  const select = document.getElementById('candidateSelect');
  assert.match(select.innerHTML, /Candidate 1/);
  assert.match(select.innerHTML, /Candidate 2/);
  select.value = '2';
  select.handlers.change();
  assert.equal(selected, 2);
  const yAxis = document.getElementById('chartYAxis');
  yAxis.value = 'coverage';
  yAxis.handlers.change();
  assert.match(document.getElementById('paretoChart').innerHTML, /coverage \(percent\)/);
});
