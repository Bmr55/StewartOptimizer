import test from 'node:test';
import assert from 'node:assert/strict';
import { createResultsView, chartData } from '../../src/ui/results-view.js';

const candidate = id => ({
  layout: { id }, torque: 2, speedDemand: 3, coverage: 80,
  feasibility: { passing: true, homePoseSatisfied: true, sampledWorkspaceSatisfied: true, cycleSatisfied: true },
});

test('coincident candidates retain independent chart and keyboard-list access', () => {
  const a = candidate(1);
  const b = candidate(2);
  const points = chartData([a, b], 1).datasets[0];
  assert.deepEqual(points.data.map(point => point.id), [1, 2]);
  assert.deepEqual(points.pointRadius, [8, 6]);

  const elements = new Map();
  const document = { getElementById(id) {
    if (!elements.has(id)) elements.set(id, { value: '', innerHTML: '', textContent: '', handlers: {},
      setAttribute(name, value) { this[name] = value; },
      addEventListener(type, handler) { this.handlers[type] = handler; } });
    return elements.get(id);
  } };
  let chart;
  class FakeChart {
    constructor(_canvas, config) {
      chart = this;
      this.data = config.data;
      this.options = config.options;
      this.spread = config.plugins[0];
    }
    update() {}
  }
  let selected;
  const view = createResultsView(document, item => { selected = item.layout.id; }, FakeChart);
  view.render([a, b], 1);
  assert.deepEqual(chart.data.datasets[0].data.map(point => point.id), [1, 2]);
  assert.equal(chart.options.scales.x.title.text, 'torque (N m)');
  const select = document.getElementById('candidateSelect');
  assert.match(select.innerHTML, /Candidate 1/);
  assert.match(select.innerHTML, /Candidate 2/);
  select.value = '2';
  select.handlers.change();
  assert.equal(selected, 2);
  const yAxis = document.getElementById('chartYAxis');
  yAxis.value = 'coverage';
  yAxis.handlers.change();
  assert.equal(chart.options.scales.y.title.text, 'coverage (percent)');
  assert.match(document.getElementById('paretoChart')['aria-label'], /coverage \(percent\)/);
  const xAxis = document.getElementById('chartXAxis');
  xAxis.value = 'coverage';
  xAxis.handlers.change();
  assert.equal(chart.options.scales.x.title.text, 'coverage (percent)');
  assert.deepEqual(chart.data.datasets[0].data.map(point => point.x), [a.coverage, b.coverage], 'points follow the X axis metric');
  assert.match(document.getElementById('paretoChart')['aria-label'], /coverage \(percent\) versus coverage \(percent\)/);
  xAxis.value = 'torque';
  xAxis.handlers.change();
  assert.deepEqual(chart.data.datasets[0].data.map(point => point.x), [a.torque, b.torque]);
  chart.options.onClick(null, [{ datasetIndex: 0, index: 0 }], chart);
  assert.equal(selected, 1);

  const rendered = [{ x: 40, y: 50 }, { x: 40, y: 50 }];
  chart.getDatasetMeta = index => ({ data: index === 0 ? rendered : [] });
  chart.spread.afterDatasetsUpdate(chart);
  assert.notDeepEqual([rendered[0].x, rendered[0].y], [rendered[1].x, rendered[1].y]);
});
