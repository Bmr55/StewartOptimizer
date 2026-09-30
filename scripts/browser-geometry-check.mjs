// Development-only browser check for active simulator mechanical controls.
import assert from 'node:assert/strict';
import { chromium } from 'playwright';
import { startServer } from './serve.mjs';

const server = await startServer({ port: 0 });
let browser;
try {
  browser = await chromium.launch({ channel: 'chrome', headless: true });
  const page = await browser.newPage();
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.evaluate(async () => {
    const [{ Optimizer }, { createSimulatorController }, { createGeometryControls }] = await Promise.all([
      import('/src/optimization/optimizer.js'),
      import('/src/simulator/controller.js'),
      import('/src/simulator/geometry-controls.js'),
    ]);
    const layout = new Optimizer({}, { topology: 'c3_paired', seed: 37 }).createRandomLayout();
    const controller = createSimulatorController();
    const container = document.createElement('div');
    container.id = 'simGeometryControls';
    document.body.appendChild(container);
    createGeometryControls({ document, container, controller });
    controller.loadLayout(layout, { source: { kind: 'candidate', candidateId: 19 },
      options: { ballJointLimitDeg: 180 } });
    window.geometryCheck = { controller, original: structuredClone(layout) };
  });

  const original = await page.evaluate(() => window.geometryCheck.original);
  await page.locator('#sim-base-radius-number').fill('135');
  await page.locator('#sim-base-radius-number').press('Tab');
  let state = await page.evaluate(() => window.geometryCheck.controller.getState());
  assert.equal(state.source.kind, 'editable');
  assert.equal(state.source.candidateId, 19);
  assert.equal(state.layout.topologyParameters.base_radius, 135);
  assert.notDeepEqual(state.layout.baseAnchors, original.baseAnchors);
  assert.equal(await page.locator('#sim-base-radius-range').inputValue(), '135');
  assert.deepEqual(await page.evaluate(() => window.geometryCheck.original), original);

  const betas = state.layout.betaAngles;
  await page.locator('#sim-platform-orientation-range').evaluate(node => {
    node.value = '45';
    node.dispatchEvent(new Event('input', { bubbles: true }));
  });
  state = await page.evaluate(() => window.geometryCheck.controller.getState());
  assert.equal(Number(await page.locator('#sim-platform-orientation-number').inputValue()), 45);
  assert.deepEqual(state.layout.betaAngles, betas);
  assert.equal(state.assessment.reachable, state.accepted !== null);

  await page.locator('#sim-reset-geometry').click();
  state = await page.evaluate(() => window.geometryCheck.controller.getState());
  assert.deepEqual(state.layout, original);
  assert.equal(state.source.kind, 'candidate');
  await page.locator('#sim-switch-explicit').click();
  state = await page.evaluate(() => window.geometryCheck.controller.getState());
  assert.equal(state.layout.topology, 'free');
  assert.deepEqual(state.layout.baseAnchors, original.baseAnchors);
  await page.locator('#sim-base-0-x-number').fill('123.5');
  await page.locator('#sim-base-0-x-number').press('Tab');
  await page.locator('#sim-hornLength-number').fill('81.5');
  await page.locator('#sim-hornLength-number').press('Tab');
  state = await page.evaluate(() => window.geometryCheck.controller.getState());
  assert.equal(state.layout.baseAnchors[0][0], 123.5);
  assert.equal(state.layout.hornLength, 81.5);
  assert.deepEqual(state.layout.baseAnchors.slice(1), original.baseAnchors.slice(1));
  await page.locator('#sim-generate-topology').selectOption('circular');
  await page.locator('#sim-generate-parametric').click();
  state = await page.evaluate(() => window.geometryCheck.controller.getState());
  assert.equal(state.layout.topology, 'circular');
  assert.notDeepEqual(state.layout.baseAnchors, original.baseAnchors);
  assert.ok(Math.abs(Number(await page.locator('#sim-beta-pair-offset-number').inputValue()) - 30) < 1e-10);
  const circularAnchors = state.layout.baseAnchors;
  const circularBetas = state.layout.betaAngles;
  await page.locator('#sim-beta-pair-offset-number').fill('45');
  await page.locator('#sim-beta-pair-offset-number').press('Tab');
  state = await page.evaluate(() => window.geometryCheck.controller.getState());
  assert.equal(state.layout.topologyParameters.beta_pair_offset, Math.PI / 4);
  assert.deepEqual(state.layout.baseAnchors, circularAnchors);
  assert.notDeepEqual(state.layout.betaAngles, circularBetas);
  const circularLayout = state.layout;
  await page.locator('#sim-reset-geometry').click();
  state = await page.evaluate(() => window.geometryCheck.controller.getState());
  assert.deepEqual(state.layout, original);
  // An older layout without the optional field still exposes a zero-valued
  // control, even when replacing a new layout of the same topology.
  await page.evaluate(layout => window.geometryCheck.controller.loadLayout(layout), circularLayout);
  await page.evaluate(async () => {
    const { topologyGeometry } = await import('/src/optimization/topology.js');
    const { controller } = window.geometryCheck;
    const layout = controller.getReferenceLayout();
    delete layout.topologyParameters.beta_pair_offset;
    Object.assign(layout, topologyGeometry(layout.topology, layout.topologyParameters));
    controller.loadLayout(layout);
  });
  assert.equal(await page.locator('#sim-beta-pair-offset-number').inputValue(), '0');
  console.log('Browser geometry sliders, explicit mode, editable copy, diagnostics, and reset passed.');
} finally {
  await browser?.close();
  await new Promise(resolve => server.close(resolve));
}
