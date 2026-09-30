// Development-only active-simulator diagnostics interaction check.
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { chromium } from 'playwright';
import { startServer } from './serve.mjs';
import { layoutToJSON } from '../src/io/results.js';
import { asymmetricJointFixture } from '../tests/fixtures/layout.js';

const server = await startServer({ port: 0 });
let browser;
try {
  browser = await chromium.launch({ channel: 'chrome', headless: true });
  const page = await browser.newPage({ acceptDownloads: true });
  const pageErrors = [];
  page.on('pageerror', error => pageErrors.push(error.message));
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.locator('#referenceLayoutInput').fill(JSON.stringify(layoutToJSON(asymmetricJointFixture())));
  await page.locator('#simulateTab').click();
  await page.locator('#simLoadReference').click();
  await page.locator('#simDiagnosticRows tr').first().waitFor();
  assert.equal(await page.locator('#simDiagnosticRows tr').count(), 6);
  assert.match(await page.locator('#simRequestedDiagnostic').textContent(), /accepted/);
  assert.match(await page.locator('#simAcceptedDiagnostic').textContent(), /X 0 Y 0 Z 0/);

  await page.locator('#simZInput').fill('100');
  await page.locator('#simZInput').press('Tab');
  assert.match(await page.locator('#simRequestedDiagnostic').textContent(), /rejected/);
  assert.match(await page.locator('#simAcceptedDiagnostic').textContent(), /X 0 Y 0 Z 0/);
  assert.ok(await page.locator('#simDiagnosticRows tr.sim-leg-failure').count() > 0);
  await page.locator('#simUpperJointLimit').fill('120');
  await page.locator('#simUpperJointLimit').press('Tab');
  await page.locator('#simRodTolerance').fill('0.01');
  await page.locator('#simRodTolerance').press('Tab');
  assert.match(await page.locator('#simConditionPolicy').textContent(), /120° upper; rod tolerance ±0.01 mm/);

  const downloadPromise = page.waitForEvent('download');
  await page.locator('#simDownload').click();
  const exported = JSON.parse(await readFile(await (await downloadPromise).path(), 'utf8'));
  assert.equal(exported.simulator.animation.pattern, 'none');
  assert.equal(exported.simulator.options.upperBallJointLimitDeg, 120);
  assert.equal(exported.simulator.options.rodLengthTolerance, 0.01);
  assert.equal(exported.simulator.requested.z, 100);
  assert.equal(exported.simulator.accepted.z, 0);
  await page.locator('#optimizeTab').click();
  await page.locator('#referenceLayoutInput').fill(JSON.stringify(exported));
  await page.locator('#simulateTab').click();
  await page.locator('#simLoadReference').click();
  assert.equal(await page.locator('#simRodTolerance').inputValue(), '0.01');
  assert.match(await page.locator('#simRequestedDiagnostic').textContent(), /rejected/);
  assert.match(await page.locator('#simAcceptedDiagnostic').textContent(), /X 0 Y 0 Z 0/);
  assert.equal(await page.locator('#simPattern').inputValue(), 'wobble');
  await page.locator('#simResetPose').click();
  // Check Play synchronously, then pause before the first frame so a physical
  // pose rejection cannot mask whether the restored pattern starts correctly.
  const playingText = await page.locator('#simPlay').evaluate(button => {
    button.click();
    const text = button.textContent;
    button.click();
    return text;
  });
  assert.equal(playingText, 'Pause');
  assert.deepEqual(pageErrors, []);
  await page.locator('#optimizeTab').click();
  await page.locator('#referenceLayoutInput').fill('');
  // Parameters live in a collapsed details panel.
  await page.locator('#optimizationParameters summary').click();
  await page.locator('#optPopulation').fill('4');
  await page.locator('#optGenerations').fill('1');
  await page.locator('#optSampling').selectOption('256');
  await page.locator('#ballJointLimit').fill('60');
  await page.locator('#runOptimization').click();
  await page.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('Optimization complete'), null, { timeout: 30000 });
  await page.locator('#simulateTab').click();
  assert.equal(await page.locator('#simLowerJointLimit').inputValue(), '60');
  assert.equal(await page.locator('#simUpperJointLimit').inputValue(), '60');
  console.log('Browser evaluator diagnostics, rejected/accepted pose, settings, animation replay, JSON round trip and optimizer transfer passed.');
} finally {
  await browser?.close();
  await new Promise(resolve => server.close(resolve));
}
