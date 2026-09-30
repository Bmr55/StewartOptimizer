// Development-only browser interaction check. Uses installed Chrome via Playwright.
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { chromium } from 'playwright';
import { startServer } from './serve.mjs';

const server = await startServer({ port: 0 });
let browser;
try {
  browser = await chromium.launch({ channel: 'chrome', headless: true });
  const page = await browser.newPage({ acceptDownloads: true });
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.locator('#requirementsInput').waitFor({ state: 'visible' });
  await page.waitForFunction(() => document.querySelector('#requirementsInput').value.includes('mass_kg'));
  await page.locator('#optPopulation').fill('4');
  await page.locator('#optGenerations').fill('1');
  await page.locator('#servoTorqueRating').fill('0.000001');
  await page.locator('#servoSpeedRating').fill('180');
  await page.locator('#servoTorque1').fill('2');
  await page.locator('#servoRatingPolicy').selectOption('advisory');
  await page.locator('#runOptimization').click();
  await page.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('Optimization complete'), null, { timeout: 30000 });

  const candidateSelect = page.locator('#candidateSelect');
  const options = await candidateSelect.locator('option').evaluateAll(nodes => nodes.map(node => node.value));
  assert.equal(options.length, 4);
  const selected = options[1];
  await candidateSelect.selectOption(selected);
  assert.equal(JSON.parse(await page.locator('#resultOutput').inputValue()).result.id, Number(selected));
  await page.locator('#chartXAxis').selectOption('coverage');
  await page.locator('#chartYAxis').selectOption('coverage');
  assert.match(await page.locator('#paretoChart').innerHTML(), /coverage \(percent\)/);
  assert.equal(await page.locator(`#paretoChart circle[data-candidate-id="${selected}"]`).count(), 1);

  const downloadPromise = page.waitForEvent('download');
  await page.locator('#exportBestLayout').click();
  const downloaded = JSON.parse(await readFile(await (await downloadPromise).path(), 'utf8'));
  assert.equal(downloaded.id, Number(selected));
  assert.equal(downloaded.schema_version, 2);
  assert.equal(downloaded.feasibility.passing, !downloaded.diagnostic);
  assert.equal(downloaded.run.effective_settings.servoRatingPolicy, 'advisory');
  assert.equal(downloaded.run.effective_settings.effectiveServoRatings.perServo[0].torqueNm, 2);
  assert.equal(downloaded.run.effective_settings.effectiveServoRatings.perServo[1].torqueNm, 0.000001);
  assert.equal(downloaded.servo_capacity.policy, 'advisory');
  assert.match(await page.locator('#candidateSummary').textContent(), /Servo capacity .+ \(advisory(?: warning)?\)/);
  await page.locator('#referenceLayoutInput').fill(JSON.stringify(downloaded));
  await page.locator('#runOptimization').click();
  await page.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('Optimization complete'), null, { timeout: 30000 });
  assert.match(await candidateSelect.innerText(), /Exact reference/);
  await candidateSelect.selectOption('1');
  const imported = JSON.parse(await page.locator('#resultOutput').inputValue()).result.layout;
  for (const field of ['base_anchors', 'platform_anchors', 'beta_angles', 'horn_length',
    'rod_length', 'servo_range', 'home_height']) assert.deepEqual(imported[field], downloaded[field]);
  await page.locator('#referenceLayoutInput').fill('');
  // A second run can be cancelled while the main thread remains responsive.
  await page.locator('#optSampling').selectOption('4096');
  await page.locator('#runOptimization').click();
  assert.equal(await page.locator('#cancelOptimization').isEnabled(), true);
  await page.locator('#cancelOptimization').click();
  await page.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('cancelled'), null, { timeout: 30000 });
  await page.locator('#optSampling').selectOption('256');
  await page.locator('#runOptimization').click();
  await page.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('Optimization complete'), null, { timeout: 30000 });

  // Block module-worker loading and verify that the user must explicitly choose fallback.
  const fallbackPage = await browser.newPage();
  await fallbackPage.route('**/src/ui/optimizer-worker.js', route => route.abort());
  await fallbackPage.goto(`http://127.0.0.1:${server.address().port}/`);
  await fallbackPage.waitForFunction(() => document.querySelector('#requirementsInput').value.includes('mass_kg'));
  await fallbackPage.locator('#optPopulation').fill('4');
  await fallbackPage.locator('#optGenerations').fill('1');
  await fallbackPage.locator('#optSampling').selectOption('256');
  await fallbackPage.locator('#runOptimization').click();
  await fallbackPage.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('Worker startup failed'));
  assert.equal(await fallbackPage.locator('#runMainThreadFallback').isVisible(), true);
  assert.equal(await fallbackPage.locator('#runMainThreadFallback').isEnabled(), true);
  await fallbackPage.locator('#runMainThreadFallback').click();
  await fallbackPage.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('Optimization complete'), null, { timeout: 30000 });
  console.log('Browser worker completion, cancellation/restart, startup fallback, servo ratings, reference import, selection and export passed.');
} finally {
  await browser?.close();
  await new Promise(resolve => server.close(resolve));
}
