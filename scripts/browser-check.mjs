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
  await page.locator('#referenceLayoutInput').fill(JSON.stringify(downloaded));
  await page.locator('#runOptimization').click();
  await page.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('Optimization complete'), null, { timeout: 30000 });
  assert.match(await candidateSelect.innerText(), /Exact reference/);
  await candidateSelect.selectOption('1');
  const imported = JSON.parse(await page.locator('#resultOutput').inputValue()).result.layout;
  for (const field of ['base_anchors', 'platform_anchors', 'beta_angles', 'horn_length',
    'rod_length', 'servo_range', 'home_height']) assert.deepEqual(imported[field], downloaded[field]);
  console.log('Browser result selection, chart axes, selected JSON export, and reference import passed.');
} finally {
  await browser?.close();
  await new Promise(resolve => server.close(resolve));
}
