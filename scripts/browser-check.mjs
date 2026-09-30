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
  const infoButtons = page.locator('.info-button');
  assert.equal(await infoButtons.count(), 30);
  assert.equal(await infoButtons.first().getAttribute('aria-label'), 'More information');
  assert.match(await infoButtons.first().evaluate(button => getComputedStyle(button, '::before').maskImage), /info\.svg/);
  await infoButtons.first().click();
  assert.match(await page.locator('.info-popup.visible').innerText(), /requirements JSON/i);
  await infoButtons.first().click();
  assert.equal(await page.locator('.info-popup.visible').count(), 0);
  const requirementsGuide = page.locator('.requirements-guide');
  assert.equal(await requirementsGuide.locator('table').isVisible(), false);
  await requirementsGuide.locator('summary').click();
  assert.equal(await requirementsGuide.locator('table').isVisible(), true);
  assert.equal(await requirementsGuide.locator('tbody tr').count(), 16);
  assert.equal(await page.locator('a[href="./docs/REQUIREMENTS.md"]').count(), 0);
  await requirementsGuide.locator('summary').click();
  await page.locator('#optPopulation').fill('4');
  await page.locator('#optGenerations').fill('1');
  await page.locator('#optObjectiveSet').selectOption('full');
  await page.locator('#optMutationRate').fill('0');
  await page.locator('#servoTorqueRating').fill('0.000001');
  await page.locator('#servoSpeedRating').fill('180');
  await page.locator('#servoTorque1').fill('2');
  await page.locator('#servoRatingPolicy').selectOption('advisory');
  await page.locator('#runOptimization').click();
  await page.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('Optimization complete'), null, { timeout: 30000 });
  assert.equal(await page.locator('#runPhase').textContent(), 'completed');
  assert.equal(await page.locator('#runCandidates').textContent(), '8 / 8');
  assert.equal(await page.locator('#runGeneration').textContent(), '1 / 1');
  assert.match(await page.locator('#runElapsed').textContent(), /\d+\.\d s/);
  assert.match(await page.locator('#runFrontSize').textContent(), /^\d+$/);
  assert.match(await page.locator('#runBestCandidate').textContent(), /#\d+ ·/);
  assert.equal(await page.locator('#runEta').textContent(), 'Complete');
  const workText = await page.locator('#runPoseWork').textContent();
  const workCounts = workText.match(/^([\d,]+) actual \/ ([\d,]+) budgeted$/);
  assert.ok(workCounts);
  assert.ok(Number(workCounts[1].replaceAll(',', '')) <= Number(workCounts[2].replaceAll(',', '')));

  const candidateSelect = page.locator('#candidateSelect');
  const options = await candidateSelect.locator('option').evaluateAll(nodes => nodes.map(node => node.value));
  assert.equal(options.length, 4);
  const selected = options[1];
  await candidateSelect.selectOption(selected);
  assert.equal(JSON.parse(await page.locator('#resultOutput').inputValue()).result.id, Number(selected));
  await page.locator('#chartXAxis').selectOption('coverage');
  await page.locator('#chartYAxis').selectOption('coverage');
  assert.match(await page.locator('#paretoChart').getAttribute('aria-label'), /coverage \(percent\)/);
  const plotted = await page.evaluate(() => Chart.getChart('paretoChart').data.datasets
    .flatMap(dataset => dataset.data.map(point => point.id)));
  assert.equal(plotted.length, 4);
  assert.ok(plotted.includes(Number(selected)));
  await page.locator('#chartXAxis').selectOption('torque');
  await page.locator('#chartYAxis').selectOption('speedDemand');
  await page.locator('#paretoChart').scrollIntoViewIfNeeded();
  const chartPoint = await page.evaluate(id => {
    const chart = Chart.getChart('paretoChart');
    for (const [datasetIndex, dataset] of chart.data.datasets.entries()) {
      const index = dataset.data.findIndex(point => point.id === Number(id));
      if (index >= 0) {
        const { x, y } = chart.getDatasetMeta(datasetIndex).data[index];
        return { x, y };
      }
    }
  }, options[0]);
  const chartBounds = await page.locator('#paretoChart').boundingBox();
  await page.mouse.click(chartBounds.x + chartPoint.x, chartBounds.y + chartPoint.y);
  await page.waitForFunction(id => document.getElementById('candidateSelect').value === id, options[0]);
  assert.equal(await candidateSelect.inputValue(), options[0]);
  await candidateSelect.selectOption(selected);

  const downloadPromise = page.waitForEvent('download');
  await page.locator('#exportBestLayout').click();
  const downloaded = JSON.parse(await readFile(await (await downloadPromise).path(), 'utf8'));
  assert.equal(downloaded.id, Number(selected));
  assert.equal(downloaded.schema_version, 2);
  assert.equal(downloaded.feasibility.passing, !downloaded.diagnostic);
  assert.equal(downloaded.run.effective_settings.objectiveSet, 'full');
  assert.equal(downloaded.run.effective_settings.objectiveDefinitions.length, 9);
  assert.equal(downloaded.run.effective_settings.mutationRate, 0);
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
  assert.match(await page.locator('#runPhase').textContent(), /^cancelled/);
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
  assert.equal(await fallbackPage.locator('#runPhase').textContent(), 'failed');
  assert.equal(await fallbackPage.locator('#runMainThreadFallback').isVisible(), true);
  assert.equal(await fallbackPage.locator('#runMainThreadFallback').isEnabled(), true);
  await fallbackPage.locator('#runMainThreadFallback').click();
  await fallbackPage.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('Optimization complete'), null, { timeout: 30000 });
  assert.equal(await fallbackPage.locator('#runPhase').textContent(), 'completed');
  assert.equal(await fallbackPage.locator('#runCandidates').textContent(), '8 / 8');

  const errorPage = await browser.newPage();
  await errorPage.route('**/src/ui/optimizer-worker.js', route => route.fulfill({
    contentType: 'text/javascript', body: `
      import { createWorkerRuntime } from './worker-runtime.js';
      import { Optimizer } from '../optimization/optimizer.js';
      class FaultyOptimizer extends Optimizer {
        emitCheckpoint() { super.emitCheckpoint(); throw new Error('forced runtime failure'); }
      }
      const runtime = createWorkerRuntime({ postMessage: message => self.postMessage(message), OptimizerClass: FaultyOptimizer });
      self.addEventListener('message', event => { void runtime.handleMessage(event.data); });
    `,
  }));
  await errorPage.goto(`http://127.0.0.1:${server.address().port}/`);
  await errorPage.waitForFunction(() => document.querySelector('#requirementsInput').value.includes('mass_kg'));
  await errorPage.locator('#optPopulation').fill('4');
  await errorPage.locator('#optGenerations').fill('1');
  await errorPage.locator('#optSampling').selectOption('256');
  await errorPage.locator('#runOptimization').click();
  await errorPage.waitForFunction(() => document.querySelector('#optStatus').textContent.includes('forced runtime failure'), null, { timeout: 30000 });
  assert.equal(await errorPage.locator('#runPhase').textContent(), 'failed · partial results');
  assert.equal(await errorPage.locator('#runCandidates').textContent(), '4 / 8');
  assert.match(await errorPage.locator('#runBestCandidate').textContent(), /#\d+ ·/);
  assert.equal(await errorPage.locator('#exportBestLayout').isEnabled(), true);

  const stalePage = await browser.newPage();
  await stalePage.route('**/src/ui/optimizer-worker.js', route => route.fulfill({
    contentType: 'text/javascript', body: `
      self.addEventListener('message', event => {
        if (event.data.type !== 'start') return;
        const runId = event.data.runId;
        self.postMessage({ type: 'started', runId });
        self.postMessage({ type: 'progress', runId: 'obsolete', snapshot: {
          elapsedMs: 100, completedCandidates: 999, totalCandidates: 8, generation: 99,
          frontSize: 99, bestCandidate: null, actualCompletedPoseWork: 999,
          budgetedPoseWork: 100, etaMs: 0, etaApproximate: true,
        } });
        setTimeout(() => self.postMessage({ type: 'result', runId,
          outcome: { status: 'cancelled', partialResults: false },
          snapshot: { fitness: [], generation: 0 },
          summary: { elapsedMs: 250, completedCandidates: 0, totalCandidates: 8,
            generation: 0, frontSize: 0, bestCandidate: null,
            actualCompletedPoseWork: 0, budgetedPoseWork: 100,
            etaMs: null, etaApproximate: false },
        }), 250);
      });
    `,
  }));
  await stalePage.goto(`http://127.0.0.1:${server.address().port}/`);
  await stalePage.waitForFunction(() => document.querySelector('#requirementsInput').value.includes('mass_kg'));
  await stalePage.locator('#optPopulation').fill('4');
  await stalePage.locator('#optGenerations').fill('1');
  await stalePage.locator('#optSampling').selectOption('256');
  await stalePage.locator('#runOptimization').click();
  await stalePage.waitForTimeout(100);
  assert.equal(await stalePage.locator('#runCandidates').textContent(), '0 / 8');
  await stalePage.waitForFunction(() => document.querySelector('#runPhase').textContent === 'cancelled');
  console.log('Browser dashboard completion, cancellation, runtime error/partial, stale messages, startup fallback, objectives, mutation, ratings, import, selection and export passed.');
} finally {
  await browser?.close();
  await new Promise(resolve => server.close(resolve));
}
