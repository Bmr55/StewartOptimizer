// Development-only active-simulator diagnostics interaction check.
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { launchBrowser } from './browser-launch.mjs';
import { startServer } from './serve.mjs';
import { layoutToJSON } from '../src/io/results.js';
import { asymmetricJointFixture } from '../tests/fixtures/layout.js';

const server = await startServer({ port: 0 });
let browser;
try {
  browser = await launchBrowser();
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

  // Overlay toggles change what is drawn: count non-background pixels right
  // after each synchronous redraw, before the browser composites and clears.
  const overlayPixels = await page.evaluate(() => {
    const canvas = document.querySelector('#simCanvas');
    const gl = canvas.getContext('webgl2');
    const count = () => {
      const pixels = new Uint8Array(canvas.width * canvas.height * 4);
      gl.readPixels(0, 0, canvas.width, canvas.height, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
      let drawn = 0;
      for (let i = 0; i < pixels.length; i += 4) if (Math.abs(pixels[i] - 14) + Math.abs(pixels[i + 1] - 19) + Math.abs(pixels[i + 2] - 28) > 12) drawn++;
      return drawn;
    };
    const toggle = (id, checked) => {
      const input = document.getElementById(id);
      input.checked = checked;
      input.dispatchEvent(new Event('change'));
      return count();
    };
    // A joint-limit edit redraws the socket cones within the same change event.
    const upperLimit = value => {
      const input = document.getElementById('simUpperJointLimit');
      input.value = value;
      input.dispatchEvent(new Event('change'));
      return count();
    };
    const all = toggle('simOverlayWorldAxes', true);
    // The Z 100 request above is rejected, so its ghost is part of the scene.
    const withoutGhost = toggle('simOverlayRequestedGhost', false);
    const withGhost = toggle('simOverlayRequestedGhost', true);
    const narrowCones = upperLimit('20');
    const wideCones = upperLimit('120');
    const withoutCones = toggle('simOverlayJointCones', false);
    const withoutArcs = toggle('simOverlayServoArcs', false);
    const withoutAxes = toggle('simOverlayWorldAxes', false);
    return { all, withoutGhost, withGhost, narrowCones, wideCones, withoutCones, withoutArcs, withoutAxes,
      withoutEither: toggle('simOverlayPlatformAxes', false), restored: toggle('simOverlayPlatformAxes', true) };
  });
  assert.ok(overlayPixels.withoutGhost < overlayPixels.all, `the rejected request drew no ghost: ${JSON.stringify(overlayPixels)}`);
  assert.equal(overlayPixels.withGhost, overlayPixels.all);
  assert.ok(overlayPixels.narrowCones < overlayPixels.all, `a narrower joint limit did not shrink the cones: ${JSON.stringify(overlayPixels)}`);
  assert.equal(overlayPixels.wideCones, overlayPixels.all);
  assert.ok(overlayPixels.all > overlayPixels.withoutCones, `joint cones toggle drew nothing: ${JSON.stringify(overlayPixels)}`);
  assert.ok(overlayPixels.withoutCones > overlayPixels.withoutArcs, `servo arcs toggle drew nothing: ${JSON.stringify(overlayPixels)}`);
  assert.ok(overlayPixels.withoutArcs > overlayPixels.withoutAxes, `world axes toggle drew nothing: ${JSON.stringify(overlayPixels)}`);
  assert.ok(overlayPixels.withoutAxes > overlayPixels.withoutEither, `platform axes toggle drew nothing: ${JSON.stringify(overlayPixels)}`);
  assert.equal(overlayPixels.restored, overlayPixels.withoutAxes);

  const downloadPromise = page.waitForEvent('download');
  await page.locator('#simDownload').click();
  const exported = JSON.parse(await readFile(await (await downloadPromise).path(), 'utf8'));
  assert.equal(exported.simulator.animation.pattern, 'none');
  assert.equal(exported.simulator.options.upperBallJointLimitDeg, 120);
  assert.equal(exported.simulator.options.rodLengthTolerance, 0.01);
  assert.equal(exported.simulator.requested.z, 100);
  assert.equal(exported.simulator.accepted.z, 0);
  assert.deepEqual(exported.simulator.overlays, { servoArcs: false, jointCones: false, requestedGhost: true,
    platformAxes: true, worldAxes: false });
  await page.locator('#optimizeTab').click();
  await page.locator('#referenceLayoutInput').fill(JSON.stringify(exported));
  await page.locator('#simulateTab').click();
  await page.locator('#simLoadReference').click();
  assert.equal(await page.locator('#simRodTolerance').inputValue(), '0.01');
  assert.equal(await page.locator('#simOverlayServoArcs').isChecked(), false);
  assert.equal(await page.locator('#simOverlayJointCones').isChecked(), false);
  assert.equal(await page.locator('#simOverlayRequestedGhost').isChecked(), true);
  assert.equal(await page.locator('#simOverlayWorldAxes').isChecked(), false);
  assert.equal(await page.locator('#simOverlayPlatformAxes').isChecked(), true);
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
  // A pose field being typed into is not rewritten by animation frames; the
  // committed value pauses the animation and is requested (#106).
  await page.locator('#simPlay').click();
  assert.equal(await page.locator('#simPlay').textContent(), 'Pause');
  await page.locator('#simXInput').focus();
  await page.keyboard.press('Control+a');
  await page.keyboard.type('1');
  await page.evaluate(() => new Promise(resolve => setTimeout(resolve, 120)));
  assert.equal(await page.locator('#simXInput').inputValue(), '1', 'typed pose text was rewritten by the frame loop');
  await page.keyboard.press('Tab');
  assert.equal(await page.locator('#simPlay').textContent(), 'Play');
  assert.match(await page.locator('#simPoseStatus').textContent(), /Accepted request: X 1,/);
  // A lost WebGL2 context is reported, restored and redrawn.
  const contextLoss = await page.evaluate(async () => {
    const gl = document.querySelector('#simCanvas').getContext('webgl2');
    const extension = gl?.getExtension('WEBGL_lose_context');
    if (!extension) return null;
    const status = () => document.querySelector('#simPoseStatus').textContent;
    extension.loseContext();
    await new Promise(resolve => setTimeout(resolve, 50));
    const lost = { status: status(), isLost: gl.isContextLost() };
    extension.restoreContext();
    await new Promise(resolve => setTimeout(resolve, 200));
    return { lost, restored: { status: status(), isLost: gl.isContextLost() } };
  });
  if (contextLoss) {
    assert.match(contextLoss.lost.status, /WebGL2 context lost/);
    assert.equal(contextLoss.lost.isLost, true);
    assert.equal(contextLoss.restored.isLost, false, 'the browser was not allowed to restore the context');
    assert.match(contextLoss.restored.status, /Accepted request/);
  } else console.warn('WEBGL_lose_context is unavailable; the context-loss check was skipped.');
  // A request just past a joint limit leaves its ghost almost on top of the held
  // pose. The failing legs are red on the ghost instead of the held legs, and the
  // ghost's depth bias must keep them visible there (not hidden by the held legs).
  const nearMiss = await page.evaluate(() => {
    const canvas = document.querySelector('#simCanvas');
    const gl = canvas.getContext('webgl2');
    const red = () => {
      const pixels = new Uint8Array(canvas.width * canvas.height * 4);
      gl.readPixels(0, 0, canvas.width, canvas.height, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
      let count = 0;
      for (let i = 0; i < pixels.length; i += 4) if (pixels[i] > 200 && pixels[i + 1] < 90 && pixels[i + 2] < 110) count++;
      return count;
    };
    const check = (id, checked) => {
      const input = document.getElementById(id);
      input.checked = checked;
      input.dispatchEvent(new Event('change'));
      return red();
    };
    const change = (id, value) => {
      const input = document.getElementById(id);
      input.value = value;
      input.dispatchEvent(new Event('change'));
    };
    for (const id of ['simMarkers', 'simOverlayServoArcs', 'simOverlayJointCones']) check(id, false);
    check('simOverlayRequestedGhost', true);
    document.getElementById('simResetPose').click();
    change('simUpperJointLimit', '10');
    // Coarse steps find the limit, then 0.05° steps from the last accepted pose
    // leave the ghost within 0.05° of the held pose.
    const firstRejected = (from, step) => {
      for (let rx = from; rx <= 30; rx = Math.round((rx + step) * 100) / 100) {
        change('simRXInput', String(rx));
        const status = document.querySelector('#simPoseStatus').textContent;
        if (status.startsWith('Rejected')) return { rx, status };
      }
      return null;
    };
    const coarse = firstRejected(0.5, 0.5);
    if (!coarse) return null;
    change('simRXInput', String(coarse.rx - 0.5));
    const found = firstRejected(coarse.rx - 0.45, 0.05);
    const ghostOn = red();
    return { ...found, ghostOn, ghostOff: check('simOverlayRequestedGhost', false),
      restored: check('simOverlayRequestedGhost', true) };
  });
  assert.ok(nearMiss, 'no Rx up to 30° exceeded a 10° upper joint limit');
  assert.match(nearMiss.status, /ballJoint/, nearMiss.status);
  assert.ok(nearMiss.ghostOff > 0, `held failing legs drew no red: ${JSON.stringify(nearMiss)}`);
  // Measured: 97 % of the red survives with the bias, 71 % without it.
  assert.ok(nearMiss.ghostOn > nearMiss.ghostOff * 0.9,
    `ghost failure legs were hidden behind the held pose: ${JSON.stringify(nearMiss)}`);
  assert.equal(nearMiss.restored, nearMiss.ghostOn);
  for (const id of ['#simMarkers', '#simOverlayServoArcs', '#simOverlayJointCones']) await page.locator(id).check();
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
  console.log('Browser evaluator diagnostics, rejected/accepted pose, settings, overlay toggles, animation replay, JSON round trip and optimizer transfer passed.');
} finally {
  await browser?.close();
  await new Promise(resolve => server.close(resolve));
}
