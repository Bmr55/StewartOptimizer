// Development-only check that the WebGL canvas draws the solved pose where an
// independent pinhole camera says it should be. Positions come from the shared
// evaluator and tests/fixtures/independent-geometry.js, never from the renderer.
import assert from 'node:assert/strict';
import { launchBrowser } from './browser-launch.mjs';
import { startServer } from './serve.mjs';
import { layoutToJSON } from '../src/io/results.js';
import { evaluatePose } from '../src/model/pose.js';
import { SCENE_BACKGROUND, SCENE_COLORS } from '../src/simulator/scene.js';
import { asymmetricJointFixture } from '../tests/fixtures/layout.js';
import { pinhole } from '../tests/fixtures/independent-geometry.js';

const layout = asymmetricJointFixture();
const options = { ballJointLimitDeg: 180 };
const pose = { x: 12, y: -7, z: 18, rx: 0.06, ry: -0.04, rz: 0.1 };
const camera = { yaw: 0.9, pitch: 0.35, distance: 650, target: [0, 0, 170] };
const solved = evaluatePose(layout, pose, { ...options, recordLegData: true });
assert.equal(solved.reachable, true, 'the check pose must be reachable');

// Only the legs, base and platform outlines are drawn: no markers or overlays.
const overlays = { groundGrid: false, servoArcs: false, jointCones: false, workspaceBox: false,
  reachabilityCloud: false, conditioningEllipsoid: false, requestedGhost: false, platformAxes: false, worldAxes: false };
const file = { ...layoutToJSON(layout), simulator: { options, requested: pose, accepted: pose, camera, overlays, markers: false } };

const lerp = (a, b, t) => a.map((value, k) => value + (b[k] - value) * t);
const to255 = color => color.map(value => Math.round(value * 255));

const server = await startServer({ port: 0 });
let browser;
try {
  browser = await launchBrowser();
  const page = await browser.newPage({ viewport: { width: 1280, height: 900 }, deviceScaleFactor: 1 });
  const pageErrors = [];
  page.on('pageerror', error => pageErrors.push(error.message));
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.locator('#referenceLayoutInput').fill(JSON.stringify(file));
  await page.locator('#simulateTab').click();
  await page.locator('#simLoadReference').click();
  await page.locator('#simDiagnosticRows tr').first().waitFor();
  assert.match(await page.locator('#simAcceptedDiagnostic').textContent(), /X 12 Y -7 Z 18/);
  const { width, height } = await page.locator('#simCanvas').evaluate(canvas => ({ width: canvas.width, height: canvas.height }));

  // Sample points along every rod and horn, each with the colour that must be
  // drawn there. A perspective camera maps a 3D point on a segment onto the
  // drawn 2D segment, so each sample lies on its drawn line.
  const samples = [];
  for (let leg = 0; leg < 6; leg++) {
    for (const t of [0.25, 0.5, 0.75]) {
      samples.push({ label: `leg ${leg} rod ${t}`, color: to255(SCENE_COLORS.rod),
        pixel: pinhole(lerp(solved.hornTips[leg], solved.platformPoints[leg], t), camera, width, height).pixel });
    }
    samples.push({ label: `leg ${leg} horn`, color: to255(SCENE_COLORS.horn),
      pixel: pinhole(lerp(layout.baseAnchors[leg], solved.hornTips[leg], 0.5), camera, width, height).pixel });
  }
  // World axes, drawn in a second frame: the X, Y and Z axes must appear on the
  // side of the screen the right-handed camera puts them.
  const axes = [[[15, 0, 0], SCENE_COLORS.x], [[0, 15, 0], SCENE_COLORS.y], [[0, 0, 15], SCENE_COLORS.z]]
    .map(([point, color], k) => ({ label: `world axis ${'XYZ'[k]}`, color: to255(color), pixel: pinhole(point, camera, width, height).pixel }));
  // The same samples mirrored left-right and top-bottom: a flipped image would
  // pass there instead, so most of them must miss. The Z axis is left out of the
  // mirrored axes because it stands on the vertical centre line of this view.
  const mirror = (list, flip) => list.map(sample => ({ ...sample,
    pixel: flip === 'x' ? [width - sample.pixel[0], sample.pixel[1]] : [sample.pixel[0], height - sample.pixel[1]] }));

  const hits = await page.evaluate(({ groups, background }) => {
    const canvas = document.querySelector('#simCanvas');
    const gl = canvas.getContext('webgl2');
    // A pixel shows a colour when it is the background blended toward that colour
    // (antialiased line edges) by at least 40%, with little of any other hue.
    const shows = (pixels, x, y, color) => {
      const at = (Math.floor(y) * canvas.width + Math.floor(x)) * 4;
      if (at < 0 || at >= pixels.length || x < 0 || x >= canvas.width) return false;
      const d = [0, 1, 2].map(k => pixels[at + k] - background[k]);
      const c = [0, 1, 2].map(k => color[k] - background[k]);
      const t = (d[0] * c[0] + d[1] * c[1] + d[2] * c[2]) / (c[0] ** 2 + c[1] ** 2 + c[2] ** 2);
      const residual = Math.hypot(...d.map((value, k) => value - t * c[k]));
      return t >= 0.4 && residual < 30;
    };
    // Within two pixels, to allow for rasterization rounding.
    const near = (pixels, [x, y], color) => {
      for (let dy = -2; dy <= 2; dy++) for (let dx = -2; dx <= 2; dx++) if (shows(pixels, x + dx, y + dy, color)) return true;
      return false;
    };
    // Each toggle redraws synchronously; read before the browser composites.
    const frame = (axesOn) => {
      const input = document.getElementById('simOverlayWorldAxes');
      input.checked = axesOn;
      input.dispatchEvent(new Event('change'));
      const pixels = new Uint8Array(canvas.width * canvas.height * 4);
      gl.readPixels(0, 0, canvas.width, canvas.height, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
      return pixels;
    };
    const legs = frame(false), axes = frame(true);
    const check = (pixels, list) => list.map(sample => ({ label: sample.label, hit: near(pixels, sample.pixel, sample.color) }));
    return { legs: check(legs, groups.legs), mirroredX: check(legs, groups.mirroredX), mirroredY: check(legs, groups.mirroredY),
      axes: check(axes, groups.axes), axesMirrored: check(axes, groups.axesMirrored) };
  }, { groups: { legs: samples, mirroredX: mirror(samples, 'x'), mirroredY: mirror(samples, 'y'), axes, axesMirrored: mirror(axes.slice(0, 2), 'x') },
    background: to255(SCENE_BACKGROUND) });

  const missed = list => list.filter(sample => !sample.hit).map(sample => sample.label);
  const hitCount = list => list.filter(sample => sample.hit).length;
  assert.deepEqual(missed(hits.legs), [], 'drawn legs are not at their projected positions');
  assert.deepEqual(missed(hits.axes), [], 'world axes are not at their projected positions');
  assert.ok(hitCount(hits.mirroredX) <= samples.length / 3, `a left-right mirror also matches: ${hitCount(hits.mirroredX)}`);
  assert.ok(hitCount(hits.mirroredY) <= samples.length / 3, `a top-bottom mirror also matches: ${hitCount(hits.mirroredY)}`);
  assert.ok(hitCount(hits.axesMirrored) === 0, 'a mirrored world axis also matches');
  assert.deepEqual(pageErrors, []);
  console.log(`Browser render check passed: ${samples.length} leg samples and 3 world axes drawn at their pinhole positions; `
    + `mirrored images match ${hitCount(hits.mirroredX)} and ${hitCount(hits.mirroredY)} samples.`);
} finally {
  await browser?.close();
  await new Promise(resolve => server.close(resolve));
}
