import test from 'node:test';
import assert from 'node:assert/strict';
import { launchBrowser, launchOptions } from '../../scripts/browser-launch.mjs';

function fakeChromium(failing = {}) {
  const launches = [];
  return {
    launches,
    async launch(options) {
      launches.push(options);
      const channel = options.channel ?? 'bundled';
      if (failing[channel]) throw new Error(failing[channel]);
      return { channel };
    },
  };
}

test('launch options name a Playwright channel except for the bundled Chromium', () => {
  assert.deepEqual(launchOptions('chrome'), { headless: true, channel: 'chrome' });
  assert.deepEqual(launchOptions('msedge', { headless: false }), { headless: false, channel: 'msedge' });
  assert.deepEqual(launchOptions('bundled'), { headless: true });
});

test('the browser checks use Chrome by default and PLAYWRIGHT_CHANNEL overrides it', async () => {
  const chromium = fakeChromium();
  assert.equal((await launchBrowser({}, { env: {}, chromium })).channel, 'chrome');
  assert.equal((await launchBrowser({}, { env: { PLAYWRIGHT_CHANNEL: 'msedge' }, chromium })).channel, 'msedge');
  assert.equal((await launchBrowser({}, { env: { PLAYWRIGHT_CHANNEL: 'bundled' }, chromium })).channel, 'bundled');
  assert.equal((await launchBrowser({}, { env: { PLAYWRIGHT_CHANNEL: '' }, chromium })).channel, 'chrome');
  assert.deepEqual(chromium.launches.map(options => options.channel), ['chrome', 'msedge', undefined, 'chrome']);
});

test('a missing Chrome falls back to the bundled Chromium only when no channel was requested', async () => {
  const notFound = "browserType.launch: Chromium distribution 'chrome' is not found at /opt/google/chrome/chrome";
  const warnings = [];
  const chromium = fakeChromium({ chrome: notFound });
  const browser = await launchBrowser({}, { env: {}, chromium, warn: message => warnings.push(message) });
  assert.equal(browser.channel, 'bundled');
  assert.deepEqual(chromium.launches, [{ headless: true, channel: 'chrome' }, { headless: true }]);
  assert.equal(warnings.length, 1);
  assert.match(warnings[0], /bundled Chromium.*PLAYWRIGHT_CHANNEL/);
  await assert.rejects(launchBrowser({}, { env: { PLAYWRIGHT_CHANNEL: 'chrome' }, chromium, warn() {} }), /not found/);
  const crashing = fakeChromium({ chrome: 'Target page, context or browser has been closed' });
  await assert.rejects(launchBrowser({}, { env: {}, chromium: crashing, warn() {} }), /has been closed/);
  assert.equal(crashing.launches.length, 1);
});
