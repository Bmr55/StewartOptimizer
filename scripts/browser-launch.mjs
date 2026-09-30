// Shared Playwright launch for the browser checks. PLAYWRIGHT_CHANNEL selects
// the browser: a Playwright channel such as chrome, chrome-beta, msedge or
// chromium, or "bundled" for the Chromium that `npx playwright install chromium`
// downloads. Unset, the checks try Google Chrome and fall back to the bundled
// Chromium when Chrome is not installed.
export const DEFAULT_CHANNEL = 'chrome';
export const BUNDLED_CHANNEL = 'bundled';
const MISSING_BROWSER = /not found|doesn't exist|does not exist|not installed/i;

export function launchOptions(channel, options = {}) {
  const launch = { headless: true, ...options };
  if (channel !== BUNDLED_CHANNEL) launch.channel = channel;
  return launch;
}

export async function launchBrowser(options = {}, {
  env = process.env, chromium, warn = message => console.warn(message),
} = {}) {
  const browserType = chromium ?? (await import('playwright')).chromium;
  const requested = env.PLAYWRIGHT_CHANNEL;
  const channel = requested || DEFAULT_CHANNEL;
  try {
    return await browserType.launch(launchOptions(channel, options));
  } catch (error) {
    if (requested || !MISSING_BROWSER.test(error?.message ?? '')) throw error;
    warn(`Google Chrome is not available (${String(error.message).split('\n')[0]}); using Playwright's bundled Chromium. Set PLAYWRIGHT_CHANNEL to choose a browser.`);
    return browserType.launch(launchOptions(BUNDLED_CHANNEL, options));
  }
}
