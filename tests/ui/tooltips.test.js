import test from 'node:test';
import assert from 'node:assert/strict';
import { installTooltips } from '../../src/ui/tooltips.js';

function fakeElement(extra = {}) {
  const classes = new Set();
  const attributes = {};
  return {
    attributes, style: {}, textContent: '',
    classList: { add: name => classes.add(name), remove: name => classes.delete(name), contains: name => classes.has(name) },
    setAttribute(name, value) { attributes[name] = value; },
    getBoundingClientRect: () => ({ left: 40, bottom: 20, width: 16, height: 16 }),
    contains: () => false,
    ...extra,
  };
}

function harness({ requestAnimationFrame = true } = {}) {
  const frames = [];
  const documentHandlers = {};
  const windowHandlers = {};
  const buttons = [
    fakeElement({ dataset: { info: 'First message' }, closest() { return this; } }),
    fakeElement({ dataset: { info: 'Second message' }, closest() { return this; } }),
  ];
  let popup;
  const document = {
    createElement() { popup = fakeElement({ getBoundingClientRect: () => ({ width: 100, height: 40 }) }); return popup; },
    body: { appendChild() {} },
    documentElement: { clientWidth: 300 },
    querySelectorAll: () => buttons,
    addEventListener(type, handler) { documentHandlers[type] = handler; },
  };
  const window = {
    innerWidth: 317, scrollX: 0, scrollY: 100, pageXOffset: 0, pageYOffset: 100,
    addEventListener(type, handler) { windowHandlers[type] = handler; },
    ...(requestAnimationFrame ? { requestAnimationFrame: callback => { frames.push(callback); return frames.length; } } : {}),
  };
  installTooltips(document, window);
  const click = button => documentHandlers.click({ target: button, preventDefault() {}, stopPropagation() {} });
  const runFrame = () => { const pending = frames.splice(0); for (const callback of pending) callback(); };
  return { buttons, popup: () => popup, click, scroll: () => windowHandlers.scroll(), runFrame, frames,
    visible: () => popup.classList.contains('visible') };
}

test('a scroll event in the frame of the opening click does not close the tooltip, later scrolls do', () => {
  const view = harness();
  view.click(view.buttons[0]);
  assert.equal(view.visible(), true);
  assert.equal(view.popup().textContent, 'First message');
  view.scroll();
  assert.equal(view.visible(), true, 'the deferred scroll event closed the popup');
  view.runFrame();
  assert.equal(view.visible(), true);
  view.scroll();
  assert.equal(view.visible(), false);
});

test('switching buttons before the frame ends keeps ignoring the scroll until the last opening frame passes', () => {
  const view = harness();
  view.click(view.buttons[0]);
  const firstFrame = view.frames.slice();
  view.click(view.buttons[1]);
  assert.equal(view.popup().textContent, 'Second message');
  for (const callback of firstFrame) callback();
  view.scroll();
  assert.equal(view.visible(), true, 'the first frame callback re-enabled scroll dismissal early');
  view.runFrame();
  view.scroll();
  assert.equal(view.visible(), false);
});

test('tooltips fall back to a timer without requestAnimationFrame and expose aria-expanded', async () => {
  const view = harness({ requestAnimationFrame: false });
  assert.equal(view.buttons[0].attributes['aria-expanded'], 'false');
  view.click(view.buttons[0]);
  assert.equal(view.buttons[0].attributes['aria-expanded'], 'true');
  assert.equal(view.popup().attributes['aria-label'], 'More information');
  view.scroll();
  assert.equal(view.visible(), true);
  await new Promise(resolve => setTimeout(resolve, 5));
  view.scroll();
  assert.equal(view.visible(), false);
  assert.equal(view.buttons[0].attributes['aria-expanded'], 'false');
});

test('the popup is clamped to the viewport width excluding a classic scrollbar', () => {
  const view = harness();
  view.buttons[0].getBoundingClientRect = () => ({ left: 280, bottom: 20, width: 16, height: 16 });
  view.click(view.buttons[0]);
  // documentElement.clientWidth 300 (not innerWidth 317) minus popup width 100 and the 8 px margin.
  assert.equal(view.popup().style.left, '192px');
  assert.equal(view.popup().style.top, '128px');
});
