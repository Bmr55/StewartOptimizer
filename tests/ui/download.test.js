import test from 'node:test';
import assert from 'node:assert/strict';
import { download, REVOKE_DELAY_MS } from '../../src/ui/download.js';

function fakeDocument() {
  const anchors = [];
  return {
    anchors,
    createElement(tag) {
      const anchor = { tag, clicked: 0, removed: false, click() { this.clicked++; }, remove() { this.removed = true; } };
      anchors.push(anchor);
      return anchor;
    },
    body: { appended: [], appendChild(node) { this.appended.push(node); } },
  };
}

test('download revokes the Blob URL only after the click has had a chance to start the navigation', () => {
  const document = fakeDocument();
  const revoked = [];
  const timers = [];
  const originalRevoke = URL.revokeObjectURL;
  URL.revokeObjectURL = url => revoked.push(url);
  try {
    download('{"a":1}', 'layout.json', 'application/json', document,
      { setTimeout: (fn, delay) => timers.push({ fn, delay }) });
    const [anchor] = document.anchors;
    assert.equal(anchor.clicked, 1);
    assert.equal(anchor.removed, true);
    assert.equal(anchor.download, 'layout.json');
    assert.match(anchor.href, /^blob:/);
    assert.deepEqual(document.body.appended, [anchor]);
    assert.deepEqual(revoked, [], 'URL was revoked synchronously after the click');
    assert.equal(timers.length, 1);
    assert.equal(timers[0].delay, REVOKE_DELAY_MS);
    timers[0].fn();
    assert.deepEqual(revoked, [anchor.href]);
  } finally {
    URL.revokeObjectURL = originalRevoke;
  }
});

test('download still removes the anchor and schedules the revoke when the click throws', () => {
  const document = fakeDocument();
  const timers = [];
  document.createElement = () => {
    const anchor = { removed: false, click() { throw new Error('blocked'); }, remove() { this.removed = true; } };
    document.anchors.push(anchor);
    return anchor;
  };
  assert.throws(() => download('x', 'x.txt', 'text/plain', document,
    { setTimeout: (fn, delay) => timers.push({ fn, delay }) }), /blocked/);
  assert.equal(document.anchors[0].removed, true);
  assert.equal(timers.length, 1);
  timers[0].fn();
});

test('download uses the real timer by default and the URL stays valid until it fires', async () => {
  const document = fakeDocument();
  const revoked = [];
  const originalRevoke = URL.revokeObjectURL;
  URL.revokeObjectURL = url => revoked.push(url);
  try {
    download('x', 'x.txt', 'text/plain', document, { revokeDelayMs: 0 });
    assert.deepEqual(revoked, []);
    await new Promise(resolve => setTimeout(resolve, 5));
    assert.equal(revoked.length, 1);
  } finally {
    URL.revokeObjectURL = originalRevoke;
  }
});
