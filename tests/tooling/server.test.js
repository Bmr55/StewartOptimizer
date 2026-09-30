import test from 'node:test';
import assert from 'node:assert/strict';
import { startServer } from '../../scripts/serve.mjs';

test('development server serves entry points and blocks private/outside paths', async t => {
  const server = await startServer({port:0});
  t.after(() => new Promise(resolve => server.close(resolve)));
  const origin = `http://127.0.0.1:${server.address().port}`;
  for (const [route, type] of [['/', 'text/html'], ['/src/main.js', 'text/javascript'],
    ['/styles/app.css', 'text/css'], ['/examples/sample-requirements.json', 'application/json'],
    ['/archive/simulator/', 'text/html']]) {
    const response = await fetch(origin + route);
    assert.equal(response.status, 200, route);
    assert.equal(response.headers.get('content-type'), type);
    assert.ok((await response.text()).length);
  }
  for (const route of ['/.git', '/node_modules/package.json', '/%2e%2e%2fpackage.json']) {
    assert.equal((await fetch(origin + route)).status, 403, route);
  }
  assert.equal((await fetch(origin + '/missing-file')).status, 404);
  assert.equal((await fetch(origin, {method:'POST'})).status, 405);
});
