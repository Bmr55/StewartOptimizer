import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { jointFixture } from '../fixtures/layout.js';

test('shipped legacy bundle supports the custom-layout API used by its page', () => {
  const root = new URL('../../archive/simulator/', import.meta.url);
  const html = fs.readFileSync(new URL('index.html', root), 'utf8');
  const context = vm.createContext({ console, window: { addEventListener() {} } });
  for (const name of ['quaternion.min.js', 'stewart.min.js']) {
    assert.ok(html.includes(`src="${name}"`));
    vm.runInContext(fs.readFileSync(new URL(name, root), 'utf8'), context);
  }
  const input = jointFixture();
  const platform = new context.Stewart();
  platform.initCustom({ base_anchors: input.baseAnchors, platform_anchors: input.platformAnchors,
    beta_angles: input.betaAngles, horn_length: input.hornLength, rod_length: input.rodLength,
    servo_range: [-180, 180] });
  platform.update([0, 0, 0], context.Quaternion.ONE);
  assert.equal(platform.computeAngles().length, 6);
  assert.ok(platform.computeAngles().every(Number.isFinite));
  assert.equal(typeof context.Stewart.Animation, 'function');
});
