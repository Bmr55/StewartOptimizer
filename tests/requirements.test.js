import test from 'node:test';
import assert from 'node:assert/strict';
import { parseRequirements } from '../requirements.js';
import { Optimizer } from '../optimizer.js';
import { sampleText, loadUI } from './ui-helper.js';

for (const format of ['nested', 'flat']) {
  const make = () => format === 'nested' ? JSON.parse(sampleText) : parseRequirements(sampleText).normalized;
  const set = (data, section, field, value) => { (format === 'nested' ? data[section] : data)[field] = value; };
  test(`${format} rejects invalid payloads and constraint bounds with field errors`, () => {
    const cases = [
      ['payload', 'mass_kg', -2], ['payload', 'mass_kg', null],
      ['payload', 'frequency_hz', 'invalid'], ['payload', 'cycle_mm', -1],
      ['constraints', 'rod_length_bounds_mm', [400, 100]],
      ['constraints', 'rod_length_bounds_mm', [0, 100]],
      ['constraints', 'horn_length_bounds_mm', [20, 30, 40]],
      ['constraints', 'horn_length_bounds_mm', null],
      ['constraints', 'servo_travel_bounds_deg', [10, -10]],
      ['constraints', 'servo_travel_bounds_deg', ['0', 10]],
      ['constraints', 'ball_joint_max_deg', 181],
      ['constraints', 'ball_joint_max_deg', null],
      ['constraints', 'servo_max_deg', -90],
      ['workspace', 'x_range_mm', [0, 1, 2]],
    ];
    for (const [section, field, value] of cases) {
      const data = make(); set(data, section, field, value);
      assert.throws(() => parseRequirements(JSON.stringify(data)), new RegExp(field));
    }
  });
  test(`${format} accepts documented boundaries and range object forms`, () => {
    const data = make();
    for (const field of ['mass_kg', 'frequency_hz', 'cycle_mm']) set(data, 'payload', field, 0);
    set(data, 'constraints', 'ball_joint_max_deg', 0);
    set(data, 'constraints', 'rod_length_bounds_mm', { min: 200, max: 200 });
    set(data, 'constraints', 'horn_length_bounds_mm', { from: 50, to: 50 });
    set(data, 'constraints', 'servo_travel_bounds_deg', [0, 0]);
    const result = parseRequirements(JSON.stringify(data));
    assert.deepEqual(result.normalized.rod_length_bounds_mm, [200, 200]);
    assert.deepEqual(result.normalized.servo_travel_bounds_deg, [0, 0]);
  });
}

test('nested malformed sections and nonfinite numeric input are rejected', () => {
  assert.throws(() => parseRequirements('[]'), /Requirements/);
  const data = JSON.parse(sampleText); data.payload = [];
  assert.throws(() => parseRequirements(JSON.stringify(data)), /payload/);
  assert.throws(() => parseRequirements(sampleText.replace('2.5', '1e400')), /mass_kg/);
  assert.throws(() => new Optimizer({}, { ballJointLimitDeg: -1 }), /ball_joint_max_deg/);
});

test('invalid UI sweep controls fail before computation starts', async () => {
  const element = await loadUI(Optimizer);
  element('optXStep').value = '-1';
  await element('runOptimization').handlers.click();
  assert.match(element('optStatus').textContent, /step > 0/);
});
