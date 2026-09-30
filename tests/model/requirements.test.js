import test from 'node:test';
import assert from 'node:assert/strict';
import { parseRequirements } from '../../requirements.js';
import { Optimizer } from '../../optimizer.js';
import { sampleText, loadUI } from '../ui/helpers.js';

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

test('nested documents merge top-level constraint keys instead of dropping them', () => {
  const data = JSON.parse(sampleText);
  const { constraints } = data;
  delete data.constraints;
  Object.assign(data, constraints, { ball_joint_max_deg: 20, servo_torque_rating_nm: 3,
    workspace_payload_support: { rating: 'peak' } });
  const { normalized } = parseRequirements(JSON.stringify(data));
  assert.equal(normalized.ball_joint_max_deg, 20);
  assert.equal(normalized.servo_torque_rating_nm, 3);
  assert.equal(normalized.workspace_payload_support.rating, 'peak');
  assert.deepEqual(normalized.rod_length_bounds_mm, constraints.rod_length_bounds_mm);
  data.constraints = { ball_joint_max_deg: 30 };
  assert.throws(() => parseRequirements(JSON.stringify(data)), /ball_joint_max_deg.*both/);
});

test('optional sub-fields reject null instead of applying their default', () => {
  const trajectory = extra => ({ frequency_hz: 1, components: [{ axis: 'z', amplitude_mm: 5, ...extra }] });
  const cases = [
    ['payload', 'trajectory', { type: null, ...trajectory() }, /trajectory\.type/],
    ['payload', 'trajectory', trajectory({ phase_deg: null }), /phase_deg/],
    ['constraints', 'servo_torque_speed_curve', { speed_deg_s: [0, 100], torque_nm: [5, 1], quadrants: null }, /quadrants/],
    ['constraints', 'servo_torque_speed_curve',
      { speed_deg_s: [0, 100], torque_nm: [5, 1], quadrants: 'motoring-braking', braking_torque_nm: null }, /braking_torque_nm/],
    ['constraints', 'servo_actuator', { coulomb_nm: null }, /coulomb_nm/],
    ['constraints', 'stiffness_model', { servo_torsional_stiffness_nm_per_rad: 100, rods: 'rigid', use_as_objective: null }, /use_as_objective/],
    ['constraints', 'stiffness_model', { servo_torsional_stiffness_nm_per_rad: 100, rods: 'rigid', characteristic_length_mm: null }, /characteristic_length_mm/],
    ['constraints', 'stiffness_model', { servo_torsional_stiffness_nm_per_rad: 100, rods: 'rigid', test_wrenches: null }, /test_wrenches/],
    ['constraints', 'stiffness_model', { servo_torsional_stiffness_nm_per_rad: 100, rods: 'rigid', test_wrenches: [{ force_n: null }] }, /force_n/],
    ['constraints', 'stiffness_model', { servo_torsional_stiffness_nm_per_rad: 100, rods: 'rigid', test_wrenches: [{ moment_nm: null }] }, /moment_nm/],
    ['constraints', 'stiffness_model', { servo_torsional_stiffness_nm_per_rad: 100, rods: 'rigid', test_wrenches: [{ name: null }] }, /name/],
    ['constraints', 'stiffness_model', { servo_torsional_stiffness_nm_per_rad: 100, rods: null }, /rods/],
    ['constraints', 'stiffness_model', { servo_torsional_stiffness_nm_per_rad: 100, rod_material: null }, /rod_material/],
    ['constraints', 'workspace_payload_support', { rating: null }, /rating/],
    ['constraints', 'workspace_payload_support', { policy: null }, /policy/],
  ];
  for (const [section, field, value, pattern] of cases) {
    const data = JSON.parse(sampleText);
    if (field === 'trajectory') for (const key of ['cycle_mm', 'frequency_hz', 'cycle_axis']) delete data.payload[key];
    data[section][field] = value;
    assert.throws(() => parseRequirements(JSON.stringify(data)), pattern, `${field} ${JSON.stringify(value)}`);
  }
  // Omitting the same fields still applies the documented defaults.
  const data = JSON.parse(sampleText);
  for (const key of ['cycle_mm', 'frequency_hz', 'cycle_axis']) delete data.payload[key];
  data.payload.trajectory = trajectory();
  data.constraints.workspace_payload_support = {};
  data.constraints.stiffness_model = { servo_torsional_stiffness_nm_per_rad: 100, rods: 'rigid', test_wrenches: [{}] };
  const { normalized } = parseRequirements(JSON.stringify(data));
  assert.equal(normalized.trajectory.type, 'sinusoid');
  assert.equal(normalized.trajectory.components[0].phase_deg, 0);
});
