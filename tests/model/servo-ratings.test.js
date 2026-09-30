import test from 'node:test';
import assert from 'node:assert/strict';
import { parseRequirements } from '../../src/model/requirements.js';
import { normalizeServoRatings, evaluateServoCapacity } from '../../src/model/servo-ratings.js';
import { failureCategories, isPassing, compareCandidates, layoutToJSON } from '../../src/io/results.js';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { sampleText } from '../ui/helpers.js';

const cycle = (torque, speed) => ({ valid: true,
  perServoTorqueNm: torque, perServoSpeedRadPerSec: speed });

test('shared and per-servo ratings validate in nested and flat requirements', () => {
  for (const nested of [true, false]) {
    const data = nested ? JSON.parse(sampleText) : parseRequirements(sampleText).normalized;
    const settings = nested ? data.constraints : data;
    settings.servo_torque_rating_nm = 10;
    settings.servo_speed_rating_deg_s = 180;
    settings.per_servo_ratings = [{ torque_nm: 5, speed_deg_s: 90 }, null, null, null, null, null];
    settings.servo_rating_policy = 'advisory';
    const parsed = parseRequirements(JSON.stringify(data)).normalized;
    const ratings = normalizeServoRatings(parsed);
    assert.equal(ratings.policy, 'advisory');
    assert.equal(ratings.perServo[0].torqueNm, 5);
    assert.equal(ratings.perServo[0].speedRadPerSec, Math.PI / 2);
    assert.equal(ratings.perServo[1].torqueNm, 10);
    assert.equal(ratings.perServo[1].speedRadPerSec, Math.PI);
    assert.equal(ratings.perServo[0].source.torque, 'override');
    assert.equal(ratings.perServo[1].source.speed, 'shared');
    for (const [key, invalid] of [
      ['servo_torque_rating_nm', 0], ['servo_speed_rating_deg_s', '12'],
      ['servo_torque_rating_nm', null], ['servo_speed_rating_deg_s', null],
      ['servo_rating_policy', 'ignore'], ['per_servo_ratings', [{}]],
      ['per_servo_ratings', [{ speed_deg_s: -1 }, null, null, null, null, null]],
    ]) {
      const bad = structuredClone(data);
      (nested ? bad.constraints : bad)[key] = invalid;
      assert.throws(() => parseRequirements(JSON.stringify(bad)), new RegExp(key));
    }
  }
});

test('per-servo demand reports below, at and above rating with worst margin', () => {
  const ratings = normalizeServoRatings({ servo_torque_rating_nm: 10,
    servo_speed_rating_deg_s: 180,
    per_servo_ratings: [{ torque_nm: 5 }, null, null, null, null, null] });
  const values = evaluateServoCapacity(cycle([4, 10, 11, 0, 0, 0],
    [Math.PI / 2, Math.PI, 0, 0, 0, 0]), ratings);
  assert.equal(values.perServo[0].torque.status, 'below');
  assert.equal(values.perServo[0].torque.headroom, 1);
  assert.equal(values.perServo[0].torque.headroomFraction, 0.2);
  assert.equal(values.perServo[1].torque.status, 'at');
  assert.equal(values.perServo[1].speed.status, 'at');
  assert.equal(values.perServo[2].torque.status, 'above');
  assert.equal(values.worstHeadroomFraction, -0.1);
  assert.equal(values.status, 'above');
  assert.equal(values.enforcedSatisfied, false);
});

test('enforced capacity affects feasibility and ranking; advisory remains visible', () => {
  const base = { layout: { id: 1 }, feasibility: { homePoseSatisfied: true,
    sampledWorkspaceSatisfied: true, cycleSatisfied: true }, torque: 1,
  speedDemand: 1, coverage: 100, conditioningQuality: 1 };
  const ratings = normalizeServoRatings({ servo_torque_rating_nm: 5 });
  const over = evaluateServoCapacity(cycle([6, 1, 1, 1, 1, 1], [0, 0, 0, 0, 0, 0]), ratings);
  const failing = { ...base, servoCapacity: over, feasibility: { ...base.feasibility,
    servoCapacitySatisfied: false, servoCapacityEnforced: true } };
  const passing = { ...base, layout: { id: 2 }, torque: 2 };
  assert.deepEqual(failureCategories(failing), ['servo_capacity']);
  assert.equal(isPassing(failing), false);
  assert.equal(compareCandidates(failing, passing), 1);
  const advisory = { ...failing, servoCapacity: evaluateServoCapacity(
    cycle([6, 1, 1, 1, 1, 1], [0, 0, 0, 0, 0, 0]),
    normalizeServoRatings({ servo_torque_rating_nm: 5, servo_rating_policy: 'advisory' })),
    feasibility: { ...failing.feasibility, servoCapacityEnforced: false } };
  assert.equal(advisory.servoCapacity.status, 'above');
  assert.equal(advisory.servoCapacity.policy, 'advisory');
  assert.equal(isPassing(advisory), true);
  assert.deepEqual(failureCategories(advisory), []);
  assert.equal(evaluateServoCapacity(cycle([1, 2, 3, 4, 5, 6], [0, 0, 0, 0, 0, 0])).status, 'unrated');
});

test('unavailable demand cannot satisfy an enforced rating or imply zero headroom', () => {
  const ratings = normalizeServoRatings({ servo_torque_rating_nm: 5 });
  for (const input of [null, { valid: false }, cycle([NaN, 1, 1, 1, 1, 1], [0, 0, 0, 0, 0, 0])]) {
    const capacity = evaluateServoCapacity(input, ratings);
    assert.equal(capacity.status, 'unavailable');
    assert.equal(capacity.compliant, false);
    assert.equal(capacity.enforcedSatisfied, false);
    assert.equal(capacity.perServo[0].torque.demand, null);
    assert.equal(capacity.worstHeadroomFraction, null);
  }
});

test('effective settings replay UI rating overrides with the same sources and policy', () => {
  const optimizer = new Optimizer({}, { servoRatings: {
    servo_torque_rating_nm: 10,
    servo_speed_rating_deg_s: 180,
    per_servo_ratings: [{ torque_nm: 4 }, null, null, null, null, null],
    servo_rating_policy: 'advisory',
  } });
  const settings = optimizer.effectiveSettings();
  assert.equal(settings.servoRatings.per_servo_ratings[0].torque_nm, 4);
  assert.equal(settings.effectiveServoRatings.perServo[0].source.torque, 'override');
  const saved = { ...layoutToJSON(optimizer.createRandomLayout()), run: { effective_settings: settings } };
  const replay = Optimizer.fromReplay(saved);
  assert.deepEqual(replay.servoRatings, optimizer.servoRatings);
  assert.equal(replay.servoRatings.policy, 'advisory');
});
