import { degToRad } from '../math.js';

const positiveOrNull = (value, field) => {
  if (value == null) return null;
  if (!Number.isFinite(value) || value <= 0) throw new RangeError(`${field} must be a finite positive number.`);
  return value;
};

export function normalizeServoRatings(input = {}) {
  const policy = input.servo_rating_policy ?? 'enforced';
  if (!['enforced', 'advisory'].includes(policy)) {
    throw new RangeError('servo_rating_policy must be enforced or advisory.');
  }
  const sharedTorque = positiveOrNull(input.servo_torque_rating_nm, 'servo_torque_rating_nm');
  const sharedSpeedDeg = positiveOrNull(input.servo_speed_rating_deg_s, 'servo_speed_rating_deg_s');
  const overrides = input.per_servo_ratings;
  if (overrides != null && (!Array.isArray(overrides) || overrides.length !== 6)) {
    throw new RangeError('per_servo_ratings must contain six entries.');
  }
  const perServo = Array.from({ length: 6 }, (_, index) => {
    const entry = overrides?.[index];
    if (entry != null && (typeof entry !== 'object' || Array.isArray(entry))) {
      throw new TypeError(`per_servo_ratings[${index}] must be an object or null.`);
    }
    const torqueOverride = positiveOrNull(entry?.torque_nm, `per_servo_ratings[${index}].torque_nm`);
    const speedOverrideDeg = positiveOrNull(entry?.speed_deg_s, `per_servo_ratings[${index}].speed_deg_s`);
    const effectiveSpeedDeg = speedOverrideDeg ?? sharedSpeedDeg;
    return {
      torqueNm: torqueOverride ?? sharedTorque,
      speedRadPerSec: effectiveSpeedDeg == null ? null : degToRad(effectiveSpeedDeg),
      source: {
        torque: torqueOverride == null ? sharedTorque == null ? 'unrated' : 'shared' : 'override',
        speed: speedOverrideDeg == null ? sharedSpeedDeg == null ? 'unrated' : 'shared' : 'override',
      },
    };
  });
  return {
    policy,
    shared: { torqueNm: sharedTorque, speedRadPerSec: sharedSpeedDeg == null ? null : degToRad(sharedSpeedDeg) },
    perServo,
    hasRatings: perServo.some(servo => servo.torqueNm != null || servo.speedRadPerSec != null),
  };
}

function compareDemand(demand, rating) {
  if (rating == null) return { demand: Number.isFinite(demand) ? demand : null,
    rating: null, headroom: null, headroomFraction: null, status: 'unrated' };
  if (!Number.isFinite(demand) || demand < 0) return {
    demand: null, rating, headroom: null, headroomFraction: null, status: 'unavailable',
  };
  const headroom = rating - demand;
  return {
    demand, rating, headroom, headroomFraction: headroom / rating,
    status: headroom > 0 ? 'below' : headroom < 0 ? 'above' : 'at',
  };
}

export function evaluateServoCapacity(cycle, ratings = normalizeServoRatings()) {
  const perServo = ratings.perServo.map((rating, index) => ({
    servo: index + 1,
    torque: compareDemand(cycle?.valid ? cycle.perServoTorqueNm?.[index] : null, rating.torqueNm),
    speed: compareDemand(cycle?.valid ? cycle.perServoSpeedRadPerSec?.[index] : null, rating.speedRadPerSec),
    source: rating.source,
  }));
  const measured = perServo.flatMap(servo => [servo.torque, servo.speed]).filter(item => item.rating != null);
  const statuses = measured.map(item => item.status);
  const status = !ratings.hasRatings ? 'unrated'
    : statuses.includes('unavailable') ? 'unavailable'
      : statuses.includes('above') ? 'above'
        : statuses.includes('at') ? 'at' : 'below';
  const availableMargins = measured.map(item => item.headroomFraction).filter(Number.isFinite);
  const compliant = ratings.hasRatings ? status === 'below' || status === 'at' : null;
  return {
    policy: ratings.policy,
    hasRatings: ratings.hasRatings,
    status,
    compliant,
    enforcedSatisfied: ratings.policy === 'advisory' || !ratings.hasRatings || compliant === true,
    worstHeadroomFraction: status !== 'unavailable' && availableMargins.length
      ? Math.min(...availableMargins) : null,
    shared: ratings.shared,
    perServo,
  };
}
