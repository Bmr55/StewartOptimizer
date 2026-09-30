import { degToRad } from '../math.js';

export const SERVO_CAPACITY_MODEL = 'servo-capacity-v2';
export const SERVO_RATING_KEYS = Object.freeze(['servo_torque_rating_nm', 'servo_speed_rating_deg_s',
  'servo_continuous_torque_rating_nm', 'servo_torque_speed_curve', 'servo_duration_ratings', 'servo_actuator',
  'per_servo_ratings', 'servo_rating_policy']);
export const PER_SERVO_RATING_KEYS = Object.freeze(['torque_nm', 'speed_deg_s', 'continuous_torque_nm',
  'torque_speed_curve', 'duration_ratings', 'actuator']);

const positiveOrNull = (value, field) => {
  if (value == null) return null;
  if (!Number.isFinite(value) || value <= 0) throw new RangeError(`${field} must be a finite positive number.`);
  return value;
};

const requiredPositive = (value, field) => {
  if (value == null) throw new RangeError(`${field} is required.`);
  return positiveOrNull(value, field);
};

const plainObject = (value, field) => {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new TypeError(`${field} must be an object.`);
  return value;
};

// Output-shaft torque capacity versus speed magnitude, linear between supplied
// points and unavailable outside them (no extrapolation). `symmetric` applies one
// curve to both directions and to braking (torque opposing motion); `motoring-braking`
// uses `torque_nm` when torque and speed share a sign and `braking_torque_nm`
// otherwise; without braking data, braking operating points are unmodeled.
function normalizeCurve(input, field) {
  if (input == null) return null;
  plainObject(input, field);
  const speeds = input.speed_deg_s, torques = input.torque_nm;
  if (!Array.isArray(speeds) || speeds.length < 2) throw new RangeError(`${field}.speed_deg_s must list at least two speeds.`);
  if (!Array.isArray(torques) || torques.length !== speeds.length) {
    throw new RangeError(`${field}.torque_nm must have one torque per speed.`);
  }
  speeds.forEach((speed, i) => {
    if (!Number.isFinite(speed) || speed < 0) throw new RangeError(`${field}.speed_deg_s must be finite and >= 0.`);
    if (i && speed <= speeds[i - 1]) throw new RangeError(`${field}.speed_deg_s must be strictly increasing.`);
  });
  const checkTorques = (values, name) => values.forEach(value => {
    if (!Number.isFinite(value) || value < 0) throw new RangeError(`${field}.${name} must be finite and >= 0.`);
  });
  checkTorques(torques, 'torque_nm');
  const quadrants = input.quadrants ?? 'symmetric';
  if (!['symmetric', 'motoring-braking'].includes(quadrants)) {
    throw new RangeError(`${field}.quadrants must be symmetric or motoring-braking.`);
  }
  let braking = null;
  if (input.braking_torque_nm != null) {
    if (quadrants !== 'motoring-braking') throw new RangeError(`${field}.braking_torque_nm requires quadrants motoring-braking.`);
    if (!Array.isArray(input.braking_torque_nm) || input.braking_torque_nm.length !== speeds.length) {
      throw new RangeError(`${field}.braking_torque_nm must have one torque per speed.`);
    }
    checkTorques(input.braking_torque_nm, 'braking_torque_nm');
    braking = input.braking_torque_nm.slice();
  }
  if (Math.max(...torques) <= 0) throw new RangeError(`${field}.torque_nm must include a positive capacity.`);
  return { quadrants, speedRadPerSec: speeds.map(degToRad), torqueNm: torques.slice(), brakingTorqueNm: braking,
    peakTorqueNm: Math.max(...torques, ...(braking ?? [])) };
}

function normalizeDurationRatings(input, field) {
  if (input == null) return null;
  if (!Array.isArray(input) || !input.length) throw new RangeError(`${field} must be a nonempty array.`);
  return input.map((entry, i) => {
    plainObject(entry, `${field}[${i}]`);
    return { torqueNm: requiredPositive(entry.torque_nm, `${field}[${i}].torque_nm`),
      durationS: requiredPositive(entry.duration_s, `${field}[${i}].duration_s`) };
  });
}

// Reduced actuator at the output shaft: tau = tau_load + J alphaDDot + b alphaDot + c sign(alphaDot).
// J is the rotor/gear inertia reflected to the output (J_motor N^2); the load model already
// covers the platform, so only actuator-side inertia belongs here. Coulomb friction is zero at rest.
function normalizeActuator(input, field) {
  if (input == null) return null;
  plainObject(input, field);
  const value = key => {
    const entry = input[key] ?? 0;
    if (!Number.isFinite(entry) || entry < 0) throw new RangeError(`${field}.${key} must be finite and >= 0.`);
    return entry;
  };
  return { outputInertiaKgM2: value('output_inertia_kg_m2'), viscousNmSPerRad: value('viscous_nm_s_per_rad'),
    coulombNm: value('coulomb_nm') };
}

export function normalizeServoRatings(input = {}) {
  const policy = input.servo_rating_policy ?? 'enforced';
  if (!['enforced', 'advisory'].includes(policy)) {
    throw new RangeError('servo_rating_policy must be enforced or advisory.');
  }
  const sharedTorque = positiveOrNull(input.servo_torque_rating_nm, 'servo_torque_rating_nm');
  const sharedSpeedDeg = positiveOrNull(input.servo_speed_rating_deg_s, 'servo_speed_rating_deg_s');
  const sharedContinuous = positiveOrNull(input.servo_continuous_torque_rating_nm, 'servo_continuous_torque_rating_nm');
  const sharedCurve = normalizeCurve(input.servo_torque_speed_curve, 'servo_torque_speed_curve');
  const sharedDurations = normalizeDurationRatings(input.servo_duration_ratings, 'servo_duration_ratings');
  const sharedActuator = normalizeActuator(input.servo_actuator, 'servo_actuator');
  const overrides = input.per_servo_ratings;
  if (overrides != null && (!Array.isArray(overrides) || overrides.length !== 6)) {
    throw new RangeError('per_servo_ratings must contain six entries.');
  }
  const source = (override, shared) => override == null ? shared == null ? 'unrated' : 'shared' : 'override';
  const perServo = Array.from({ length: 6 }, (_, index) => {
    const entry = overrides?.[index];
    if (entry != null && (typeof entry !== 'object' || Array.isArray(entry))) {
      throw new TypeError(`per_servo_ratings[${index}] must be an object or null.`);
    }
    const name = key => `per_servo_ratings[${index}].${key}`;
    const torqueOverride = positiveOrNull(entry?.torque_nm, name('torque_nm'));
    const speedOverrideDeg = positiveOrNull(entry?.speed_deg_s, name('speed_deg_s'));
    const continuousOverride = positiveOrNull(entry?.continuous_torque_nm, name('continuous_torque_nm'));
    const curveOverride = normalizeCurve(entry?.torque_speed_curve, name('torque_speed_curve'));
    const durationOverride = normalizeDurationRatings(entry?.duration_ratings, name('duration_ratings'));
    const actuatorOverride = normalizeActuator(entry?.actuator, name('actuator'));
    const effectiveSpeedDeg = speedOverrideDeg ?? sharedSpeedDeg;
    return {
      torqueNm: torqueOverride ?? sharedTorque,
      speedRadPerSec: effectiveSpeedDeg == null ? null : degToRad(effectiveSpeedDeg),
      continuousTorqueNm: continuousOverride ?? sharedContinuous,
      curve: curveOverride ?? sharedCurve,
      durationRatings: durationOverride ?? sharedDurations,
      actuator: actuatorOverride ?? sharedActuator,
      source: {
        torque: source(torqueOverride, sharedTorque),
        speed: source(speedOverrideDeg, sharedSpeedDeg),
        continuous: source(continuousOverride, sharedContinuous),
        curve: source(curveOverride, sharedCurve),
        duration: source(durationOverride, sharedDurations),
        actuator: actuatorOverride ? 'override' : sharedActuator ? 'shared' : 'unmodeled',
      },
    };
  });
  return {
    policy,
    modelVersion: SERVO_CAPACITY_MODEL,
    shared: { torqueNm: sharedTorque, speedRadPerSec: sharedSpeedDeg == null ? null : degToRad(sharedSpeedDeg),
      continuousTorqueNm: sharedContinuous, curve: sharedCurve, durationRatings: sharedDurations, actuator: sharedActuator },
    perServo,
    hasRatings: perServo.some(servo => servo.torqueNm != null || servo.speedRadPerSec != null
      || servo.continuousTorqueNm != null || servo.curve != null || servo.durationRatings != null),
    hasActuatorModel: perServo.some(servo => servo.actuator != null),
  };
}

export function actuatorTorque(actuator, loadTorque, speed, acceleration) {
  if (!actuator) return loadTorque;
  return loadTorque + actuator.outputInertiaKgM2 * acceleration + actuator.viscousNmSPerRad * speed
    + actuator.coulombNm * Math.sign(speed);
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

function interpolate(speeds, torques, speed) {
  if (speed < speeds[0] || speed > speeds.at(-1)) return null;
  let i = 1;
  while (speeds[i] < speed) i++;
  const t = (speed - speeds[i - 1]) / (speeds[i] - speeds[i - 1]);
  return torques[i - 1] + t * (torques[i] - torques[i - 1]);
}

// Capacity at one signed operating point, or a reason it is not modeled.
export function envelopeCapacity(curve, torque, speed) {
  const magnitude = Math.abs(speed);
  const braking = torque * speed < 0;
  const quadrant = curve.quadrants === 'symmetric' ? 'symmetric' : braking ? 'braking' : 'motoring';
  if (magnitude < curve.speedRadPerSec[0] || magnitude > curve.speedRadPerSec.at(-1)) {
    return { capacity: null, quadrant, reason: 'outOfDomain' };
  }
  if (quadrant === 'braking' && !curve.brakingTorqueNm) return { capacity: null, quadrant, reason: 'unmodeledQuadrant' };
  const values = quadrant === 'braking' ? curve.brakingTorqueNm : curve.torqueNm;
  return { capacity: interpolate(curve.speedRadPerSec, values, magnitude), quadrant, reason: null };
}

// Evaluates every sampled simultaneous (torque, speed) point; headroom fractions
// use the curve's peak torque because capacity can reach zero at the speed limit.
function compareEnvelope(history, servo, curve) {
  if (!curve) return { status: 'unrated', worstHeadroomFraction: null, limiting: null };
  if (!history) return { status: 'unavailable', worstHeadroomFraction: null, limiting: null };
  let limiting = null;
  const torques = history.signedTorqueNm[servo], speeds = history.signedSpeedRadPerSec[servo];
  for (let k = 0; k < history.times.length; k++) {
    const point = envelopeCapacity(curve, torques[k], speeds[k]);
    const base = { time: history.times[k], torqueNm: torques[k], speedRadPerSec: speeds[k], quadrant: point.quadrant };
    if (point.capacity == null) {
      // Do not extrapolate: an operating point outside the supplied data cannot pass.
      if (!limiting || limiting.reason == null) limiting = { ...base, capacityNm: null, headroomFraction: null, reason: point.reason };
      continue;
    }
    const headroomFraction = (point.capacity - Math.abs(torques[k])) / curve.peakTorqueNm;
    if (!limiting || (limiting.reason == null && headroomFraction < limiting.headroomFraction)) {
      limiting = { ...base, capacityNm: point.capacity, headroomFraction, reason: null };
    }
  }
  if (limiting.reason) return { status: limiting.reason === 'outOfDomain' ? 'outOfDomain' : 'unavailable',
    worstHeadroomFraction: null, limiting };
  const fraction = limiting.headroomFraction;
  return { status: fraction > 0 ? 'below' : fraction < 0 ? 'above' : 'at', worstHeadroomFraction: fraction, limiting };
}

// Integral of torque^2 from 0 to t with torque^2 interpolated linearly between samples,
// so a whole-period window equals the trapezoidal cycle RMS.
function squaredIntegral(times, values, period, t) {
  const n = times.length;
  const piece = (t0, v0, t1, v1, upTo) => {
    const s = (upTo - t0) / (t1 - t0), end = v0 * v0 + s * (v1 * v1 - v0 * v0);
    return (upTo - t0) * (v0 * v0 + end) / 2;
  };
  // Every schedule starts at t = 0, so the samples cover [0, period) with a wrap interval.
  const periods = Math.floor(t / period);
  const local = t - periods * period;
  let partial = 0, full = 0;
  for (let k = 0; k < n; k++) {
    const t0 = times[k], v0 = values[k];
    const t1 = k === n - 1 ? times[0] + period : times[k + 1];
    const v1 = values[(k + 1) % n];
    const whole = piece(t0, v0, t1, v1, t1);
    full += whole;
    if (local >= t1) partial += whole;
    else if (local > t0) partial += piece(t0, v0, t1, v1, local);
  }
  return periods * full + partial;
}

export function windowRms(times, values, period, duration) {
  if (times.length === 1 || !(period > 0)) return Math.abs(values[0]);
  let worst = 0;
  for (const start of times) {
    const energy = squaredIntegral(times, values, period, start + duration) - squaredIntegral(times, values, period, start);
    worst = Math.max(worst, Math.sqrt(Math.max(0, energy) / duration));
  }
  return worst;
}

const STATUS_ORDER = ['unavailable', 'outOfDomain', 'above', 'at', 'below'];
function combine(statuses) {
  const rated = statuses.filter(status => status !== 'unrated');
  if (!rated.length) return 'unrated';
  return STATUS_ORDER.find(status => rated.includes(status));
}
const minimum = values => {
  const finite = values.filter(Number.isFinite);
  return finite.length ? Math.min(...finite) : null;
};

export function evaluateServoCapacity(cycle, ratings = normalizeServoRatings()) {
  const valid = cycle?.valid === true;
  const history = valid ? cycle.history ?? null : null;
  const peakTorque = valid ? cycle.actuator?.perServoPeakTorqueNm ?? cycle.perServoTorqueNm : null;
  const rmsTorque = valid ? cycle.actuator?.perServoRmsTorqueNm : null;
  const perServo = ratings.perServo.map((rating, index) => {
    const duration = (rating.durationRatings ?? []).map(entry => ({ durationS: entry.durationS,
      ...compareDemand(history ? windowRms(history.times, history.signedTorqueNm[index], history.periodS, entry.durationS)
        : null, entry.torqueNm) }));
    return {
      servo: index + 1,
      torque: compareDemand(peakTorque?.[index], rating.torqueNm),
      speed: compareDemand(valid ? cycle.perServoSpeedRadPerSec?.[index] : null, rating.speedRadPerSec),
      envelope: compareEnvelope(history, index, rating.curve),
      continuous: compareDemand(rmsTorque?.[index], rating.continuousTorqueNm),
      duration,
      source: rating.source,
    };
  });
  const group = items => {
    const status = combine(items.map(item => item.status));
    return { status, worstHeadroomFraction: ['unavailable', 'outOfDomain', 'unrated'].includes(status) ? null
      : minimum(items.filter(item => item.status !== 'unrated').map(item => item.headroomFraction ?? item.worstHeadroomFraction)) };
  };
  const peak = group(perServo.flatMap(servo => [servo.torque, servo.speed, servo.envelope]));
  const continuous = group(perServo.map(servo => servo.continuous));
  const durationGroup = group(perServo.flatMap(servo => servo.duration));
  const status = !ratings.hasRatings ? 'unrated' : combine([peak.status, continuous.status, durationGroup.status]);
  const compliant = ratings.hasRatings ? status === 'below' || status === 'at' : null;
  return {
    policy: ratings.policy,
    modelVersion: ratings.modelVersion ?? SERVO_CAPACITY_MODEL,
    hasRatings: ratings.hasRatings,
    status,
    compliant,
    enforcedSatisfied: ratings.policy === 'advisory' || !ratings.hasRatings || compliant === true,
    worstHeadroomFraction: ['unavailable', 'outOfDomain'].includes(status) ? null
      : minimum([peak.worstHeadroomFraction, continuous.worstHeadroomFraction, durationGroup.worstHeadroomFraction]),
    peak, continuous, duration: durationGroup,
    demandReference: cycle?.actuator?.model === 'reduced' ? 'actuator output torque (load plus reduced actuator model)'
      : 'load torque at the output shaft (actuator dynamics unmodeled)',
    rmsNote: 'RMS torque is a time-weighted screening metric over the sampled cycle, not a calibrated temperature prediction.',
    shared: ratings.shared,
    perServo,
  };
}
