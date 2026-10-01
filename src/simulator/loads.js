import { vectorAdd, vectorMagnitude, vectorScale } from '../math.js';
import { dynamicsAtPose } from '../model/cycle.js';
import { GRAVITY, normalizeMassProperties, RIGID_BODY_FIELDS } from '../model/mass-properties.js';
import { actuatorTorque, normalizeServoRatings, SERVO_RATING_KEYS } from '../model/servo-ratings.js';

// A simulator load model is the moving mass and servo ratings, written with the
// requirement JSON keys and units (kg, mm, kg m², N, N m, deg/s). Other keys are dropped.
export const MASS_MODEL_KEYS = Object.freeze(['mass_kg', ...RIGID_BODY_FIELDS]);
export const LOAD_MODEL_KEYS = Object.freeze([...MASS_MODEL_KEYS, ...SERVO_RATING_KEYS]);

const pick = (source, keys) => Object.fromEntries(keys.filter(key => source?.[key] != null)
  .map(key => [key, structuredClone(source[key])]));

// Reads a load model through the same normalizers as the optimizer. Returns
// { input, massProperties, servoRatings }, where `input` is the kept user-unit
// fields, or null when nothing is supplied.
export function parseLoadModel(value, field = 'loadModel') {
  if (value == null) return null;
  if (typeof value !== 'object' || Array.isArray(value)) throw new TypeError(`${field} must be an object.`);
  const input = pick(value, LOAD_MODEL_KEYS);
  if (!Object.keys(input).length) return null;
  try {
    return { input, massProperties: normalizeMassProperties(input), servoRatings: normalizeServoRatings(input) };
  } catch (error) { throw new (error.constructor)(`${field}: ${error.message}`); }
}

// The load model of an optimizer run's `effective_settings`: the payload fields
// of its requirements and its effective servo rating input, which carries any
// UI rating overrides. Null when the run supplies neither.
export function loadModelFromSettings(settings) {
  const requirements = settings?.requirements ?? {};
  const model = { ...pick(requirements, MASS_MODEL_KEYS),
    ...pick(settings?.servoRatings ?? requirements, SERVO_RATING_KEYS) };
  return Object.keys(model).length ? model : null;
}

// Static force the rods must carry against gravity and the external force (N).
const staticForce = massProperties => vectorMagnitude(vectorAdd(vectorScale(GRAVITY, massProperties.massKg),
  massProperties.externalForceN));

// True when a load model's user-unit fields load the platform at all: a mass
// or a nonzero external force or moment.
export function hasLoad(input) {
  return Boolean(input) && (input.mass_kg > 0 || [input.external_force_n, input.external_moment_nm]
    .some(vector => Array.isArray(vector) && vector.some(value => value !== 0)));
}

// Rod forces and servo torques at a solved pose (evaluated with leg data) for
// one motion state in SI units, through the cycle model's dynamicsAtPose. Rod
// force is positive in compression (the rod pushes the platform). Servo torque
// is the signed output-shaft torque, including any reduced actuator model, and
// utilisation is its magnitude over the servo's peak torque rating (null unrated).
export function poseLoads(layout, solved, state, model, motion = 'static') {
  const result = dynamicsAtPose(layout, solved, state, model.massProperties);
  if (!result.valid) return { valid: false, motion, reason: result.reason };
  const ratings = model.servoRatings.perServo;
  const servoTorqueNm = result.signedTorque.map((torque, i) => actuatorTorque(ratings[i].actuator, torque,
    result.signedSpeed[i], result.servoAcceleration[i]));
  const ratedTorqueNm = ratings.map(rating => rating.torqueNm);
  return { valid: true, motion, reason: null, rodForceN: result.rodForces, servoTorqueNm,
    servoSpeedRadPerSec: result.signedSpeed, ratedTorqueNm,
    utilization: servoTorqueNm.map((torque, i) => ratedTorqueNm[i] == null ? null : Math.abs(torque) / ratedTorqueNm[i]),
    staticForceN: staticForce(model.massProperties) };
}
