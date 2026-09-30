import { normalizeServoRatings, PER_SERVO_RATING_KEYS, SERVO_RATING_KEYS } from './servo-ratings.js';
import { normalizeTrajectory } from './trajectory.js';
import { normalizeMassProperties, RIGID_BODY_FIELDS } from './mass-properties.js';
import { normalizeStiffnessModel } from './compliance.js';
import { normalizePayloadSupport } from '../workspace/payload-support.js';
import { DEFAULT_BALL_JOINT_LIMIT_DEG } from '../contracts.js';

const DEFAULTS = {
  ball_joint_max_deg: DEFAULT_BALL_JOINT_LIMIT_DEG,
  servo_travel_bounds_deg: [-120, 120],
  rod_length_bounds_mm: [160, 420],
  horn_length_bounds_mm: [30, 110],
  home_height_bounds_mm: [50, 450],
};
const PAYLOAD = ['mass_kg', 'cycle_mm', 'frequency_hz', 'cycle_axis'];
const LEGACY_CYCLE = ['cycle_mm', 'frequency_hz', 'cycle_axis'];
const PAYLOAD_OPTIONAL = ['trajectory', ...RIGID_BODY_FIELDS];
const TRANSLATIONS = ['x_range_mm', 'y_range_mm', 'z_range_mm'];
const ROTATIONS = ['rx_range_deg', 'ry_range_deg', 'rz_range_deg'];
const CONSTRAINTS = [...Object.keys(DEFAULTS), 'servo_max_deg', ...SERVO_RATING_KEYS, 'stiffness_model',
  'workspace_payload_support'];

function object(value, name) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) {
    throw new Error(`${name} must be a JSON object.`);
  }
  return value;
}

function range(value, name) {
  let candidate = value;
  if (!Array.isArray(candidate) && candidate && typeof candidate === 'object') {
    if ('min' in candidate && 'max' in candidate) candidate = [candidate.min, candidate.max];
    else if ('from' in candidate && 'to' in candidate) candidate = [candidate.from, candidate.to];
  }
  if (!Array.isArray(candidate) || candidate.length !== 2) {
    throw new Error(`${name} must define exactly two min/max values.`);
  }
  const [min, max] = candidate;
  if (![min, max].every(Number.isFinite) || max < min) {
    throw new Error(`${name} must contain finite numbers with max >= min.`);
  }
  return [min, max];
}

function nonnegative(value, name) {
  if (!Number.isFinite(value) || value < 0) throw new Error(`${name} must be a finite number >= 0.`);
}

export function validatePhysicalRequirements(data) {
  nonnegative(data.mass_kg, 'mass_kg');
  if (data.trajectory == null) {
    for (const key of ['cycle_mm', 'frequency_hz']) nonnegative(data[key], key);
    if (!['x', 'y', 'z'].includes(data.cycle_axis)) throw new Error('cycle_axis must be x, y or z.');
  } else {
    normalizeTrajectory(data.trajectory);
  }
  normalizeMassProperties(data);
  nonnegative(data.ball_joint_max_deg, 'ball_joint_max_deg');
  if (data.ball_joint_max_deg > 180) throw new Error('ball_joint_max_deg must be between 0 and 180.');
  for (const key of ['rod_length_bounds_mm', 'horn_length_bounds_mm', 'home_height_bounds_mm']) {
    const bounds = range(data[key], key);
    if (bounds[0] <= 0) throw new Error(`${key} must contain positive lengths.`);
  }
  range(data.servo_travel_bounds_deg, 'servo_travel_bounds_deg');
}

// A grouped document may still carry constraint keys at the top level; they are
// merged rather than silently dropped, and a key present in both places is an error.
function nestedConstraints(data) {
  const grouped = 'constraints' in data ? object(data.constraints, 'constraints') : {};
  const constraints = { ...grouped };
  for (const key of CONSTRAINTS) {
    if (!(key in data)) continue;
    if (key in grouped) throw new Error(`Requirements ${key} appears both at the top level and in constraints.`);
    constraints[key] = data[key];
  }
  return constraints;
}

export function parseRequirements(text) {
  let data;
  try { data = JSON.parse(text); }
  catch { throw new Error('Requirements JSON is invalid.'); }
  object(data, 'Requirements');
  const nested = ['payload', 'workspace', 'rotations', 'constraints'].some(key => key in data);
  const source = {};
  for (const [section, fields] of [['payload', PAYLOAD], ['workspace', TRANSLATIONS], ['rotations', ROTATIONS]]) {
    const part = nested ? object(data[section], section) : data;
    const trajectory = section === 'payload' && 'trajectory' in part;
    for (const key of fields) {
      // A supplied trajectory replaces the legacy single-axis cycle fields; both would be ambiguous.
      if (trajectory && LEGACY_CYCLE.includes(key)) {
        if (key in part) throw new Error(`Requirements ${key} cannot be combined with trajectory.`);
        continue;
      }
      if (!(key in part)) throw new Error(`Requirements missing ${nested ? section + '.' : ''}${key}.`);
      source[key] = part[key];
    }
    if (section !== 'payload') continue;
    for (const key of PAYLOAD_OPTIONAL) {
      if (!(key in part)) continue;
      if (part[key] === null) throw new Error(`${key} must not be null.`);
      source[key] = part[key];
    }
  }
  const constraints = nested ? nestedConstraints(data) : data;
  for (const key of [...SERVO_RATING_KEYS, 'stiffness_model', 'workspace_payload_support']) {
    if (key in constraints && constraints[key] === null) throw new Error(`${key} must not be null.`);
  }
  if (Array.isArray(constraints.per_servo_ratings)) {
    constraints.per_servo_ratings.forEach((entry, index) => {
      for (const key of PER_SERVO_RATING_KEYS) {
        if (entry && typeof entry === 'object' && key in entry && entry[key] === null) {
          throw new Error(`per_servo_ratings[${index}].${key} must not be null.`);
        }
      }
    });
  }
  for (const key of CONSTRAINTS) {
    if (key in constraints) source[key] = constraints[key];
  }
  const normalized = { ...source };
  if (source.trajectory == null) {
    if (typeof source.cycle_axis !== 'string') throw new Error('cycle_axis must be a string.');
    normalized.cycle_axis = source.cycle_axis.toLowerCase();
  } else {
    normalized.trajectory = normalizeTrajectory(source.trajectory);
  }
  if ('servo_max_deg' in source) nonnegative(source.servo_max_deg, 'servo_max_deg');
  if (!('servo_travel_bounds_deg' in source) && 'servo_max_deg' in source) {
    normalized.servo_travel_bounds_deg = [-source.servo_max_deg, source.servo_max_deg];
  }
  for (const [key, value] of Object.entries(DEFAULTS)) {
    if (!(key in normalized)) normalized[key] = Array.isArray(value) ? value.slice() : value;
  }
  for (const key of [...TRANSLATIONS, ...ROTATIONS, 'rod_length_bounds_mm', 'horn_length_bounds_mm',
    'home_height_bounds_mm', 'servo_travel_bounds_deg']) {
    normalized[key] = range(normalized[key], key);
  }
  validatePhysicalRequirements(normalized);
  normalizeServoRatings(normalized);
  const stiffnessModel = normalizeStiffnessModel(normalized.stiffness_model);
  // Record the resolved objective choice so an export replays with the same one.
  if (stiffnessModel) normalized.stiffness_model = { ...normalized.stiffness_model, use_as_objective: stiffnessModel.useAsObjective };
  normalizePayloadSupport(normalized.workspace_payload_support);
  normalized.servo_rating_policy ??= 'enforced';
  const workspace = {};
  [...TRANSLATIONS, ...ROTATIONS].forEach((key, index) => {
    const [min, max] = normalized[key];
    workspace[['x', 'y', 'z', 'rx', 'ry', 'rz'][index]] = { min, max, step: (max - min) / 2 || 5 };
  });
  return { normalized, workspace };
}
