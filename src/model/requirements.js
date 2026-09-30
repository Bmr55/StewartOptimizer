const DEFAULTS = {
  ball_joint_max_deg: 45,
  servo_travel_bounds_deg: [-120, 120],
  rod_length_bounds_mm: [160, 420],
  horn_length_bounds_mm: [30, 110],
};
const PAYLOAD = ['mass_kg', 'cycle_mm', 'frequency_hz', 'cycle_axis'];
const TRANSLATIONS = ['x_range_mm', 'y_range_mm', 'z_range_mm'];
const ROTATIONS = ['rx_range_deg', 'ry_range_deg', 'rz_range_deg'];

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
  for (const key of ['mass_kg', 'cycle_mm', 'frequency_hz']) nonnegative(data[key], key);
  if (!['x', 'y', 'z'].includes(data.cycle_axis)) throw new Error('cycle_axis must be x, y or z.');
  nonnegative(data.ball_joint_max_deg, 'ball_joint_max_deg');
  if (data.ball_joint_max_deg > 180) throw new Error('ball_joint_max_deg must be between 0 and 180.');
  for (const key of ['rod_length_bounds_mm', 'horn_length_bounds_mm']) {
    const bounds = range(data[key], key);
    if (bounds[0] <= 0) throw new Error(`${key} must contain positive lengths.`);
  }
  range(data.servo_travel_bounds_deg, 'servo_travel_bounds_deg');
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
    for (const key of fields) {
      if (!(key in part)) throw new Error(`Requirements missing ${nested ? section + '.' : ''}${key}.`);
      source[key] = part[key];
    }
  }
  const constraints = nested ? ('constraints' in data ? object(data.constraints, 'constraints') : {}) : data;
  for (const key of [...Object.keys(DEFAULTS), 'servo_max_deg']) {
    if (key in constraints) source[key] = constraints[key];
  }
  const normalized = { ...source };
  if (typeof source.cycle_axis !== 'string') throw new Error('cycle_axis must be a string.');
  normalized.cycle_axis = source.cycle_axis.toLowerCase();
  if ('servo_max_deg' in source) nonnegative(source.servo_max_deg, 'servo_max_deg');
  if (!('servo_travel_bounds_deg' in source) && 'servo_max_deg' in source) {
    normalized.servo_travel_bounds_deg = [-source.servo_max_deg, source.servo_max_deg];
  }
  for (const [key, value] of Object.entries(DEFAULTS)) {
    if (!(key in normalized)) normalized[key] = Array.isArray(value) ? value.slice() : value;
  }
  for (const key of [...TRANSLATIONS, ...ROTATIONS, 'rod_length_bounds_mm', 'horn_length_bounds_mm', 'servo_travel_bounds_deg']) {
    normalized[key] = range(normalized[key], key);
  }
  validatePhysicalRequirements(normalized);
  const workspace = {};
  [...TRANSLATIONS, ...ROTATIONS].forEach((key, index) => {
    const [min, max] = normalized[key];
    workspace[['x', 'y', 'z', 'rx', 'ry', 'rz'][index]] = { min, max, step: (max - min) / 2 || 5 };
  });
  return { normalized, workspace };
}
