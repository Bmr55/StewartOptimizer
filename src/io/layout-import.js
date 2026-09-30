import { degToRad } from '../math.js';
import { MODEL_VERSION, SCHEMA_VERSION, TOPOLOGIES } from '../contracts.js';
import { resolveMounting } from '../model/mounting.js';
import { validateTopology } from '../optimization/topology.js';

function object(value, field) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) {
    throw new TypeError(`${field} must be an object.`);
  }
  return value;
}

function anchors(value, field) {
  if (!Array.isArray(value) || value.length !== 6) {
    throw new RangeError(`${field} must contain six 3D anchors.`);
  }
  value.forEach((point, i) => {
    if (!Array.isArray(point) || point.length !== 3 || !point.every(Number.isFinite)) {
      throw new RangeError(`${field}[${i}] must contain three finite coordinates.`);
    }
  });
  return value.map(point => point.slice());
}

function sixAngles(value, field) {
  if (!Array.isArray(value) || value.length !== 6) {
    throw new RangeError(`${field} must contain six finite angles in radians.`);
  }
  value.forEach((angle, i) => {
    if (!Number.isFinite(angle)) throw new RangeError(`${field}[${i}] must be finite.`);
  });
  return value.slice();
}

function positive(value, field) {
  if (!Number.isFinite(value) || value <= 0) throw new RangeError(`${field} must be a positive finite number.`);
  return value;
}

// Equal bounds lock the servo, which parseRequirements and evaluatePose both
// accept, so a run with a locked range must export a layout that imports and
// replays.
function servoBounds(value, field) {
  if (!Array.isArray(value) || value.length !== 2 || !value.every(Number.isFinite)
      || value[1] < value[0]) {
    throw new RangeError(`${field} must contain two finite bounds with max >= min.`);
  }
  return value.slice();
}

function unwrap(input) {
  const root = object(input, 'layout input');
  if ('result' in root) {
    const result = object(root.result, 'result');
    return { source: object(result.layout, 'result.layout'), run: root.run ?? result.run ?? null };
  }
  if ('layout' in root && !('base_anchors' in root) && !('baseAnchors' in root)) {
    return { source: object(root.layout, 'layout'), run: root.run ?? null };
  }
  return { source: root, run: root.run ?? null };
}

// Discard all imported metrics. Geometry is copied exactly; the current model
// must evaluate it before any score or feasibility flag is trusted.
export function parseLayoutJSON(text) {
  try { return JSON.parse(text); }
  catch { throw new SyntaxError('Layout JSON is invalid.'); }
}

export function importLayout(input) {
  const parsed = typeof input === 'string' ? parseLayoutJSON(input) : input;
  const { source, run } = unwrap(parsed);
  const snake = 'base_anchors' in source || 'platform_anchors' in source;
  const schemaVersion = source.schema_version ?? source.schemaVersion ?? 1;
  const modelVersion = source.model_version ?? source.modelVersion ?? 1;
  if (!Number.isSafeInteger(schemaVersion) || schemaVersion < 1 || schemaVersion > SCHEMA_VERSION) {
    throw new RangeError(`schema_version ${JSON.stringify(schemaVersion)} is unsupported.`);
  }
  if (!Number.isSafeInteger(modelVersion) || modelVersion < 1 || modelVersion > MODEL_VERSION) {
    throw new RangeError(`model_version ${JSON.stringify(modelVersion)} is unsupported.`);
  }
  const field = (snakeName, camelName) => snake ? snakeName : camelName;
  const baseField = field('base_anchors', 'baseAnchors');
  const platformField = field('platform_anchors', 'platformAnchors');
  const betaField = field('beta_angles', 'betaAngles');
  const hornField = field('horn_length', 'hornLength');
  const rodField = field('rod_length', 'rodLength');
  const heightField = field('home_height', 'homeHeight');
  const servoField = field('servo_range', 'servoRangeRad');
  const servo = servoBounds(source[servoField], servoField);
  const topology = source.topology ?? 'free';
  if (!TOPOLOGIES.includes(topology)) throw new RangeError(`topology must be one of ${TOPOLOGIES.join(', ')}.`);
  // Free layouts carry parameters unchecked by validateTopology; they must still
  // be an object and are copied so later input mutation cannot reach the layout.
  const parameters = source.topology_parameters ?? source.topologyParameters ?? null;
  if (parameters !== null) object(parameters, 'topology_parameters');
  const layout = {
    baseAnchors: anchors(source[baseField], baseField),
    platformAnchors: anchors(source[platformField], platformField),
    betaAngles: sixAngles(source[betaField], betaField),
    hornLength: positive(source[hornField], hornField),
    rodLength: positive(source[rodField], rodField),
    homeHeight: positive(source[heightField], heightField),
    servoRangeRad: snake ? servo.map(degToRad) : servo,
    topology,
    topologyParameters: parameters === null ? (topology === 'free' ? {} : null) : structuredClone(parameters),
    modelVersion,
    schemaVersion,
    mounting: source.mounting ?? null,
  };
  if (snake) layout.servoRangeDeg = servo;
  validateTopology(layout);
  const resolved = resolveMounting(layout, { imported: true });
  layout.mounting = resolved.mounting;
  layout.migration = resolved.migration ?? source.migration ?? null;
  return { layout, sourceRun: run };
}

export const parseLayoutInput = importLayout;
