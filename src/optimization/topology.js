import { DEFAULT_TOPOLOGY, TOPOLOGIES } from '../contracts.js';

export function wrapAngle(angle) {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

function point(radius, angle) {
  return [radius * Math.cos(angle), radius * Math.sin(angle), 0];
}

function rotate([x, y, z], angle) {
  return [x * Math.cos(angle) - y * Math.sin(angle),
    x * Math.sin(angle) + y * Math.cos(angle), z];
}

function ring(radius, orientation) {
  return Array.from({ length: 6 }, (_, i) => point(radius, orientation + i * Math.PI / 3));
}

function pairedRing(radius, gap, orientation) {
  const halfAngle = Math.asin(gap / (2 * radius));
  return Array.from({ length: 6 }, (_, i) => point(radius,
    orientation + Math.floor(i / 2) * 2 * Math.PI / 3 + (i % 2 ? halfAngle : -halfAngle)));
}

function rectangle(radius, aspect, orientation) {
  const halfDepth = radius / Math.hypot(aspect, 1);
  const halfWidth = aspect * halfDepth;
  return [-1, 0, 1].flatMap(row => [-1, 1].map(column =>
    rotate([column * halfWidth, row * halfDepth, 0], orientation)));
}

export function topologyGeometry(topology, parameters) {
  if (!TOPOLOGIES.includes(topology)) throw new Error(`topology must be one of ${TOPOLOGIES.join(', ')}.`);
  if (topology === 'free') throw new Error('Free topology has no parametric geometry.');
  if (!parameters || typeof parameters !== 'object' || Array.isArray(parameters)) {
    throw new Error('topology_parameters must be an object.');
  }
  const fields = topology === 'c3_paired'
    ? ['base_radius', 'platform_radius', 'base_pair_gap', 'platform_pair_gap',
      'base_orientation', 'platform_orientation', 'beta_offset']
    : topology === 'rectangular_paired'
      ? ['base_radius', 'platform_radius', 'base_aspect', 'platform_aspect',
        'base_orientation', 'platform_orientation', 'beta_offset']
      : ['base_radius', 'platform_radius', 'base_orientation', 'platform_orientation', 'beta_offset'];
  for (const field of fields) {
    if (!Number.isFinite(parameters[field])) throw new Error(`topology_parameters.${field} must be finite.`);
  }
  for (const field of ['base_radius', 'platform_radius']) {
    if (parameters[field] <= 0) throw new Error(`topology_parameters.${field} must be positive.`);
  }
  if (topology === 'c3_paired') {
    for (const [gap, radius] of [['base_pair_gap', 'base_radius'], ['platform_pair_gap', 'platform_radius']]) {
      if (parameters[gap] <= 0 || parameters[gap] >= 2 * parameters[radius]) {
        throw new Error(`topology_parameters.${gap} must be positive and less than twice ${radius}.`);
      }
    }
  }
  if (topology === 'rectangular_paired') {
    for (const field of ['base_aspect', 'platform_aspect']) {
      if (parameters[field] <= 0) throw new Error(`topology_parameters.${field} must be positive.`);
    }
  }
  const make = (radius, orientation, pairGap, aspect) => {
    if (topology === 'circular') return ring(radius, orientation);
    if (topology === 'c3_paired') return pairedRing(radius, pairGap, orientation);
    return rectangle(radius, aspect, orientation);
  };
  const baseAnchors = make(parameters.base_radius, parameters.base_orientation,
    parameters.base_pair_gap, parameters.base_aspect);
  const platformAnchors = make(parameters.platform_radius, parameters.platform_orientation,
    parameters.platform_pair_gap, parameters.platform_aspect);
  const betaAngles = baseAnchors.map(([x, y]) =>
    wrapAngle(Math.atan2(y, x) + Math.PI / 2 + parameters.beta_offset));
  return { baseAnchors, platformAnchors, betaAngles };
}

// A declared family is a claim about the actual anchors, never an instruction
// to reshape imported coordinates. The importer in #22 calls this before use.
export function validateTopology(layout) {
  const topology = layout.topology ?? 'free';
  if (!TOPOLOGIES.includes(topology)) throw new Error(`topology must be one of ${TOPOLOGIES.join(', ')}.`);
  if (topology === 'free') return topology;
  const expected = topologyGeometry(topology, layout.topologyParameters ?? layout.topology_parameters);
  for (const [field, values] of Object.entries(expected)) {
    const actual = layout[field] ?? layout[{ baseAnchors: 'base_anchors',
      platformAnchors: 'platform_anchors', betaAngles: 'beta_angles' }[field]];
    if (!Array.isArray(actual) || actual.length !== 6) {
      throw new Error(`${field} must contain six entries for declared topology ${topology}.`);
    }
    for (let i = 0; i < 6; i++) {
      const a = actual[i];
      const b = values[i];
      const good = field === 'betaAngles'
        ? Number.isFinite(a) && Math.abs(wrapAngle(a - b)) <= 1e-7
        : Array.isArray(a) && a.length === 3 && a.every((value, j) =>
          Number.isFinite(value) && Math.abs(value - b[j]) <= 1e-7 * Math.max(1, Math.abs(b[j])));
      if (!good) throw new Error(`${field}[${i}] conflicts with declared topology ${topology} and topology_parameters.`);
    }
  }
  return topology;
}

export { DEFAULT_TOPOLOGY };
