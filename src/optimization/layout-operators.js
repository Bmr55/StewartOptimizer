import { clamp, randomNormal, degToRad } from '../math.js';
import { DEFAULT_TOPOLOGY, TOPOLOGIES } from '../contracts.js';
import { topologyGeometry, validateTopology, wrapAngle } from './topology.js';

export const DEFAULT_DESIGN_SPACE = {
  baseRadius: [90, 160], platformRadius: [40, 120], homeHeightBounds: [50, 450],
  hornLengthBounds: [30, 120], rodLengthBounds: [160, 420],
  pairGapBounds: [12, 45], rectangularAspectBounds: [0.6, 1.4],
  betaJitterRad: degToRad(20), anchorJitter: 6, platformJitter: 6, baseZJitter: 2,
  mutationHorn: 4, mutationRod: 6, mutationHeight: 15, mutationAngle: degToRad(4),
};

const randomInRange = ([min, max]) => min + Math.random() * (max - min);

function pairGapRange(radius, space) {
  const [min, max] = space.pairGapBounds;
  const ceiling = Math.min(max, 1.2 * radius);
  if (ceiling < min) throw new Error('pairGapBounds cannot fit the selected radius bounds.');
  return [min, ceiling];
}

function clampPointRadius(anchor, bounds) {
  const radius = Math.hypot(anchor[0], anchor[1]);
  if (radius < 1e-12) {
    anchor[0] = bounds[0]; anchor[1] = 0;
  } else {
    const scale = clamp(radius, ...bounds) / radius;
    anchor[0] *= scale; anchor[1] *= scale;
  }
}

function randomParameters(topology, space) {
  const radiusBounds = bounds => topology === 'c3_paired'
    ? [Math.max(bounds[0], space.pairGapBounds[0] / 1.2), bounds[1]] : bounds;
  const p = {
    base_radius: randomInRange(radiusBounds(space.baseRadius)),
    platform_radius: randomInRange(radiusBounds(space.platformRadius)),
    base_orientation: randomInRange([-Math.PI, Math.PI]),
    platform_orientation: randomInRange([-Math.PI, Math.PI]),
    beta_offset: randomInRange([-space.betaJitterRad, space.betaJitterRad]),
  };
  if (topology === 'c3_paired') {
    p.base_pair_gap = randomInRange(pairGapRange(p.base_radius, space));
    p.platform_pair_gap = randomInRange(pairGapRange(p.platform_radius, space));
  }
  if (topology === 'rectangular_paired') {
    p.base_aspect = randomInRange(space.rectangularAspectBounds);
    p.platform_aspect = randomInRange(space.rectangularAspectBounds);
  }
  return p;
}

export function validateDesignSpace(space) {
  for (const field of ['baseRadius', 'platformRadius', 'homeHeightBounds', 'hornLengthBounds',
    'rodLengthBounds', 'pairGapBounds', 'rectangularAspectBounds']) {
    const range = space[field];
    if (!Array.isArray(range) || range.length !== 2 || !range.every(Number.isFinite)
      || range[0] <= 0 || range[1] < range[0]) {
      throw new Error(`${field} must contain two positive finite bounds with max >= min.`);
    }
  }
  if (space.pairGapBounds[0] >= 1.2 * Math.min(space.baseRadius[1], space.platformRadius[1])) {
    throw new Error('pairGapBounds cannot fit baseRadius and platformRadius.');
  }
}

export const cloneLayout = layout => JSON.parse(JSON.stringify(layout));

export function createRandomLayout({ designSpace: space, servoRangeRad, id, topology = DEFAULT_TOPOLOGY }) {
  if (!TOPOLOGIES.includes(topology)) throw new Error(`topology must be one of ${TOPOLOGIES.join(', ')}.`);
  const layout = {
    id, topology, topologyParameters: topology === 'free' ? {} : randomParameters(topology, space),
    baseAnchors: [], platformAnchors: [], betaAngles: [],
    hornLength: randomInRange(space.hornLengthBounds), rodLength: randomInRange(space.rodLengthBounds),
    servoRangeRad: servoRangeRad.slice(), homeHeight: randomInRange(space.homeHeightBounds),
  };
  if (topology === 'free') {
    const baseOffset = Math.random() * 2 * Math.PI;
    for (let i = 0; i < 6; i++) {
      const angle = baseOffset + i * Math.PI / 3 + randomNormal() * degToRad(3);
      const upperAngle = baseOffset + Math.PI / 6 + i * Math.PI / 3 + randomNormal() * degToRad(3);
      const br = randomInRange(space.baseRadius);
      const pr = randomInRange(space.platformRadius);
      layout.baseAnchors.push([br * Math.cos(angle), br * Math.sin(angle),
        randomNormal() * space.baseZJitter]);
      layout.platformAnchors.push([pr * Math.cos(upperAngle), pr * Math.sin(upperAngle), 0]);
      layout.betaAngles.push(wrapAngle(angle + Math.PI / 2
        + randomNormal() * space.betaJitterRad));
    }
  } else Object.assign(layout, topologyGeometry(topology, layout.topologyParameters));
  return finalizeLayout(layout, { designSpace: space, servoRangeRad });
}

export function finalizeLayout(layout, { designSpace: space, servoRangeRad }) {
  const topology = validateTopology(layout);
  layout.topology = topology;
  layout.topologyParameters ??= topology === 'free' ? {} : layout.topology_parameters;
  layout.hornLength = clamp(layout.hornLength, ...space.hornLengthBounds);
  layout.rodLength = clamp(layout.rodLength, ...space.rodLengthBounds);
  layout.homeHeight = clamp(layout.homeHeight, ...space.homeHeightBounds);
  if (topology === 'free') {
    for (const point of layout.baseAnchors) {
      clampPointRadius(point, space.baseRadius);
      point[2] = clamp(point[2], -space.baseZJitter, space.baseZJitter);
    }
    for (const point of layout.platformAnchors) clampPointRadius(point, space.platformRadius);
  } else {
    const p = layout.topologyParameters;
    const minimum = topology === 'c3_paired' ? space.pairGapBounds[0] / 1.2 : 0;
    p.base_radius = clamp(p.base_radius, Math.max(space.baseRadius[0], minimum), space.baseRadius[1]);
    p.platform_radius = clamp(p.platform_radius, Math.max(space.platformRadius[0], minimum), space.platformRadius[1]);
    for (const field of ['base_orientation', 'platform_orientation', 'beta_offset']) {
      p[field] = wrapAngle(p[field]);
    }
    if (topology === 'c3_paired') {
      p.base_pair_gap = clamp(p.base_pair_gap, ...pairGapRange(p.base_radius, space));
      p.platform_pair_gap = clamp(p.platform_pair_gap, ...pairGapRange(p.platform_radius, space));
    }
    if (topology === 'rectangular_paired') {
      p.base_aspect = clamp(p.base_aspect, ...space.rectangularAspectBounds);
      p.platform_aspect = clamp(p.platform_aspect, ...space.rectangularAspectBounds);
    }
    Object.assign(layout, topologyGeometry(topology, p));
  }
  layout.servoRangeRad = servoRangeRad.slice();
  return layout;
}

export function mutateLayout(source, { designSpace: space, servoRangeRad }) {
  validateTopology(source);
  const layout = cloneLayout(source);
  if ((layout.topology ?? 'free') === 'free') {
    for (let i = 0; i < 6; i++) {
      layout.baseAnchors[i][0] += randomNormal() * space.anchorJitter;
      layout.baseAnchors[i][1] += randomNormal() * space.anchorJitter;
      layout.baseAnchors[i][2] += randomNormal() * space.baseZJitter;
      layout.platformAnchors[i][0] += randomNormal() * space.platformJitter;
      layout.platformAnchors[i][1] += randomNormal() * space.platformJitter;
      layout.betaAngles[i] = wrapAngle(layout.betaAngles[i]
        + randomNormal() * space.mutationAngle);
    }
  } else {
    const p = layout.topologyParameters;
    const minimum = layout.topology === 'c3_paired' ? space.pairGapBounds[0] / 1.2 : 0;
    p.base_radius = clamp(p.base_radius + randomNormal() * space.anchorJitter,
      Math.max(space.baseRadius[0], minimum), space.baseRadius[1]);
    p.platform_radius = clamp(p.platform_radius + randomNormal() * space.platformJitter,
      Math.max(space.platformRadius[0], minimum), space.platformRadius[1]);
    p.base_orientation += randomNormal() * space.mutationAngle;
    p.platform_orientation += randomNormal() * space.mutationAngle;
    p.beta_offset += randomNormal() * space.mutationAngle;
    if (layout.topology === 'c3_paired') {
      p.base_pair_gap = clamp(p.base_pair_gap + randomNormal() * space.anchorJitter,
        ...pairGapRange(p.base_radius, space));
      p.platform_pair_gap = clamp(p.platform_pair_gap + randomNormal() * space.platformJitter,
        ...pairGapRange(p.platform_radius, space));
    }
    if (layout.topology === 'rectangular_paired') {
      p.base_aspect = clamp(p.base_aspect + randomNormal() * 0.05,
        ...space.rectangularAspectBounds);
      p.platform_aspect = clamp(p.platform_aspect + randomNormal() * 0.05,
        ...space.rectangularAspectBounds);
    }
    Object.assign(layout, topologyGeometry(layout.topology, p));
  }
  layout.hornLength += randomNormal() * space.mutationHorn;
  layout.rodLength += randomNormal() * space.mutationRod;
  layout.homeHeight += randomNormal() * space.mutationHeight;
  return finalizeLayout(layout, { designSpace: space, servoRangeRad });
}

export function crossoverLayouts(a, b, { designSpace: space, servoRangeRad }) {
  validateTopology(a); validateTopology(b);
  if ((a.topology ?? 'free') !== (b.topology ?? 'free')) {
    throw new Error('Cannot cross layouts with different topologies.');
  }
  const layout = cloneLayout(a);
  if ((layout.topology ?? 'free') === 'free') {
    const split = Math.floor(Math.random() * 6);
    for (let i = split; i < 6; i++) {
      layout.baseAnchors[i] = b.baseAnchors[i].slice();
      layout.platformAnchors[i] = b.platformAnchors[i].slice();
      layout.betaAngles[i] = b.betaAngles[i];
    }
  } else {
    for (const field of Object.keys(layout.topologyParameters)) {
      layout.topologyParameters[field] = Math.random() < 0.5
        ? a.topologyParameters[field] : b.topologyParameters[field];
    }
    Object.assign(layout, topologyGeometry(layout.topology, layout.topologyParameters));
  }
  layout.hornLength = (a.hornLength + b.hornLength) / 2;
  layout.rodLength = (a.rodLength + b.rodLength) / 2;
  layout.homeHeight = Math.random() < 0.5 ? a.homeHeight : b.homeHeight;
  return finalizeLayout(layout, { designSpace: space, servoRangeRad });
}
