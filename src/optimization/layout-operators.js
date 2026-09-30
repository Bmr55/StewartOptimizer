import { clamp, randomNormal, degToRad, average } from '../math.js';

export const DEFAULT_DESIGN_SPACE = {
  baseRadius: [90, 160],
  platformRadius: [40, 120],
  platformHeight: [100, 180],
  hornLengthBounds: [30, 120],
  rodLengthBounds: [160, 420],
  betaJitterRad: degToRad(20),
  anchorJitter: 6,
  platformJitter: 6,
  baseZJitter: 2,
  mutationHorn: 4,
  mutationRod: 6,
  mutationAngle: degToRad(4),
};

function randomInRange([min, max]) {
  if (max <= min) return min;
  return min + Math.random() * (max - min);
}

function wrapAngle(angle) {
  let a = angle;
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

export function cloneLayout(layout) {
  return JSON.parse(JSON.stringify(layout));
}

export function createRandomLayout({ designSpace, servoRangeRad, id }) {
  const layout = {
    id,
    baseAnchors: [],
    platformAnchors: [],
    betaAngles: [],
    hornLength: randomInRange(designSpace.hornLengthBounds),
    rodLength: randomInRange(designSpace.rodLengthBounds),
    servoRangeRad: servoRangeRad.slice(),
    homeHeight: 120,
  };

  const baseRadius = randomInRange(designSpace.baseRadius);
  const platformRadius = clamp(randomInRange(designSpace.platformRadius), 20, baseRadius - 10);
  const platformHeight = randomInRange(designSpace.platformHeight);
  const baseOffset = Math.random() * 2 * Math.PI;
  const platformOffset = baseOffset + Math.PI / 6;

  for (let i = 0; i < 6; i++) {
    const angle = baseOffset + i * (Math.PI / 3) + randomNormal() * degToRad(3);
    const platformAngle = platformOffset + i * (Math.PI / 3) + randomNormal() * degToRad(3);
    const baseAnchor = [
      baseRadius * Math.cos(angle) + randomNormal() * designSpace.anchorJitter,
      baseRadius * Math.sin(angle) + randomNormal() * designSpace.anchorJitter,
      randomNormal() * designSpace.baseZJitter,
    ];
    const platformAnchor = [
      platformRadius * Math.cos(platformAngle) + randomNormal() * designSpace.platformJitter,
      platformRadius * Math.sin(platformAngle) + randomNormal() * designSpace.platformJitter,
      0,
    ];
    const beta = angle + Math.PI / 2 + randomNormal() * designSpace.betaJitterRad;

    layout.baseAnchors.push(baseAnchor);
    layout.platformAnchors.push(platformAnchor);
    layout.betaAngles.push(wrapAngle(beta));
  }

  layout.homeHeight = platformHeight;
  finalizeLayout(layout, { designSpace, servoRangeRad });
  return layout;
}

export function finalizeLayout(layout, { designSpace, servoRangeRad }) {
  const hornMin = designSpace.hornLengthBounds[0];
  const hornMax = designSpace.hornLengthBounds[1];
  const rodMin = designSpace.rodLengthBounds[0];
  const rodMax = designSpace.rodLengthBounds[1];
  layout.hornLength = clamp(layout.hornLength, hornMin, hornMax);
  layout.rodLength = clamp(layout.rodLength, rodMin, rodMax);

  const heightContributions = [];
  let requiredRodSq = rodMin * rodMin;

  for (let i = 0; i < 6; i++) {
    const base = layout.baseAnchors[i];
    const platform = layout.platformAnchors[i];
    const dx = platform[0] - base[0];
    const dy = platform[1] - base[1];
    const horizontalSq = dx * dx + dy * dy;
    const required = horizontalSq - layout.hornLength * layout.hornLength + 1;
    if (required > requiredRodSq) {
      requiredRodSq = required;
    }
  }

  layout.rodLength = clamp(Math.max(layout.rodLength, Math.sqrt(Math.max(requiredRodSq, 0))), rodMin, rodMax);

  for (let i = 0; i < 6; i++) {
    const base = layout.baseAnchors[i];
    const platform = layout.platformAnchors[i];
    const dx = platform[0] - base[0];
    const dy = platform[1] - base[1];
    const horizontalSq = dx * dx + dy * dy;
    const radicand = layout.rodLength * layout.rodLength
      + layout.hornLength * layout.hornLength
      - horizontalSq;
    heightContributions.push(Math.sqrt(Math.max(radicand, 0)));
  }

  layout.homeHeight = average(heightContributions);
  layout.servoRangeRad = servoRangeRad.slice();
  return layout;
}

export function mutateLayout(source, options) {
  const { designSpace, servoRangeRad } = options;
  const layout = cloneLayout(source);
  for (let i = 0; i < 6; i++) {
    layout.baseAnchors[i][0] += randomNormal() * designSpace.anchorJitter;
    layout.baseAnchors[i][1] += randomNormal() * designSpace.anchorJitter;
    layout.baseAnchors[i][2] += randomNormal() * designSpace.baseZJitter;
    layout.platformAnchors[i][0] += randomNormal() * designSpace.platformJitter;
    layout.platformAnchors[i][1] += randomNormal() * designSpace.platformJitter;
    layout.betaAngles[i] = wrapAngle(layout.betaAngles[i] + randomNormal() * designSpace.mutationAngle);
  }

  layout.hornLength += randomNormal() * designSpace.mutationHorn;
  layout.rodLength += randomNormal() * designSpace.mutationRod;

  return finalizeLayout(layout, { designSpace, servoRangeRad });
}

export function crossoverLayouts(a, b, options) {
  const { designSpace, servoRangeRad } = options;
  const layout = cloneLayout(a);
  const split = Math.floor(Math.random() * 6);
  for (let i = split; i < 6; i++) {
    layout.baseAnchors[i] = b.baseAnchors[i].slice();
    layout.platformAnchors[i] = b.platformAnchors[i].slice();
    layout.betaAngles[i] = b.betaAngles[i];
  }
  layout.hornLength = (a.hornLength + b.hornLength) / 2;
  layout.rodLength = (a.rodLength + b.rodLength) / 2;
  layout.servoRangeRad = servoRangeRad.slice();
  return finalizeLayout(layout, { designSpace, servoRangeRad });
}
