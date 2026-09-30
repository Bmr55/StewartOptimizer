import { vectorMagnitude, vectorNormalize, vectorSub } from '../math.js';
import { computeHornTip, solveServoAngle, worldToHornLocal } from './kinematics.js';

const JOINTS = ['lower', 'upper'];

function normalizeOverride(value, joint, leg) {
  if (value == null) return { source: 'derived' };
  const entry = Array.isArray(value) ? { direction: value, source: 'supplied' } : value;
  if (!entry || typeof entry !== 'object' || Array.isArray(entry)) {
    throw new TypeError(`Mounting ${joint} leg ${leg} must be a direction or joint entry.`);
  }
  const source = entry.source ?? 'supplied';
  if (source !== 'derived' && source !== 'supplied') {
    throw new RangeError(`Mounting ${joint} leg ${leg} has an unknown direction source.`);
  }
  if (source === 'derived') return { source };
  const direction = entry.direction;
  if (!Array.isArray(direction) || direction.length !== 3
      || !direction.every(Number.isFinite) || vectorMagnitude(direction) < 1e-12) {
    throw new RangeError(`Mounting ${joint} leg ${leg} needs a finite, nonzero 3D direction.`);
  }
  return { source, direction: vectorNormalize(direction) };
}

function homeDirections(layout) {
  const lower = [], upper = [];
  for (let leg = 0; leg < 6; leg++) {
    const base = layout.baseAnchors[leg];
    const platform = layout.platformAnchors[leg];
    const q = [platform[0], platform[1], platform[2] + (layout.homeHeight || 0)];
    const beta = layout.betaAngles[leg];
    const solved = solveServoAngle(base, q, layout.hornLength, layout.rodLength, beta);
    if (solved.violation) {
      lower.push(null);
      upper.push(null);
      continue;
    }
    const hornTip = computeHornTip(base, layout.hornLength, beta, solved.alpha);
    const rod = vectorSub(q, hornTip);
    if (!Number.isFinite(vectorMagnitude(rod)) || vectorMagnitude(rod) < 1e-12) {
      lower.push(null);
      upper.push(null);
      continue;
    }
    const towardPlatform = vectorNormalize(rod);
    lower.push(worldToHornLocal(beta, solved.alpha, towardPlatform));
    upper.push(towardPlatform.map(component => -component));
  }
  return { lower, upper };
}

// Derived directions are recalculated against the current home geometry. This
// keeps them home-aligned after evolution or an explicit editable copy.
export function resolveMounting(layout, { imported = false } = {}) {
  if (!layout || !Array.isArray(layout.baseAnchors) || !Array.isArray(layout.platformAnchors)
      || !Array.isArray(layout.betaAngles) || layout.baseAnchors.length !== 6
      || layout.platformAnchors.length !== 6 || layout.betaAngles.length !== 6) {
    throw new TypeError('Six anchors and servo orientations are required to resolve mounting.');
  }
  const input = layout.mounting;
  if (input != null && (typeof input !== 'object' || Array.isArray(input))) {
    throw new TypeError('Mounting must contain lower and upper joint arrays.');
  }
  const home = homeDirections(layout);
  const mounting = {};
  for (const joint of JOINTS) {
    const entries = input?.[joint];
    if (entries != null && (!Array.isArray(entries) || entries.length !== 6)) {
      throw new RangeError(`Mounting ${joint} must contain six leg entries.`);
    }
    mounting[joint] = Array.from({ length: 6 }, (_, leg) => {
      const override = normalizeOverride(entries?.[leg], joint, leg);
      return {
        direction: override.source === 'supplied' ? override.direction : home[joint][leg],
        source: override.source,
      };
    });
  }
  const migration = imported && input == null ? {
    upgraded: true,
    fromModelVersion: layout.modelVersion ?? layout.model_version ?? 1,
    toModelVersion: 2,
    note: 'Legacy single-angle joint model upgraded to home-aligned lower and upper sockets; previous metrics are stale and must be recalculated.',
  } : null;
  return { mounting, migration };
}
