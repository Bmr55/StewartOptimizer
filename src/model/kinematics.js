import { clamp, vectorDot } from '../math.js';

const EPS = 1e-8;

// Local horn axes at angle alpha: X follows the horn, Y is tangential to its
// servo plane, and Z completes the right-handed frame. Positive alpha raises X.
export function hornFrameAxes(beta, alpha) {
  const radial = [Math.cos(beta), Math.sin(beta), 0];
  const tangent = [-Math.sin(beta), Math.cos(beta), 0];
  return [
    [Math.cos(alpha) * radial[0], Math.cos(alpha) * radial[1], Math.sin(alpha)],
    tangent,
    [-Math.sin(alpha) * radial[0], -Math.sin(alpha) * radial[1], Math.cos(alpha)],
  ];
}

export function hornLocalToWorld(beta, alpha, direction) {
  const axes = hornFrameAxes(beta, alpha);
  return [0, 1, 2].map(row => axes[0][row] * direction[0]
    + axes[1][row] * direction[1] + axes[2][row] * direction[2]);
}

export function worldToHornLocal(beta, alpha, direction) {
  return hornFrameAxes(beta, alpha).map(axis => vectorDot(axis, direction));
}

export function solveServoAngle(base, platformPoint, hornLength, rodLength, beta) {
  const dx = platformPoint[0] - base[0];
  const dy = platformPoint[1] - base[1];
  const dz = platformPoint[2] - base[2];
  const e = 2 * hornLength * dz;
  const f = 2 * hornLength * (Math.cos(beta) * dx + Math.sin(beta) * dy);
  const g = dx * dx + dy * dy + dz * dz - (rodLength * rodLength - hornLength * hornLength);
  const denom = Math.hypot(e, f);
  if (!Number.isFinite(denom) || denom < EPS) {
    return { violation: { type: 'degenerateFourBar', value: denom } };
  }
  const ratio = g / denom;
  if (!Number.isFinite(ratio) || Math.abs(ratio) > 1 + 1e-6) {
    return { violation: { type: 'invalidGeometry', value: ratio } };
  }
  return { alpha: Math.asin(clamp(ratio, -1, 1)) - Math.atan2(f, e) };
}

export function computeHornTip(base, hornLength, beta, alpha) {
  const x = hornFrameAxes(beta, alpha)[0];
  return [base[0] + hornLength * x[0], base[1] + hornLength * x[1], base[2] + hornLength * x[2]];
}
