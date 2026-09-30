import { vectorCross, vectorDot, vectorMagnitude, vectorSub } from '../math.js';

export const NUMERICAL_RECIPROCAL_CUTOFF = 1e-10;
const LEVERAGE_CUTOFF = 1e-10;

export function validateConditionLimit(conditionLimit) {
  if (conditionLimit == null) return null;
  if (!Number.isFinite(conditionLimit) || conditionLimit < 1) {
    throw new RangeError('conditionLimit must be a finite number >= 1.');
  }
  return conditionLimit;
}

// One-sided Jacobi SVD acts on the matrix itself. Forming J^T J loses the
// small singular values needed at the 1e-10 reciprocal-condition boundary.
export function singularValuesOneSided(matrix) {
  const n = matrix.length;
  if (!n || matrix.some(row => !Array.isArray(row) || row.length !== n
      || row.some(value => !Number.isFinite(value)))) return null;
  const scale = Math.max(...matrix.flat().map(Math.abs));
  if (scale === 0 || !Number.isFinite(scale)) return new Array(n).fill(0);
  const columns = Array.from({ length: n }, (_, col) => matrix.map(row => row[col] / scale));
  const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
  let converged = false;
  for (let sweep = 0; sweep < 80; sweep++) {
    let rotated = false;
    for (let p = 0; p < n - 1; p++) {
      for (let q = p + 1; q < n; q++) {
        const a = dot(columns[p], columns[p]);
        const b = dot(columns[q], columns[q]);
        const c = dot(columns[p], columns[q]);
        if (a === 0 || b === 0 || Math.abs(c) <= 4 * Number.EPSILON * Math.sqrt(a * b)) continue;
        rotated = true;
        const tau = (b - a) / (2 * c);
        const t = (tau >= 0 ? 1 : -1) / (Math.abs(tau) + Math.hypot(1, tau));
        const cosine = 1 / Math.hypot(1, t);
        const sine = t * cosine;
        const left = columns[p], right = columns[q];
        for (let row = 0; row < n; row++) {
          const lp = left[row], rq = right[row];
          left[row] = cosine * lp - sine * rq;
          right[row] = sine * lp + cosine * rq;
        }
      }
    }
    if (!rotated) { converged = true; break; }
  }
  if (!converged) return null;
  return columns.map(col => Math.hypot(...col) * scale).sort((a, b) => b - a);
}

export function assessJacobian(rows, conditionLimit = null) {
  validateConditionLimit(conditionLimit);
  const singularValues = singularValuesOneSided(rows);
  if (!singularValues) return { available: false, satisfied: false,
    numericalSingularity: true, engineeringFailure: false, reason: 'nonfiniteOrNonconvergent',
    singularValues: null, condition: null, reciprocal: null };
  const sigmaMax = singularValues[0];
  const sigmaMin = singularValues.at(-1);
  const reciprocal = sigmaMax > 0 ? sigmaMin / sigmaMax : 0;
  const condition = reciprocal > 0 ? 1 / reciprocal : null;
  const numericalSingularity = !Number.isFinite(reciprocal)
    || reciprocal <= NUMERICAL_RECIPROCAL_CUTOFF;
  const engineeringFailure = !numericalSingularity && conditionLimit != null
    && condition > conditionLimit;
  return { available: true, satisfied: !numericalSingularity && !engineeringFailure,
    numericalSingularity, engineeringFailure, singularValues,
    sigmaMax, sigmaMin, condition, reciprocal };
}

export function rotaryActuatorJacobian(platformPoints, rodVectors, servoAngles,
  betaAngles, hornLength) {
  if ([platformPoints, rodVectors, servoAngles, betaAngles].some(values => values.length !== 6)) {
    return { rows: null, reason: 'incompletePose' };
  }
  const centroid = [0, 1, 2].map(axis => platformPoints.reduce((sum, q) => sum + q[axis], 0) / 6);
  const offsets = platformPoints.map(q => vectorSub(q, centroid));
  const radius = Math.sqrt(offsets.reduce((sum, r) => sum + vectorDot(r, r), 0) / 6);
  if (!Number.isFinite(radius) || radius <= 0) return { rows: null, centroid, radius, reason: 'anchorRadius' };
  const rows = [];
  for (let leg = 0; leg < 6; leg++) {
    const rod = rodVectors[leg];
    const rodLength = vectorMagnitude(rod);
    if (!Number.isFinite(rodLength) || rodLength <= 0) {
      return { rows: null, centroid, radius, leg, reason: 'rodDirection' };
    }
    const u = rod.map(value => value / rodLength);
    const alpha = servoAngles[leg], beta = betaAngles[leg];
    const tangent = [
      -hornLength * Math.sin(alpha) * Math.cos(beta),
      -hornLength * Math.sin(alpha) * Math.sin(beta),
      hornLength * Math.cos(alpha),
    ];
    const leverage = vectorDot(u, tangent);
    if (!Number.isFinite(leverage) || Math.abs(leverage) <= LEVERAGE_CUTOFF * hornLength) {
      return { rows: null, centroid, radius, leg, leverage,
        reason: 'servoLeverage' };
    }
    rows.push([...u.map(value => radius * value), ...vectorCross(offsets[leg], u)]
      .map(value => value / leverage));
  }
  return { rows, centroid, radius };
}

export function assessPoseConditioning(platformPoints, rodVectors, servoAngles,
  betaAngles, hornLength, conditionLimit = null) {
  const geometry = rotaryActuatorJacobian(platformPoints, rodVectors, servoAngles,
    betaAngles, hornLength);
  if (!geometry.rows) return { available: false, satisfied: false,
    numericalSingularity: true, engineeringFailure: false, reason: geometry.reason,
    leg: geometry.leg ?? null, leverage: geometry.leverage ?? null,
    condition: null, reciprocal: null, singularValues: null, jacobianRows: null,
    centroid: geometry.centroid ?? null, radius: geometry.radius ?? null };
  const assessment = assessJacobian(geometry.rows, conditionLimit);
  return { ...assessment, reason: assessment.numericalSingularity ? 'singularValues'
    : assessment.engineeringFailure ? 'engineeringLimit' : null,
    jacobianRows: geometry.rows, centroid: geometry.centroid, radius: geometry.radius };
}
