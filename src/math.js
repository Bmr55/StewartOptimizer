export const DEG2RAD = Math.PI / 180;
export const RAD2DEG = 180 / Math.PI;

export function degToRad(value) {
  return value * DEG2RAD;
}

export function radToDeg(value) {
  return value * RAD2DEG;
}

export function clamp(value, min, max) {
  if (Number.isNaN(value)) return min;
  return Math.min(Math.max(value, min), max);
}

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function vectorAdd(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function vectorSub(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function vectorScale(a, s) {
  return [a[0] * s, a[1] * s, a[2] * s];
}

export function vectorDot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function vectorCross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

export function vectorMagnitude(a) {
  return Math.sqrt(vectorDot(a, a));
}

export function vectorNormalize(a) {
  const mag = vectorMagnitude(a);
  if (mag === 0) return [0, 0, 0];
  return vectorScale(a, 1 / mag);
}

export function rotationMatrixFromEuler(rx, ry, rz) {
  const cx = Math.cos(rx);
  const sx = Math.sin(rx);
  const cy = Math.cos(ry);
  const sy = Math.sin(ry);
  const cz = Math.cos(rz);
  const sz = Math.sin(rz);

  return [
    [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
    [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
    [-sy, cy * sx, cy * cx],
  ];
}

export function rotateVector(matrix, vector) {
  return [
    matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
    matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
    matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
  ];
}

export function rangeCount(range) {
  if (!range) return 1;
  const { min, max, step } = range;
  if (![min, max, step].every(Number.isFinite) || max < min || step <= 0) {
    throw new RangeError('Sweep ranges require finite min/max, max >= min and a positive step.');
  }
  const count = Math.floor((max - min) / step + 1e-6) + 1;
  if (!Number.isSafeInteger(count) || count < 1) throw new RangeError('Sweep range is too large.');
  return count;
}

export function buildRange(range, fallback = 0) {
  if (!range) return [fallback];
  const count = rangeCount(range);
  if (count > 100000) throw new RangeError('A sweep axis cannot exceed 100,000 samples.');
  return Array.from({ length: count }, (_, index) => range.min + index * range.step);
}

export function cloneMatrix(matrix) {
  return matrix.map((row) => row.slice());
}

function identityMatrix(size) {
  const m = Array.from({ length: size }, () => new Array(size).fill(0));
  for (let i = 0; i < size; i++) {
    m[i][i] = 1;
  }
  return m;
}

export function jacobiEigenvaluesSymmetric(matrix, tolerance = 1e-9, maxIterations = 64) {
  const n = matrix.length;
  const a = cloneMatrix(matrix);
  const v = identityMatrix(n);

  for (let iter = 0; iter < maxIterations; iter++) {
    let p = 0;
    let q = 1;
    let maxOff = Math.abs(a[p][q]);

    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        const value = Math.abs(a[i][j]);
        if (value > maxOff) {
          maxOff = value;
          p = i;
          q = j;
        }
      }
    }

    if (maxOff < tolerance) {
      break;
    }

    const app = a[p][p];
    const aqq = a[q][q];
    const apq = a[p][q];
    const phi = 0.5 * Math.atan2(2 * apq, aqq - app);
    const c = Math.cos(phi);
    const s = Math.sin(phi);

    for (let k = 0; k < n; k++) {
      const aik = a[k][p];
      const aiq = a[k][q];
      a[k][p] = c * aik - s * aiq;
      a[k][q] = s * aik + c * aiq;
    }

    for (let k = 0; k < n; k++) {
      const akp = a[p][k];
      const akq = a[q][k];
      a[p][k] = c * akp - s * akq;
      a[q][k] = s * akp + c * akq;
    }

    a[p][p] = c * c * app - 2 * s * c * apq + s * s * aqq;
    a[q][q] = s * s * app + 2 * s * c * apq + c * c * aqq;
    a[p][q] = 0;
    a[q][p] = 0;
  }

  const eigenvalues = new Array(n);
  for (let i = 0; i < n; i++) {
    eigenvalues[i] = a[i][i];
  }
  eigenvalues.sort((aVal, bVal) => bVal - aVal);
  return eigenvalues;
}

export function singularValues(matrix) {
  const rows = matrix.length;
  const cols = matrix[0]?.length || 0;
  if (!rows || !cols) return [];
  const jt = new Array(cols).fill(null).map(() => new Array(rows).fill(0));
  for (let i = 0; i < rows; i++) {
    for (let j = 0; j < cols; j++) {
      jt[j][i] = matrix[i][j];
    }
  }

  const jtj = new Array(cols).fill(null).map(() => new Array(cols).fill(0));
  for (let i = 0; i < cols; i++) {
    for (let j = 0; j < cols; j++) {
      let sum = 0;
      for (let k = 0; k < rows; k++) {
        sum += jt[i][k] * jt[j][k];
      }
      jtj[i][j] = sum;
    }
  }

  const eigenvalues = jacobiEigenvaluesSymmetric(jtj);
  return eigenvalues.map((val) => Math.sqrt(Math.max(0, val)));
}

export function randomNormal(random = Math.random) {
  let u = 0;
  let v = 0;
  while (u === 0) u = random();
  while (v === 0) v = random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export function average(values) {
  if (!values.length) return 0;
  const sum = values.reduce((acc, v) => acc + v, 0);
  return sum / values.length;
}

export function sum(values) {
  return values.reduce((acc, v) => acc + v, 0);
}

export function variance(values) {
  if (!values.length) return 0;
  const mean = average(values);
  const squared = values.map((value) => {
    const diff = value - mean;
    return diff * diff;
  });
  return average(squared);
}

export function standardDeviation(values) {
  if (!values.length) return 0;
  return Math.sqrt(variance(values));
}

export function squaredDistance(a, b) {
  const dx = a[0] - b[0];
  const dy = a[1] - b[1];
  const dz = a[2] - b[2];
  return dx * dx + dy * dy + dz * dz;
}

export function toDegreesVector([rx, ry, rz]) {
  return [radToDeg(rx), radToDeg(ry), radToDeg(rz)];
}
