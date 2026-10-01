import test from 'node:test';
import assert from 'node:assert/strict';
import { rotationMatrixFromEuler, rotateVector } from '../../src/math.js';
import { eulerZYX, matMul, matVec, rotX, rotY, rotZ } from '../fixtures/independent-geometry.js';

// rotationMatrixFromEuler is checked against the documented convention,
// R = Rz(rz) Ry(ry) Rx(rx), built from hand-written elementary rotations,
// and against what each rotation does to the base axes.

const closeMatrix = (actual, expected, tolerance = 1e-12) => actual.forEach((row, i) => row.forEach((value, j) =>
  assert.ok(Math.abs(value - expected[i][j]) <= tolerance, `[${i}][${j}] ${value} != ${expected[i][j]}`)));
const closeVector = (actual, expected, tolerance = 1e-12) => actual.forEach((value, k) =>
  assert.ok(Math.abs(value - expected[k]) <= tolerance, `component ${k}: ${value} != ${expected[k]}`));
const deg = Math.PI / 180;
const [ex, ey, ez] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

test('each single-axis Euler angle is the right-handed rotation about that base axis', () => {
  for (const angle of [-170, -90, -33, 0, 12.5, 45, 90, 179].map(value => value * deg)) {
    closeMatrix(rotationMatrixFromEuler(angle, 0, 0), rotX(angle));
    closeMatrix(rotationMatrixFromEuler(0, angle, 0), rotY(angle));
    closeMatrix(rotationMatrixFromEuler(0, 0, angle), rotZ(angle));
  }
  // A positive quarter turn moves the axes counterclockwise seen from the tip
  // of the rotation axis: roll lifts +Y to +Z, pitch tips +Z onto +X, yaw
  // swings +X to +Y.
  closeVector(rotateVector(rotationMatrixFromEuler(90 * deg, 0, 0), ey), ez);
  closeVector(rotateVector(rotationMatrixFromEuler(0, 90 * deg, 0), ez), ex);
  closeVector(rotateVector(rotationMatrixFromEuler(0, 0, 90 * deg), ex), ey);
});

test('combined angles apply roll, then pitch, then yaw about the fixed base axes', () => {
  // Roll then yaw sends +Y to +Z (roll lifts it, yaw leaves Z alone). The
  // opposite order would send it to -X.
  closeVector(rotateVector(rotationMatrixFromEuler(90 * deg, 0, 90 * deg), ey), ez);
  // Pitch then yaw sends +Z to +Y; yaw first would leave +Z for pitch to tip onto +X.
  closeVector(rotateVector(rotationMatrixFromEuler(0, 90 * deg, 90 * deg), ez), ey);
  // Roll then pitch sends +Y to +X (roll lifts it to +Z, pitch tips that onto
  // +X); pitch first would leave +Y for roll to lift to +Z.
  closeVector(rotateVector(rotationMatrixFromEuler(90 * deg, 90 * deg, 0), ey), ex);
  for (const [rx, ry, rz] of [[0.3, -0.2, 0.9], [-1.1, 0.7, -2.4], [2.9, -1.3, 0.05], [0.01, 0.02, -0.03]]) {
    const actual = rotationMatrixFromEuler(rx, ry, rz);
    closeMatrix(actual, eulerZYX(rx, ry, rz));
    for (const v of [ex, ey, ez, [3, -4, 12]]) closeVector(rotateVector(actual, v), matVec(rotZ(rz), matVec(rotY(ry), matVec(rotX(rx), v))));
  }
});

test('every Euler matrix is a proper rotation', () => {
  let seed = 7;
  const random = () => (seed = (seed * 48271) % 2147483647) / 2147483647;
  for (let k = 0; k < 200; k++) {
    const R = rotationMatrixFromEuler(...[0, 1, 2].map(() => (random() * 2 - 1) * Math.PI));
    const transpose = [0, 1, 2].map(i => [0, 1, 2].map(j => R[j][i]));
    closeMatrix(matMul(transpose, R), [ex, ey, ez]);
    const det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
      - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
      + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
    assert.ok(Math.abs(det - 1) < 1e-12, `det ${det}`);
  }
});

test('small Euler angles rotate by their own vector: R = I + [w]x to first order', () => {
  // With w = (rx, ry, rz) small, R v = v + w x v plus second-order terms. This
  // pins every sign of the convention without reusing the elementary matrices.
  const w = [2e-6, -3e-6, 5e-6];
  const R = rotationMatrixFromEuler(...w);
  const skew = [[1, -w[2], w[1]], [w[2], 1, -w[0]], [-w[1], w[0], 1]];
  closeMatrix(R, skew, 1e-10);
});
