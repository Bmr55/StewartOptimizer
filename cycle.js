import { evaluatePose } from './workspace.js';
import { vectorNormalize, vectorCross, vectorDot, vectorScale, rotateVector } from './math.js';

// Partial-pivot elimination; a singular equilibrium is not a zero-demand design.
function solve(matrix, rhs) {
  const a = matrix.map((row, i) => [...row, rhs[i]]);
  for (let col = 0; col < rhs.length; col++) {
    let pivot = col;
    for (let row = col + 1; row < rhs.length; row++) {
      if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
    }
    if (Math.abs(a[pivot][col]) < 1e-10) return null;
    [a[col], a[pivot]] = [a[pivot], a[col]];
    const scale = a[col][col];
    for (let j = col; j <= rhs.length; j++) a[col][j] /= scale;
    for (let row = 0; row < rhs.length; row++) {
      if (row === col) continue;
      const factor = a[row][col];
      for (let j = col; j <= rhs.length; j++) a[row][j] -= factor * a[col][j];
    }
  }
  const result = a.map(row => row[rhs.length]);
  return result.every(Number.isFinite) ? result : null;
}

export function evaluateCyclePose(layout, pose, velocity, acceleration, mass, options = {}) {
  const result = evaluatePose(layout, pose, { ...options, ballJointClamp: false, recordLegData: true });
  if (!result.reachable) return { valid: false, reason: 'Cycle pose violates a modeled mechanical constraint.' };
  const directions = result.rodVectors.map(vectorNormalize);
  // Each column is a rod's unit force and its moment about the moving origin (meters).
  const columns = directions.map((direction, i) => {
    const arm = vectorScale(rotateVector(result.rotationMatrix, layout.platformAnchors[i]), 0.001);
    return [...direction, ...vectorCross(arm, direction)];
  });
  const equilibrium = Array.from({ length: 6 }, (_, row) => columns.map(col => col[row]));
  const requiredForce = acceleration.map((a, axis) => mass * (a + (axis === 2 ? 9.81 : 0)));
  const rodForces = solve(equilibrium, [...requiredForce, 0, 0, 0]);
  if (!rodForces) return { valid: false, reason: 'Cycle force equilibrium is singular.' };
  const torque = [], speed = [];
  for (let i = 0; i < 6; i++) {
    const alpha = result.servoAngles[i], beta = layout.betaAngles[i], h = layout.hornLength / 1000;
    const tangent = [-h * Math.sin(alpha) * Math.cos(beta), -h * Math.sin(alpha) * Math.sin(beta), h * Math.cos(alpha)];
    const transmission = vectorDot(directions[i], tangent);
    if (Math.abs(transmission) < 1e-10) return { valid: false, reason: 'Cycle servo transmission is singular.' };
    torque.push(Math.abs(rodForces[i] * transmission));
    speed.push(Math.abs(vectorDot(directions[i], velocity) / transmission));
  }
  if (![...torque, ...speed].every(Number.isFinite)) return { valid: false, reason: 'Cycle demand is nonfinite.' };
  return { valid: true, torque, speed, rodForces, requiredForce, equilibrium };
}

export function computeCycleDemand(layout, { mass = 0, stroke = 0, frequency = 0, axis = 'z', ballJointLimitDeg = 52, signal } = {}) {
  const axisIndex = ['x', 'y', 'z'].indexOf(axis);
  if (axisIndex < 0) throw new RangeError('Unsupported cycle axis.');
  const amplitude = stroke / 2000;
  const omega = 2 * Math.PI * frequency;
  const samples = frequency > 0 && stroke > 0 ? 64 : 1;
  const torque = new Array(6).fill(0), speed = new Array(6).fill(0);
  for (let i = 0; i < samples; i++) {
    signal?.throwIfAborted();
    const phase = 2 * Math.PI * i / samples;
    const displacement = samples === 1 ? 0 : amplitude * Math.sin(phase);
    const pose = { [axis]: displacement * 1000 };
    const velocity = [0, 0, 0], acceleration = [0, 0, 0];
    if (samples > 1) {
      velocity[axisIndex] = omega * amplitude * Math.cos(phase);
      acceleration[axisIndex] = -omega * omega * displacement;
    }
    const result = evaluateCyclePose(layout, pose, velocity, acceleration, mass, { ballJointLimitDeg });
    if (!result.valid) return { valid: false, axis, samples, failedSample: i, reason: result.reason, torqueNm: null, speedRadPerSec: null };
    for (let leg = 0; leg < 6; leg++) {
      torque[leg] = Math.max(torque[leg], result.torque[leg]);
      speed[leg] = Math.max(speed[leg], result.speed[leg]);
    }
  }
  return { valid: true, axis, samples, torqueNm: Math.max(...torque), speedRadPerSec: Math.max(...speed),
    perServoTorqueNm: torque, perServoSpeedRadPerSec: speed,
    model: '64-phase rigid-rod force balance; centered point payload; gravity along -Z; actuator, platform and rod inertia/friction omitted' };
}
