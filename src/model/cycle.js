import { evaluatePose } from './pose.js';
import { resolveMounting } from './mounting.js';
import { vectorNormalize, vectorCross, vectorDot, vectorScale, vectorAdd, vectorSub,
  rotateVector } from '../math.js';
import { GRAVITY, massPropertiesDescription, normalizeMassProperties } from './mass-properties.js';
import { isStationary, legacyTrajectory, trajectoryIdentity, trajectoryState } from './trajectory.js';
import { CYCLE_MODEL_VERSION } from '../contracts.js';

export const LEGACY_CYCLE_SAMPLES = 64;
const TRANSMISSION_CUTOFF = 1e-10;

// Partial-pivot elimination; a singular equilibrium is not a zero-demand design.
export function solveLinear(matrix, rhs) {
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

const matVec = (matrix, vector) => rotateVector(matrix, vector);
const transpose = m => m[0].map((_, j) => m.map(row => row[j]));
const matMul = (a, b) => a.map(row => b[0].map((_, j) => row.reduce((sum, value, k) => sum + value * b[k][j], 0)));

// Physical (unnormalized) rotary mapping about the moving origin, SI units.
// A_i = [u_i^T, (r_i x u_i)^T], D_ii = u_i . h'_i, alphaDot = G V with G = D^-1 A and
// V = [v; omega]. This is distinct from the dimensionless centroid conditioning Jacobian.
export function physicalMotionJacobian(layout, poseResult) {
  const h = layout.hornLength / 1000;
  const directions = poseResult.rodVectors.map(vectorNormalize);
  const arms = layout.platformAnchors.map(anchor =>
    vectorScale(rotateVector(poseResult.rotationMatrix, anchor), 0.001));
  const hornVectors = [], hornDerivatives = [], transmissions = [], A = [], rows = [];
  for (let i = 0; i < 6; i++) {
    const alpha = poseResult.servoAngles[i], beta = layout.betaAngles[i];
    const hornVector = [h * Math.cos(alpha) * Math.cos(beta), h * Math.cos(alpha) * Math.sin(beta), h * Math.sin(alpha)];
    const tangent = [-h * Math.sin(alpha) * Math.cos(beta), -h * Math.sin(alpha) * Math.sin(beta), h * Math.cos(alpha)];
    const transmission = vectorDot(directions[i], tangent);
    if (!Number.isFinite(transmission) || Math.abs(transmission) < TRANSMISSION_CUTOFF) {
      return { rows: null, leg: i, transmission, reason: 'Cycle servo transmission is singular.' };
    }
    const a = [...directions[i], ...vectorCross(arms[i], directions[i])];
    hornVectors.push(hornVector);
    hornDerivatives.push(tangent);
    transmissions.push(transmission);
    A.push(a);
    rows.push(a.map(value => value / transmission));
  }
  return { rows, A, transmissions, directions, arms, hornVectors, hornDerivatives };
}

// Newton-Euler wrench the six rods must apply to the moving body, about the moving
// origin in base-frame components: F = m (a_c - g) - F_ext, and
// M = I_w alpha + omega x (I_w omega) - M_ext + c x F.
export function requiredWrench(state, massProperties, rotationMatrix) {
  const { massKg: m, centerOfMassM, inertiaKgM2 } = massProperties;
  const c = rotateVector(rotationMatrix, centerOfMassM);
  const omega = state.omega ?? [0, 0, 0];
  const alpha = state.angularAcceleration ?? [0, 0, 0];
  const centerAcceleration = vectorAdd(vectorAdd(state.acceleration, vectorCross(alpha, c)),
    vectorCross(omega, vectorCross(omega, c)));
  const force = vectorSub(vectorScale(vectorSub(centerAcceleration, GRAVITY), m), massProperties.externalForceN);
  const worldInertia = matMul(matMul(rotationMatrix, inertiaKgM2), transpose(rotationMatrix));
  const angularMomentumRate = vectorAdd(matVec(worldInertia, alpha),
    vectorCross(omega, matVec(worldInertia, omega)));
  const moment = vectorAdd(vectorSub(angularMomentumRate, massProperties.externalMomentNm), vectorCross(c, force));
  return { force, moment, centerOfMass: c, centerAcceleration };
}

// Evaluates one trajectory state. Rod force is positive when the rod pushes the
// platform along u (horn tip to platform); signed servo torque is f_i D_ii in the
// +alpha direction, so sum(torque * alphaDot) equals F.v + M.omega.
export function evaluateDynamicPose(layout, state, massProperties, options = {}) {
  const result = evaluatePose(layout, state.pose, { ...options, ballJointClamp: false, recordLegData: true });
  if (!result.reachable) return { valid: false, reason: 'Cycle pose violates a modeled mechanical constraint.',
    violations: result.violations, conditioning: result.conditioning };
  const motion = physicalMotionJacobian(layout, result);
  if (!motion.rows) return { valid: false, reason: motion.reason, leg: motion.leg, conditioning: result.conditioning };
  const equilibrium = Array.from({ length: 6 }, (_, row) => motion.A.map(col => col[row]));
  const wrench = requiredWrench(state, massProperties, result.rotationMatrix);
  const rodForces = solveLinear(equilibrium, [...wrench.force, ...wrench.moment]);
  if (!rodForces) return { valid: false, reason: 'Cycle force equilibrium is singular.', conditioning: result.conditioning };
  const omega = state.omega ?? [0, 0, 0];
  const angular = state.angularAcceleration ?? [0, 0, 0];
  const twist = [...state.velocity, ...omega];
  const signedTorque = [], signedSpeed = [], servoAcceleration = [];
  for (let i = 0; i < 6; i++) {
    const rateValue = motion.rows[i].reduce((sum, value, k) => sum + value * twist[k], 0);
    // Second derivative of |rho|^2 = L^2 with rho = q - b - h(alpha) and h'' = -h:
    // alphaDDot = (|rhoDot|^2 + rho.qDDot + (rho.h) alphaDot^2) / (rho.h'); equals G VDot + GDot V.
    const rho = motion.directions[i];
    const arm = motion.arms[i];
    const pointVelocity = vectorAdd(state.velocity, vectorCross(omega, arm));
    const pointAcceleration = vectorAdd(vectorAdd(state.acceleration, vectorCross(angular, arm)),
      vectorCross(omega, vectorCross(omega, arm)));
    const rhoDot = vectorSub(pointVelocity, vectorScale(motion.hornDerivatives[i], rateValue));
    const rodLength = result.rodLengths[i] / 1000;
    const accelerationValue = (vectorDot(rhoDot, rhoDot) / rodLength + vectorDot(rho, pointAcceleration)
      + vectorDot(rho, motion.hornVectors[i]) * rateValue * rateValue) / motion.transmissions[i];
    signedSpeed.push(rateValue);
    servoAcceleration.push(accelerationValue);
    signedTorque.push(rodForces[i] * motion.transmissions[i]);
  }
  const values = [...signedTorque, ...signedSpeed, ...servoAcceleration];
  if (!values.every(Number.isFinite)) return { valid: false, reason: 'Cycle demand is nonfinite.' };
  return { valid: true, torque: signedTorque.map(Math.abs), speed: signedSpeed.map(Math.abs),
    signedTorque, signedSpeed, servoAcceleration, rodForces,
    requiredForce: wrench.force, requiredMoment: wrench.moment, equilibrium,
    motionJacobian: motion.rows, conditioning: result.conditioning, rotationMatrix: result.rotationMatrix };
}

// Compatibility wrapper for a translational state with a centered point mass.
export function evaluateCyclePose(layout, pose, velocity, acceleration, mass, options = {}) {
  return evaluateDynamicPose(layout, { pose, velocity, acceleration, omega: [0, 0, 0], angularAcceleration: [0, 0, 0] },
    normalizeMassProperties({ mass_kg: mass }), options);
}

export function cycleModelDescription(massProperties, samples) {
  return `${samples}-sample rigid-rod Newton-Euler force balance; ${massPropertiesDescription(massProperties)}; `
    + 'gravity along -Z; ideal massless rods/horns and joints; actuator inertia/friction omitted';
}

export function computeCycleDemand(layout, { mass = 0, stroke = 0, frequency = 0, axis = 'z',
  trajectory, massProperties, trajectorySource,
  ballJointLimitDeg = 52, lowerBallJointLimitDeg = ballJointLimitDeg,
  upperBallJointLimitDeg = ballJointLimitDeg, conditionLimit = null,
  mounting, signal, onPose } = {}) {
  const effectiveTrajectory = trajectory ?? legacyTrajectory({ stroke, frequency, axis });
  const effectiveMass = massProperties ?? normalizeMassProperties({ mass_kg: mass });
  const legacyAxis = trajectory ? null : axis;
  const stationary = isStationary(effectiveTrajectory);
  const samples = stationary ? 1 : LEGACY_CYCLE_SAMPLES;
  const period = stationary ? 0 : 1 / effectiveTrajectory.frequency_hz;
  const torque = new Array(6).fill(0), speed = new Array(6).fill(0), acceleration = new Array(6).fill(0);
  const limiting = { torque: null, speed: null, acceleration: null };
  let worstCondition = null, worstReciprocal = null;
  const effectiveMounting = mounting ?? resolveMounting(layout).mounting;
  const identity = {
    trajectory: effectiveTrajectory, trajectoryId: trajectoryIdentity(effectiveTrajectory),
    trajectorySource: trajectorySource ?? (trajectory ? 'supplied' : 'legacy-cycle'),
    massModel: effectiveMass.mode, modelVersion: CYCLE_MODEL_VERSION,
  };
  for (let i = 0; i < samples; i++) {
    signal?.throwIfAborted();
    const time = period * i / samples;
    const state = trajectoryState(effectiveTrajectory, time);
    const result = evaluateDynamicPose(layout, state, effectiveMass, {
      ballJointLimitDeg, lowerBallJointLimitDeg, upperBallJointLimitDeg,
      conditionLimit, mounting: effectiveMounting,
    });
    onPose?.();
    if (!result.valid) return { valid: false, axis: legacyAxis, samples, failedSample: i, failedTime: time,
      failedPose: state.pose, reason: result.reason, violations: result.violations ?? [],
      conditioning: { worstCondition, worstReciprocal, failedPose: result.conditioning ?? null,
        conditionLimit },
      ...identity,
      torqueNm: null, speedRadPerSec: null, accelerationRadPerSec2: null };
    const current = result.conditioning;
    worstCondition = worstCondition == null ? current.condition : Math.max(worstCondition, current.condition);
    worstReciprocal = worstReciprocal == null ? current.reciprocal
      : Math.min(worstReciprocal, current.reciprocal);
    for (let leg = 0; leg < 6; leg++) {
      for (const [key, peaks, signed] of [['torque', torque, result.signedTorque],
        ['speed', speed, result.signedSpeed], ['acceleration', acceleration, result.servoAcceleration]]) {
        const magnitude = Math.abs(signed[leg]);
        if (magnitude > peaks[leg]) peaks[leg] = magnitude;
        if (!limiting[key] || magnitude > Math.abs(limiting[key].value)) {
          limiting[key] = { servo: leg + 1, sample: i, time, value: signed[leg] };
        }
      }
    }
  }
  return { valid: true, axis: legacyAxis, samples, periodS: period,
    torqueNm: Math.max(...torque), speedRadPerSec: Math.max(...speed),
    accelerationRadPerSec2: Math.max(...acceleration),
    perServoTorqueNm: torque, perServoSpeedRadPerSec: speed, perServoAccelerationRadPerSec2: acceleration,
    limiting,
    conditioning: { worstCondition, worstReciprocal, conditionLimit },
    ...identity,
    model: cycleModelDescription(effectiveMass, samples) };
}
