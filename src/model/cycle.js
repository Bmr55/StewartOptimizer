import { evaluatePose } from './pose.js';
import { resolveMounting } from './mounting.js';
import { vectorNormalize, vectorCross, vectorDot, vectorScale, vectorAdd, vectorSub,
  rotateVector } from '../math.js';
import { GRAVITY, massPropertiesDescription, normalizeMassProperties } from './mass-properties.js';
import { isStationary, legacyTrajectory, trajectoryIdentity, trajectoryState } from './trajectory.js';
import { CYCLE_MODEL_VERSION, DEFAULT_BALL_JOINT_LIMIT_DEG } from '../contracts.js';
import { LEGACY_CYCLE_SAMPLING, normalizeCycleSampling, periodicSampleWeights } from './cycle-sampling.js';
import { actuatorTorque } from './servo-ratings.js';
import { createLoadSharingAccumulator, unavailableLoadSharing } from './load-sharing.js';

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
  if (!Array.isArray(poseResult?.rodVectors) || !Array.isArray(poseResult?.servoAngles)) {
    return { rows: null, reason: 'Pose leg data is unavailable; evaluate the pose with recordLegData: true.' };
  }
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
  return dynamicsAtPose(layout, result, state, massProperties);
}

export const staticState = pose => ({ pose, velocity: [0, 0, 0], acceleration: [0, 0, 0],
  omega: [0, 0, 0], angularAcceleration: [0, 0, 0] });

// Rod forces and servo demand for a valid pose result recorded with leg data.
export function dynamicsAtPose(layout, result, state, massProperties) {
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
    requiredForce: wrench.force, requiredMoment: wrench.moment, equilibrium, rodDirections: motion.directions,
    motionJacobian: motion.rows, conditioning: result.conditioning, rotationMatrix: result.rotationMatrix,
    servoAngles: result.servoAngles, jointAngles: result.jointAngles, jointLimits: result.jointLimits };
}

// Compatibility wrapper for a translational state with a centered point mass.
export function evaluateCyclePose(layout, pose, velocity, acceleration, mass, options = {}) {
  return evaluateDynamicPose(layout, { pose, velocity, acceleration, omega: [0, 0, 0], angularAcceleration: [0, 0, 0] },
    normalizeMassProperties({ mass_kg: mass }), options);
}

export function cycleModelDescription(massProperties, sampling, actuatorModel = 'ideal') {
  const schedule = sampling.strategy === 'adaptive' ? `adaptive ${sampling.evaluatedSamples}-sample`
    : `${sampling.evaluatedSamples}-sample`;
  const actuator = actuatorModel === 'reduced'
    ? 'reduced actuator model (reflected inertia, viscous and Coulomb friction) added to output-shaft torque'
    : 'actuator inertia/friction omitted';
  return `${schedule} rigid-rod Newton-Euler force balance; ${massPropertiesDescription(massProperties)}; `
    + `gravity along -Z; ideal massless rods/horns and joints; ${actuator}`;
}

// Quantities whose unresolved variation drives refinement: signed torque and speed,
// reciprocal conditioning, both socket angles and servo angles (31 values).
function refinementValues(result) {
  return [...result.signedTorque, ...result.signedSpeed, result.conditioning.reciprocal,
    ...result.jointAngles.lower, ...result.jointAngles.upper, ...result.servoAngles];
}

function refinementScales(samples, jointLimits, servoSpan) {
  let torque = 0, speed = 0, reciprocal = Infinity;
  for (const sample of samples) {
    for (let i = 0; i < 6; i++) {
      torque = Math.max(torque, Math.abs(sample.values[i]));
      speed = Math.max(speed, Math.abs(sample.values[6 + i]));
    }
    reciprocal = Math.min(reciprocal, sample.values[12]);
  }
  // A zero group (for example no payload) has no demand to resolve.
  const group = value => value > 1e-12 ? value : Infinity;
  return [...new Array(6).fill(group(torque)), ...new Array(6).fill(group(speed)), Math.max(reciprocal, 1e-3),
    ...new Array(6).fill(Math.max(jointLimits.lower, 1e-3)), ...new Array(6).fill(Math.max(jointLimits.upper, 1e-3)),
    ...new Array(6).fill(Math.max(servoSpan, 1e-3))];
}

const normalizedEstimate = (estimate, scales) =>
  estimate.reduce((worst, value, i) => Math.max(worst, value / scales[i]), 0);

export function computeCycleDemand(layout, { mass = 0, stroke = 0, frequency = 0, axis = 'z',
  trajectory, massProperties, trajectorySource, sampling = LEGACY_CYCLE_SAMPLING, actuators = null,
  ballJointLimitDeg = DEFAULT_BALL_JOINT_LIMIT_DEG, lowerBallJointLimitDeg = ballJointLimitDeg,
  upperBallJointLimitDeg = ballJointLimitDeg, conditionLimit = null,
  mounting, signal, onPose } = {}) {
  const effectiveTrajectory = trajectory ?? legacyTrajectory({ stroke, frequency, axis });
  const effectiveMass = massProperties ?? normalizeMassProperties({ mass_kg: mass });
  const policy = normalizeCycleSampling(sampling);
  const legacyAxis = trajectory ? null : axis;
  const stationary = isStationary(effectiveTrajectory);
  const period = stationary ? 0 : 1 / effectiveTrajectory.frequency_hz;
  const effectiveMounting = mounting ?? resolveMounting(layout).mounting;
  const servoRange = layout.servoRangeRad || [-Math.PI / 2, Math.PI / 2];
  const identity = {
    trajectory: effectiveTrajectory, trajectoryId: trajectoryIdentity(effectiveTrajectory),
    trajectorySource: trajectorySource ?? (trajectory ? 'supplied' : 'legacy-cycle'),
    massModel: effectiveMass.mode, modelVersion: CYCLE_MODEL_VERSION,
  };
  let evaluated = 0, jointLimits = null;
  let conditionTrack = { worstCondition: null, worstReciprocal: null };
  const samplingSummary = (status, extra = {}) => ({ ...policy, status, evaluatedSamples: evaluated,
    converged: status === 'converged' ? true : status === 'budget-limited' ? false : null, ...extra });
  const failure = (time, state, result) => {
    // An observed modeled violation differs from an unavailable (singular or nonfinite) calculation.
    const status = result.violations?.length ? 'violated' : 'unavailable';
    return { valid: false, axis: legacyAxis, samples: evaluated, failedSample: evaluated - 1, failedTime: time,
      failedPose: state.pose, reason: result.reason, violations: result.violations ?? [],
      conditioning: { ...conditionTrack, failedPose: result.conditioning ?? null, conditionLimit },
      sampling: samplingSummary(status), loadSharing: unavailableLoadSharing(result.reason), ...identity,
      torqueNm: null, speedRadPerSec: null, accelerationRadPerSec2: null };
  };
  const evaluateAt = time => {
    signal?.throwIfAborted();
    const state = trajectoryState(effectiveTrajectory, time);
    const result = evaluateDynamicPose(layout, state, effectiveMass, {
      ballJointLimitDeg, lowerBallJointLimitDeg, upperBallJointLimitDeg,
      conditionLimit, mounting: effectiveMounting,
    });
    evaluated++;
    onPose?.();
    if (!result.valid) return { failure: failure(time, state, result) };
    jointLimits ??= result.jointLimits;
    const current = result.conditioning;
    conditionTrack = {
      worstCondition: conditionTrack.worstCondition == null ? current.condition
        : Math.max(conditionTrack.worstCondition, current.condition),
      worstReciprocal: conditionTrack.worstReciprocal == null ? current.reciprocal
        : Math.min(conditionTrack.worstReciprocal, current.reciprocal),
    };
    return { sample: { time, result, values: refinementValues(result) } };
  };

  const samples = [];
  let status, maxUnresolved = null;
  const initial = stationary ? 1 : policy.strategy === 'uniform' ? policy.samples : policy.initialSamples;
  for (let i = 0; i < initial; i++) {
    const outcome = evaluateAt(period * i / initial);
    if (outcome.failure) return outcome.failure;
    samples.push(outcome.sample);
  }
  if (stationary) status = 'stationary';
  else if (policy.strategy === 'uniform') status = 'fixed';
  else {
    const bisect = interval => {
      const end = interval.b.time + (interval.wrap ? period : 0);
      const outcome = evaluateAt((interval.a.time + end) / 2);
      if (outcome.failure) return outcome;
      const mid = outcome.sample;
      samples.push(mid);
      // Midpoint departure from linear interpolation; each half of a smooth
      // interval is estimated to leave one quarter of it unresolved.
      const estimate = mid.values.map((value, i) =>
        Math.abs(value - (interval.a.values[i] + interval.b.values[i]) / 2) / 4);
      return { children: [{ a: interval.a, b: mid, wrap: false, estimate },
        { a: mid, b: interval.b, wrap: interval.wrap, estimate }] };
    };
    // Intervals stay in time order; the last wraps to the first sample one period later.
    // Every initial interval interior is inspected before convergence is judged.
    let intervals = [];
    for (let i = 0; i < initial; i++) {
      const outcome = bisect({ a: samples[i], b: samples[(i + 1) % initial], wrap: i === initial - 1 });
      if (outcome.failure) return outcome.failure;
      intervals.push(...outcome.children);
    }
    const servoSpan = Math.abs(servoRange[1] - servoRange[0]);
    for (;;) {
      const scales = refinementScales(samples, jointLimits, servoSpan);
      let worstIndex = 0, worst = -Infinity;
      intervals.forEach((interval, i) => {
        const value = normalizedEstimate(interval.estimate, scales);
        if (value > worst) { worst = value; worstIndex = i; }
      });
      maxUnresolved = worst;
      if (worst <= policy.tolerance) { status = 'converged'; break; }
      if (samples.length >= policy.maxSamples) { status = 'budget-limited'; break; }
      const outcome = bisect(intervals[worstIndex]);
      if (outcome.failure) return outcome.failure;
      intervals = [...intervals.slice(0, worstIndex), ...outcome.children, ...intervals.slice(worstIndex + 1)];
    }
  }

  samples.sort((a, b) => a.time - b.time);
  const torque = new Array(6).fill(0), speed = new Array(6).fill(0), acceleration = new Array(6).fill(0);
  const limiting = { torque: null, speed: null, acceleration: null };
  samples.forEach(({ time, result }, index) => {
    for (let leg = 0; leg < 6; leg++) {
      for (const [key, peaks, signed] of [['torque', torque, result.signedTorque],
        ['speed', speed, result.signedSpeed], ['acceleration', acceleration, result.servoAcceleration]]) {
        const magnitude = Math.abs(signed[leg]);
        if (magnitude > peaks[leg]) peaks[leg] = magnitude;
        if (!limiting[key] || magnitude > Math.abs(limiting[key].value)) {
          limiting[key] = { servo: leg + 1, sample: index, time, value: signed[leg] };
        }
      }
    }
  });
  // Output-shaft actuator torque: load torque plus any supplied reduced actuator model.
  const times = samples.map(sample => sample.time);
  const weights = periodicSampleWeights(times, period);
  const signedActuatorTorque = Array.from({ length: 6 }, (_, leg) => samples.map(({ result }) =>
    actuatorTorque(actuators?.[leg] ?? null, result.signedTorque[leg], result.signedSpeed[leg],
      result.servoAcceleration[leg])));
  const actuatorPeaks = signedActuatorTorque.map(values => Math.max(...values.map(Math.abs)));
  const actuatorRms = signedActuatorTorque.map(values =>
    Math.sqrt(values.reduce((sum, value, k) => sum + weights[k] * value * value, 0)));
  const actuator = {
    model: actuators?.some(Boolean) ? 'reduced' : 'ideal',
    reference: 'output shaft',
    perServoPeakTorqueNm: actuatorPeaks, perServoRmsTorqueNm: actuatorRms,
    peakTorqueNm: Math.max(...actuatorPeaks), rmsTorqueNm: Math.max(...actuatorRms),
  };
  const loadSharing = createLoadSharingAccumulator();
  samples.forEach(({ time, result }, k) => loadSharing.add(result, weights[k], time));
  const samplingResult = samplingSummary(status, { maxUnresolved });
  const output = { valid: true, axis: legacyAxis, samples: evaluated, periodS: period,
    torqueNm: Math.max(...torque), speedRadPerSec: Math.max(...speed),
    accelerationRadPerSec2: Math.max(...acceleration),
    perServoTorqueNm: torque, perServoSpeedRadPerSec: speed, perServoAccelerationRadPerSec2: acceleration,
    limiting, actuator, loadSharing: loadSharing.finish(),
    conditioning: { ...conditionTrack, conditionLimit },
    sampling: samplingResult,
    ...identity,
    model: cycleModelDescription(effectiveMass, samplingResult, actuator.model) };
  // Bounded per-sample operating points for capacity checks; not serialized with the result.
  Object.defineProperty(output, 'history', { enumerable: false, value: { times, weights, periodS: period,
    signedTorqueNm: signedActuatorTorque,
    signedSpeedRadPerSec: Array.from({ length: 6 }, (_, leg) => samples.map(({ result }) => result.signedSpeed[leg])) } });
  return output;
}
