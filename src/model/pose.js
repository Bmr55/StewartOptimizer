import { degToRad, rotationMatrixFromEuler, rotateVector, vectorAdd, vectorSub,
  vectorMagnitude, vectorNormalize, vectorDot, clamp } from '../math.js';
import { computeHornTip, hornLocalToWorld, solveServoAngle } from './kinematics.js';
import { resolveMounting } from './mounting.js';
import { assessPoseConditioning, validateConditionLimit,
  NUMERICAL_RECIPROCAL_CUTOFF } from './conditioning.js';

export function ensureLayout(layout) {
  if (!layout) throw new Error('Layout is required for workspace evaluation.');
  const { baseAnchors, platformAnchors, betaAngles, hornLength, rodLength } = layout;
  if (!Array.isArray(baseAnchors) || baseAnchors.length !== 6) {
    throw new Error('Layout must provide six base anchors.');
  }
  if (!Array.isArray(platformAnchors) || platformAnchors.length !== 6) {
    throw new Error('Layout must provide six platform anchors.');
  }
  if (!Array.isArray(betaAngles) || betaAngles.length !== 6) {
    throw new Error('Layout must provide six servo orientation angles.');
  }
  if (!Number.isFinite(hornLength) || hornLength <= 0) {
    throw new Error('Layout horn length must be a positive number.');
  }
  if (!Number.isFinite(rodLength) || rodLength <= 0) {
    throw new Error('Layout rod length must be a positive number.');
  }
  return layout;
}

export function evaluatePose(layout, pose = {}, options = {}) {
  ensureLayout(layout);
  const {
    ballJointLimitDeg = 45,
    lowerBallJointLimitDeg = ballJointLimitDeg,
    upperBallJointLimitDeg = ballJointLimitDeg,
    ballJointClamp = false,
    servoRangeRad = layout.servoRangeRad || [-Math.PI / 2, Math.PI / 2],
    rodLengthTolerance = 0.5,
    recordLegData = false,
    conditionLimit = null,
  } = options;
  validateConditionLimit(conditionLimit);
  if (!Number.isFinite(rodLengthTolerance) || rodLengthTolerance < 0) {
    throw new RangeError('rodLengthTolerance must be a finite nonnegative length in mm.');
  }
  const mounting = options.mounting ?? resolveMounting(layout).mounting;
  const jointLimits = {
    lower: degToRad(lowerBallJointLimitDeg),
    upper: degToRad(upperBallJointLimitDeg),
  };
  if (!Object.values(jointLimits).every(v => Number.isFinite(v) && v >= 0 && v <= Math.PI)) {
    throw new RangeError('Ball-joint limits must be finite angles from 0 to 180 degrees.');
  }
  const translation = [pose.x || 0, pose.y || 0, pose.z || 0];
  const rotation = [pose.rx || 0, pose.ry || 0, pose.rz || 0];
  const translated = [translation[0], translation[1], translation[2] + (layout.homeHeight || 0)];
  const rotationMatrix = rotationMatrixFromEuler(rotation[0], rotation[1], rotation[2]);

  const servoAngles = [], rodLengths = [], legDirections = [];
  const hornTips = recordLegData ? [] : null;
  const rodVectors = recordLegData ? [] : null;
  const platformPoints = recordLegData ? [] : null;
  const conditionRodVectors = [], conditionPlatformPoints = [];
  const ballJointAngles = [];
  const jointAngles = { lower: [], upper: [] };
  const violations = [];
  let structurallyReachable = true;

  for (let i = 0; i < 6; i++) {
    const base = layout.baseAnchors[i];
    const beta = layout.betaAngles[i];
    const q = vectorAdd(translated, rotateVector(rotationMatrix, layout.platformAnchors[i]));
    conditionPlatformPoints.push(q);
    if (recordLegData) platformPoints.push(q);

    const solved = solveServoAngle(base, q, layout.hornLength, layout.rodLength, beta);
    if (solved.violation) {
      violations.push({ ...solved.violation, leg: i });
      structurallyReachable = false;
      break;
    }
    const alpha = solved.alpha;
    servoAngles.push(alpha);
    if (alpha < servoRangeRad[0] - 1e-6 || alpha > servoRangeRad[1] + 1e-6) {
      violations.push({ type: 'servoLimit', leg: i, value: alpha });
      structurallyReachable = false;
      break;
    }

    const hornTip = computeHornTip(base, layout.hornLength, beta, alpha);
    const rodVector = vectorSub(q, hornTip);
    conditionRodVectors.push(rodVector);
    const rodLength = vectorMagnitude(rodVector);
    rodLengths.push(rodLength);
    if (Math.abs(rodLength - layout.rodLength) > rodLengthTolerance) {
      violations.push({ type: 'rodLength', leg: i, value: rodLength, target: layout.rodLength });
      structurallyReachable = false;
      break;
    }

    const lowerMount = mounting.lower[i]?.direction;
    const upperMount = mounting.upper[i]?.direction;
    if (!lowerMount || !upperMount) {
      violations.push({ type: 'mountingUnavailable', leg: i, joint: !lowerMount ? 'lower' : 'upper' });
      structurallyReachable = false;
      break;
    }
    const rodDirection = vectorNormalize(rodVector);
    const lowerWorld = hornLocalToWorld(beta, alpha, lowerMount);
    const upperWorld = rotateVector(rotationMatrix, upperMount);
    const lowerAngle = Math.acos(clamp(vectorDot(lowerWorld, rodDirection), -1, 1));
    const upperAngle = Math.acos(clamp(-vectorDot(upperWorld, rodDirection), -1, 1));
    jointAngles.lower.push(lowerAngle);
    jointAngles.upper.push(upperAngle);
    ballJointAngles.push(Math.max(lowerAngle, upperAngle));
    for (const [joint, angle] of [['lower', lowerAngle], ['upper', upperAngle]]) {
      if (angle > jointLimits[joint] + 1e-6) {
        violations.push({ type: 'ballJoint', leg: i, joint, value: angle, limit: jointLimits[joint] });
      }
    }

    const legDirection = vectorNormalize(vectorSub(q, base));
    legDirections.push(legDirection);
    if (recordLegData) {
      hornTips.push(hornTip);
      rodVectors.push(rodVector);
    }
  }

  const jointViolation = violations.some(v => v.type === 'ballJoint');
  const conditioning = assessPoseConditioning(conditionPlatformPoints, conditionRodVectors,
    servoAngles, layout.betaAngles, layout.hornLength, conditionLimit);
  if (structurallyReachable && conditioning.numericalSingularity) {
    violations.push({ type: 'numericalSingularity', reason: conditioning.reason,
      leg: conditioning.leg, reciprocal: conditioning.reciprocal,
      condition: conditioning.condition, threshold: NUMERICAL_RECIPROCAL_CUTOFF });
  } else if (structurallyReachable && conditioning.engineeringFailure) {
    violations.push({ type: 'conditionLimit', condition: conditioning.condition,
      reciprocal: conditioning.reciprocal, limit: conditionLimit });
  }
  const mechanicallyReachable = structurallyReachable && !jointViolation;
  return {
    reachable: mechanicallyReachable && conditioning.satisfied,
    relaxedReachable: structurallyReachable && (!jointViolation || ballJointClamp)
      && conditioning.satisfied,
    geometricallyReachable: structurallyReachable,
    mechanicallyReachable,
    violations,
    servoAngles,
    rodLengths,
    legDirections,
    jacobianRows: conditioning.jacobianRows ?? [],
    conditioning,
    hornTips,
    rodVectors,
    platformPoints,
    translation: translated,
    rotationMatrix,
    ballJointAngles,
    jointAngles,
    jointLimits,
    mounting,
  };
}
