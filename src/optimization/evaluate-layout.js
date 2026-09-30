import { computeCycleDemand } from '../model/cycle.js';
import { evaluatePose } from '../model/pose.js';
import { resolveMounting } from '../model/mounting.js';
import { validateConditionLimit, NUMERICAL_RECIPROCAL_CUTOFF } from '../model/conditioning.js';
import { computeWorkspace } from '../workspace/sweep.js';
import { failureCategories } from '../io/results.js';
import { MODEL_VERSION } from '../contracts.js';
import { clamp, degToRad } from '../math.js';

export async function evaluateLayout(layout, options) {
  const { ranges, signal, onProgress, payload, stroke, frequency, ballJointLimitDeg, ballJointClamp,
    lowerBallJointLimitDeg, upperBallJointLimitDeg, sampling, random, onPoseWork } = options;
  const conditionLimit = validateConditionLimit(options.conditionLimit);
  const mounting = resolveMounting(layout).mounting;
  layout.mounting = mounting;
  const workspaceResult = await computeWorkspace(layout, ranges, {
    signal, onProgress,
    payload, stroke, frequency, ballJointLimitDeg, lowerBallJointLimitDeg,
    upperBallJointLimitDeg, ballJointClamp, mounting, sampling, random, conditionLimit,
  });

  const coverage = Number.isFinite(workspaceResult.coverage) ? workspaceResult.coverage : 0;
  const relaxedCoverage = workspaceResult.relaxedCoverage ?? coverage;
  const stats = workspaceResult.stats || {};

  const homeResult = evaluatePose(layout, {
    x: 0,
    y: 0,
    z: 0,
    rx: 0,
    ry: 0,
    rz: 0,
  }, {
    ballJointLimitDeg, lowerBallJointLimitDeg, upperBallJointLimitDeg, ballJointClamp, mounting,
    conditionLimit,
    servoRangeRad: layout.servoRangeRad,
    recordLegData: true,
  });
  onPoseWork?.();

  const dexterity = homeResult.reachable ? homeResult.conditioning.reciprocal : null;
  const stiffness = homeResult.reachable ? homeResult.conditioning.sigmaMin : null;
  const condition = homeResult.conditioning.condition;
  const conditioningQuality = [dexterity, stats.worstReciprocal]
    .filter(Number.isFinite).reduce((worst, value) => Math.min(worst, value), Infinity);
  const availableQuality = Number.isFinite(conditioningQuality) ? conditioningQuality : null;

  const cycle = evaluateCycle(layout, { ...options, mounting, onPose: onPoseWork });
  const torque = cycle.torqueNm;
  const speedDemand = cycle.speedRadPerSec;
  const loadBalance = stats.loadBalanceScore ?? 0;
  const isotropy = stats.averageIsotropy ?? 0;
  const stiffnessScore = stats.averageStiffness > 0 ? stats.averageStiffness : stiffness;
  const marginFor = (maxAngle, limitDeg) => {
    const limit = degToRad(limitDeg ?? ballJointLimitDeg ?? 0);
    if (!Number.isFinite(maxAngle)) return 0;
    return limit > 0 ? 1 - maxAngle / limit : (maxAngle <= 1e-6 ? 1 : 0);
  };
  const ballMarginRaw = Math.min(
    marginFor(Math.max(0, ...(stats.lowerJointMax ?? [])), lowerBallJointLimitDeg),
    marginFor(Math.max(0, ...(stats.upperJointMax ?? [])), upperBallJointLimitDeg),
  );
  const violationMargin = 1 - (stats.violationRate ?? 0);
  const limitMargin = clamp(Math.max(ballMarginRaw, 0) * Math.max(violationMargin, 0), 0, 1);
  const fatigue = computeFatigue(stats, options);
  const objectives = [
    coverage,
    ballJointClamp ? relaxedCoverage : coverage,
    dexterity ?? -Infinity,
    stiffnessScore ?? -Infinity,
    loadBalance,
    isotropy,
    limitMargin,
    cycle.valid ? -torque : -Infinity,
    cycle.valid ? -speedDemand : -Infinity,
    -fatigue,
  ];

  const feasibility = {
    cycleSatisfied: cycle.valid,
    sampledWorkspaceSatisfied: coverage === 100,
    homePoseSatisfied: homeResult.reachable,
    conditionSatisfied: !homeResult.violations.some(v => ['numericalSingularity', 'conditionLimit'].includes(v.type))
      && !(stats.conditioningCounts?.numericalSingularity || stats.conditioningCounts?.engineeringLimit
        || stats.conditioningCounts?.unavailable)
      && !cycle.violations?.some(v => ['numericalSingularity', 'conditionLimit'].includes(v.type)),
    scope: 'Sampled poses under the modeled geometry, servo, rod, ball-joint and conditioning constraints',
  };
  feasibility.failedCategories = failureCategories({ feasibility, cycle });
  feasibility.passing = feasibility.failedCategories.length === 0;

  return {
    layout,
    workspace: workspaceResult,
    coverage,
    relaxedCoverage,
    cycle,
    feasibility,
    dexterity,
    stiffness: stiffnessScore,
    conditioningQuality: availableQuality,
    conditioning: {
      modelVersion: MODEL_VERSION,
      home: homeResult.conditioning,
      workspace: { worstReciprocal: stats.worstReciprocal ?? null,
        worstCondition: stats.worstCondition ?? null, counts: stats.conditioningCounts ?? null },
      cycle: cycle.conditioning ?? null,
      limit: conditionLimit,
      numericalThreshold: NUMERICAL_RECIPROCAL_CUTOFF,
    },
    torque,
    speedDemand,
    loadBalance,
    isotropy,
    limitMargin,
    fatigue,
    condition,
    objectives,
    homePose: homeResult,
    rank: Infinity,
    crowding: 0,
  };
}

export function evaluateCycle(layout, { payload, stroke, frequency, cycleAxis, ballJointLimitDeg,
  lowerBallJointLimitDeg, upperBallJointLimitDeg, conditionLimit, mounting, signal, onPose }) {
  return computeCycleDemand(layout, { mass: payload, stroke,
    frequency, axis: cycleAxis, ballJointLimitDeg, lowerBallJointLimitDeg,
    upperBallJointLimitDeg, conditionLimit, mounting, signal, onPose });
}

export function computeFatigue(stats, { ballJointLimitDeg, lowerBallJointLimitDeg,
  upperBallJointLimitDeg, servoRangeRad, stroke, frequency }) {
  if (!stats) return 0;
  const ballJointAvg = Number.isFinite(stats.ballJointAverage) ? stats.ballJointAverage : 0;
  const servoAvg = Number.isFinite(stats.servoUsageAvg) ? stats.servoUsageAvg : 0;
  const ballLimit = degToRad(Math.min(lowerBallJointLimitDeg ?? ballJointLimitDeg ?? 0,
    upperBallJointLimitDeg ?? ballJointLimitDeg ?? 0));
  const ballRatio = ballLimit > 0 ? ballJointAvg / ballLimit : 0;
  const servoSpan = Math.abs(servoRangeRad[1] - servoRangeRad[0]) || Math.PI;
  const servoDuty = servoSpan > 0 ? servoAvg / servoSpan : 0;
  const strokeMeters = stroke / 1000;
  return (Math.max(ballRatio, 0) + Math.max(servoDuty, 0)) * frequency * strokeMeters;
}
