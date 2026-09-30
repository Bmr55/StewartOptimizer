import { computeCycleDemand } from '../model/cycle.js';
import { evaluatePose } from '../model/pose.js';
import { computeWorkspace } from '../workspace/sweep.js';
import { clamp, singularValues, degToRad } from '../math.js';

const EPS = 1e-9;

export async function evaluateLayout(layout, options) {
  const { ranges, signal, onProgress, payload, stroke, frequency, ballJointLimitDeg, ballJointClamp } = options;
  const workspaceResult = await computeWorkspace(layout, ranges, {
    signal, onProgress,
    payload, stroke, frequency, ballJointLimitDeg, ballJointClamp,
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
    ballJointLimitDeg, ballJointClamp,
    servoRangeRad: layout.servoRangeRad,
    recordLegData: true,
  });

  let dexterity = 0;
  let stiffness = 0;
  let condition = Infinity;
  if (homeResult.reachable && homeResult.jacobianRows.length === 6) {
    const sv = singularValues(homeResult.jacobianRows);
    const sigmaMax = Math.max(...sv, EPS);
    const sigmaMin = sv.filter((v) => v > EPS).reduce((min, val) => Math.min(min, val), Infinity);
    if (sigmaMax > EPS && sigmaMin < Infinity) {
      dexterity = sigmaMin / sigmaMax;
      stiffness = sigmaMin;
      condition = sigmaMax / sigmaMin;
    }
  }

  const cycle = evaluateCycle(layout, options);
  const torque = cycle.torqueNm;
  const speedDemand = cycle.speedRadPerSec;
  const loadBalance = stats.loadBalanceScore ?? 0;
  const isotropy = stats.averageIsotropy ?? 0;
  const stiffnessScore = stats.averageStiffness > 0 ? stats.averageStiffness : stiffness;
  const ballLimit = degToRad(ballJointLimitDeg || 0);
  const ballMarginRaw = ballLimit > 0 && Number.isFinite(stats.ballJointOverallMax)
    ? 1 - stats.ballJointOverallMax / ballLimit
    : 1;
  const violationMargin = 1 - (stats.violationRate ?? 0);
  const limitMargin = clamp(Math.max(ballMarginRaw, 0) * Math.max(violationMargin, 0), 0, 1);
  const fatigue = computeFatigue(stats, options);
  const objectives = [
    coverage,
    ballJointClamp ? relaxedCoverage : coverage,
    dexterity,
    stiffnessScore,
    loadBalance,
    isotropy,
    limitMargin,
    cycle.valid ? -torque : -Infinity,
    cycle.valid ? -speedDemand : -Infinity,
    -fatigue,
  ];

  return {
    layout,
    workspace: workspaceResult,
    coverage,
    relaxedCoverage,
    cycle,
    feasibility: {
      cycleSatisfied: cycle.valid,
      sampledWorkspaceSatisfied: coverage === 100,
      homePoseSatisfied: homeResult.reachable,
      scope: 'Sampled poses under the modeled geometry, servo, rod and ball-joint constraints only',
    },
    dexterity,
    stiffness: stiffnessScore,
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

export function evaluateCycle(layout, { payload, stroke, frequency, cycleAxis, ballJointLimitDeg, signal }) {
  return computeCycleDemand(layout, { mass: payload, stroke,
    frequency, axis: cycleAxis, ballJointLimitDeg, signal });
}

export function computeFatigue(stats, { ballJointLimitDeg, servoRangeRad, stroke, frequency }) {
  if (!stats) return 0;
  const ballJointAvg = Number.isFinite(stats.ballJointAverage) ? stats.ballJointAverage : 0;
  const servoAvg = Number.isFinite(stats.servoUsageAvg) ? stats.servoUsageAvg : 0;
  const ballLimit = degToRad(ballJointLimitDeg || 0);
  const ballRatio = ballLimit > 0 ? ballJointAvg / ballLimit : 0;
  const servoSpan = Math.abs(servoRangeRad[1] - servoRangeRad[0]) || Math.PI;
  const servoDuty = servoSpan > 0 ? servoAvg / servoSpan : 0;
  const strokeMeters = stroke / 1000;
  return (Math.max(ballRatio, 0) + Math.max(servoDuty, 0)) * frequency * strokeMeters;
}
