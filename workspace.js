import {
  degToRad,
  buildRange,
  rangeCount,
  rotationMatrixFromEuler,
  rotateVector,
  vectorAdd,
  vectorSub,
  vectorMagnitude,
  vectorNormalize,
  vectorCross,
  vectorDot,
  clamp,
  singularValues,
  average,
  standardDeviation,
} from './math.js';

const EPS = 1e-8;

function ensureLayout(layout) {
  if (!layout) {
    throw new Error('Layout is required for workspace evaluation.');
  }
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

function toRadiansRange(range) {
  if (!range) return range;
  return {
    min: degToRad(range.min || 0),
    max: degToRad(range.max || 0),
    step: degToRad(range.step || 1),
  };
}

function computeHornTip(baseAnchor, hornLength, beta, alpha) {
  const cosAlpha = Math.cos(alpha);
  const sinAlpha = Math.sin(alpha);
  const cosBeta = Math.cos(beta);
  const sinBeta = Math.sin(beta);
  return [
    baseAnchor[0] + hornLength * cosAlpha * cosBeta,
    baseAnchor[1] + hornLength * cosAlpha * sinBeta,
    baseAnchor[2] + hornLength * sinAlpha,
  ];
}

export function evaluatePose(layout, pose, options = {}) {
  ensureLayout(layout);
  const {
    ballJointLimitDeg = 45,
    ballJointClamp = false,
    servoRangeRad = layout.servoRangeRad || [-Math.PI / 2, Math.PI / 2],
    rodLengthTolerance = 0.5,
    recordLegData = false,
  } = options;

  const ballJointLimitRad = degToRad(ballJointLimitDeg);
  const translation = [pose.x || 0, pose.y || 0, pose.z || 0];
  const rotation = [pose.rx || 0, pose.ry || 0, pose.rz || 0];

  const homeOffset = layout.homeHeight || 0;
  const translated = [translation[0], translation[1], translation[2] + homeOffset];
  const rotationMatrix = rotationMatrixFromEuler(rotation[0], rotation[1], rotation[2]);

  const servoAngles = [];
  const rodLengths = [];
  const legDirections = [];
  const jacobianRows = [];
  const hornTips = recordLegData ? [] : null;
  const rodVectors = recordLegData ? [] : null;
  const platformPoints = recordLegData ? [] : null;
  const ballJointAngles = [];
  const violations = [];

  let reachable = true;

  for (let i = 0; i < 6; i++) {
    const base = layout.baseAnchors[i];
    const platformAnchor = layout.platformAnchors[i];
    const beta = layout.betaAngles[i];

    const rotatedPlatform = rotateVector(rotationMatrix, platformAnchor);
    const q = vectorAdd(translated, rotatedPlatform);
    if (recordLegData) {
      platformPoints.push(q);
    }

    const legVector = vectorSub(q, base);
    const e = 2 * layout.hornLength * legVector[2];
    const f = 2 * layout.hornLength * (Math.cos(beta) * legVector[0] + Math.sin(beta) * legVector[1]);
    const g = legVector[0] * legVector[0]
      + legVector[1] * legVector[1]
      + legVector[2] * legVector[2]
      - (layout.rodLength * layout.rodLength - layout.hornLength * layout.hornLength);

    const denom = Math.sqrt(e * e + f * f);
    if (!Number.isFinite(denom) || denom < EPS) {
      violations.push({ type: 'degenerateFourBar', leg: i, value: denom });
      reachable = false;
      break;
    }

    const ratio = g / denom;
    if (!Number.isFinite(ratio) || Math.abs(ratio) > 1 + 1e-6) {
      violations.push({ type: 'invalidGeometry', leg: i, value: ratio });
      reachable = false;
      break;
    }

    const clampedRatio = clamp(ratio, -1, 1);
    const alpha = Math.asin(clampedRatio) - Math.atan2(f, e);
    servoAngles.push(alpha);

    if (alpha < servoRangeRad[0] - 1e-6 || alpha > servoRangeRad[1] + 1e-6) {
      violations.push({ type: 'servoLimit', leg: i, value: alpha });
      reachable = false;
      break;
    }

    const hornTip = computeHornTip(base, layout.hornLength, beta, alpha);
    const rodVector = vectorSub(q, hornTip);
    const rodLength = vectorMagnitude(rodVector);
    rodLengths.push(rodLength);

    if (Math.abs(rodLength - layout.rodLength) > rodLengthTolerance) {
      violations.push({ type: 'rodLength', leg: i, value: rodLength, target: layout.rodLength });
      reachable = false;
      break;
    }

    let servoAxis = vectorNormalize(vectorSub(hornTip, base));
    if (servoAxis[0] === 0 && servoAxis[1] === 0 && servoAxis[2] === 0) {
      servoAxis = [0, 0, 1];
    }
    const rodDirection = vectorNormalize(rodVector);
    const jointAngle = Math.acos(clamp(vectorDot(servoAxis, rodDirection), -1, 1));
    ballJointAngles.push(jointAngle);

    if (jointAngle > ballJointLimitRad + 1e-6) {
      violations.push({ type: 'ballJoint', leg: i, value: jointAngle, limit: ballJointLimitRad });
      if (!ballJointClamp) {
        reachable = false;
        break;
      }
    }

    const legDirection = vectorNormalize(legVector);
    legDirections.push(legDirection);
    const moment = vectorCross(q, legDirection);
    jacobianRows.push([...legDirection, ...moment]);

    if (recordLegData) {
      hornTips.push(hornTip);
      rodVectors.push(rodVector);
    }
  }

  return {
    reachable: reachable && violations.length === 0,
    relaxedReachable: reachable,
    violations,
    servoAngles,
    rodLengths,
    legDirections,
    jacobianRows,
    hornTips,
    rodVectors,
    platformPoints,
    translation: translated,
    rotationMatrix,
    ballJointAngles,
  };
}

export const MAX_WORKSPACE_POSES = 100000;
export function estimateWorkspaceSize(ranges = {}) {
  let total = 1;
  for (const axis of ['x', 'y', 'z', 'rx', 'ry', 'rz']) {
    total *= rangeCount(ranges[axis]);
    if (!Number.isSafeInteger(total) || total > MAX_WORKSPACE_POSES) {
      throw new RangeError('Workspace exceeds 100,000 poses per layout. Increase sweep steps or narrow ranges.');
    }
  }
  return total;
}

export const yieldToEventLoop = () => new Promise(resolve => setTimeout(resolve, 0));

function runningMean() {
  let count = 0;
  let total = 0;
  return { add(value) { total += value; count++; }, value() { return count ? total / count : 0; } };
}

export async function computeWorkspace(layout, ranges = {}, options = {}) {
  ensureLayout(layout);
  const {
    ballJointLimitDeg = 45,
    ballJointClamp = false,
    payload = 0,
    stroke = 0,
    frequency = 0,
    sampleLimit = 200,
    violationSampleLimit = sampleLimit,
    onProgress,
    signal,
  } = options;

  signal?.throwIfAborted();
  const totalPoses = estimateWorkspaceSize(ranges);
  const xs = buildRange(ranges.x, 0);
  const ys = buildRange(ranges.y, 0);
  const zs = buildRange(ranges.z, 0);
  const rxs = buildRange(toRadiansRange(ranges.rx), 0);
  const rys = buildRange(toRadiansRange(ranges.ry), 0);
  const rzs = buildRange(toRadiansRange(ranges.rz), 0);

  const normalizedSampleLimit = Math.max(0, Math.floor(sampleLimit));
  const normalizedViolationSampleLimit = Math.max(0, Math.floor(violationSampleLimit));
  const reachableSamples = [];
  const unreachableSamples = [];
  const violationSamples = [];
  const violationCounts = {};
  const isotropySamples = runningMean();
  const stiffnessSamples = runningMean();
  const loadShareSamples = runningMean();
  const ballJointSamples = runningMean();
  const servoRanges = Array.from({ length: 6 }, () => ({ min: Infinity, max: -Infinity }));
  const ballJointMax = new Array(6).fill(0);

  const recordSample = (collection, limit, seenCount, sample) => {
    if (limit <= 0) return;
    if (collection.length < limit) {
      collection.push(sample);
    } else {
      const replaceIndex = Math.floor(Math.random() * seenCount);
      if (replaceIndex < limit) {
        collection[replaceIndex] = sample;
      }
    }
  };

  let relaxedReachableCount = 0;
  let reachableCount = 0;
  let unreachableCount = 0;
  let violationPoseCount = 0;
  let reachableSeen = 0;
  let unreachableSeen = 0;
  let violationSeen = 0;

  let completed = 0;
  onProgress?.({ completed, total: totalPoses });
  await yieldToEventLoop();
  signal?.throwIfAborted();
  for (const x of xs) {
    for (const y of ys) {
      for (const z of zs) {
        for (const rx of rxs) {
          for (const ry of rys) {
            for (const rz of rzs) {
              const pose = { x, y, z, rx, ry, rz };
              const result = evaluatePose(layout, pose, {
                ballJointLimitDeg,
                ballJointClamp,
                servoRangeRad: layout.servoRangeRad,
                recordLegData: false,
              });
              const hasViolations = Array.isArray(result.violations) && result.violations.length > 0;

              if (result.relaxedReachable) relaxedReachableCount += 1;
              if (result.reachable) {
                reachableCount += 1;
                reachableSeen += 1;
                recordSample(reachableSamples, normalizedSampleLimit, reachableSeen, { pose });

                if (result.jacobianRows.length === 6) {
                  const sv = singularValues(result.jacobianRows);
                  if (sv.length) {
                    const sigmaMax = Math.max(...sv);
                    const sigmaMin = Math.min(...sv);
                    if (Number.isFinite(sigmaMax) && Number.isFinite(sigmaMin) && sigmaMax > EPS && sigmaMin > EPS) {
                      isotropySamples.add(sigmaMin / sigmaMax);
                      stiffnessSamples.add(sigmaMin);
                    }
                  }
                }

                if (result.legDirections.length === 6) {
                  const shares = result.legDirections.map((dir) => Math.abs(dir[2]));
                  const sumShares = shares.reduce((acc, value) => acc + value, 0) || 1;
                  const normalized = shares.map((value) => value / sumShares);
                  const loadStd = standardDeviation(normalized);
                  const loadScore = 1 / (1 + loadStd);
                  loadShareSamples.add(loadScore);
                }

                if (Array.isArray(result.servoAngles)) {
                  for (let iLeg = 0; iLeg < Math.min(6, result.servoAngles.length); iLeg++) {
                    const angle = result.servoAngles[iLeg];
                    const servo = servoRanges[iLeg];
                    if (angle < servo.min) servo.min = angle;
                    if (angle > servo.max) servo.max = angle;
                  }
                }

                if (Array.isArray(result.ballJointAngles)) {
                  const maxAngle = Math.max(...result.ballJointAngles.map((value) => (Number.isFinite(value) ? value : 0)), 0);
                  ballJointSamples.add(maxAngle);
                  for (let iLeg = 0; iLeg < Math.min(6, result.ballJointAngles.length); iLeg++) {
                    const angle = result.ballJointAngles[iLeg];
                    if (Number.isFinite(angle) && angle > ballJointMax[iLeg]) {
                      ballJointMax[iLeg] = angle;
                    }
                  }
                }
              } else {
                unreachableCount += 1;
                unreachableSeen += 1;
                recordSample(unreachableSamples, normalizedSampleLimit, unreachableSeen, { pose });
              }

              if (hasViolations) {
                violationPoseCount += 1;
                violationSeen += 1;
                recordSample(
                  violationSamples,
                  normalizedViolationSampleLimit,
                  violationSeen,
                  { pose, violations: result.violations.map((violation) => ({ ...violation })) },
                );
                for (const violation of result.violations) {
                  violationCounts[violation.type] = (violationCounts[violation.type] || 0) + 1;
                }
              }
              completed++;
              if (completed % 256 === 0) {
                onProgress?.({ completed, total: totalPoses });
                await yieldToEventLoop();
                signal?.throwIfAborted();
              }
            }
          }
        }
      }
    }
  }
  onProgress?.({ completed, total: totalPoses });

  const coverage = (reachableCount / totalPoses) * 100;
  const relaxedCoverage = (relaxedReachableCount / totalPoses) * 100;

  const servoUsage = servoRanges.map((range) => {
    if (range.min === Infinity || range.max === -Infinity) return 0;
    return range.max - range.min;
  });
  const servoUsageAvg = average(servoUsage);
  const servoUsagePeak = Math.max(0, ...servoUsage);

  const violationRate = totalPoses > 0 ? violationPoseCount / totalPoses : 0;

  const stats = {
    reachableCount,
    averageIsotropy: isotropySamples.value(),
    averageStiffness: stiffnessSamples.value(),
    loadBalanceScore: loadShareSamples.value(),
    servoUsage,
    servoUsageAvg,
    servoUsagePeak,
    ballJointMax,
    ballJointOverallMax: Math.max(0, ...ballJointMax),
    ballJointAverage: ballJointSamples.value(),
    violationCounts,
    violationRate,
  };

  return {
    coverage,
    relaxedCoverage,
    constraintPolicy: { mode: ballJointClamp ? 'soft-ball-joint' : 'strict', ballJointLimitDeg },
    total: totalPoses,
    reachable: reachableSamples,
    unreachable: unreachableSamples,
    violations: violationSamples,
    payload,
    stroke,
    frequency,
    stats,
    counts: {
      relaxedReachable: relaxedReachableCount,
      reachable: reachableCount,
      unreachable: unreachableCount,
      violationPoses: violationPoseCount,
    },
    samples: {
      reachable: reachableSamples,
      unreachable: unreachableSamples,
      violations: violationSamples,
      limits: {
        reachable: normalizedSampleLimit,
        unreachable: normalizedSampleLimit,
        violations: normalizedViolationSampleLimit,
      },
    },
  };
}
