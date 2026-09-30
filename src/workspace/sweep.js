import { degToRad, buildRange, rangeCount } from '../math.js';
import { ensureLayout, evaluatePose } from '../model/pose.js';
import { createWorkspaceStatistics } from './statistics.js';

function toRadiansRange(range) {
  if (!range) return range;
  return {
    min: degToRad(range.min || 0),
    max: degToRad(range.max || 0),
    step: degToRad(range.step || 1),
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

  const statistics = createWorkspaceStatistics({ totalPoses, sampleLimit, violationSampleLimit });

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
              statistics.add(pose, result);
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

  return {
    ...statistics.finish(),
    constraintPolicy: { mode: ballJointClamp ? 'soft-ball-joint' : 'strict', ballJointLimitDeg },
    payload, stroke, frequency,
  };
}
