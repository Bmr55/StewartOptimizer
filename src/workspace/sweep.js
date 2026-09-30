import { ensureLayout, evaluatePose } from '../model/pose.js';
import { resolveMounting } from '../model/mounting.js';
import { createWorkspaceStatistics } from './statistics.js';
import { createRandom } from '../optimization/random.js';
import { estimateWorkspaceSize, normalizeSampling, workspacePoses } from './sampling.js';

export { MAX_WORKSPACE_POSES, estimateWorkspaceSize } from './sampling.js';

export const yieldToEventLoop = () => new Promise(resolve => setTimeout(resolve, 0));

export async function computeWorkspace(layout, ranges = {}, options = {}) {
  ensureLayout(layout);
  const {
    ballJointLimitDeg = 45,
    lowerBallJointLimitDeg = ballJointLimitDeg,
    upperBallJointLimitDeg = ballJointLimitDeg,
    ballJointClamp = false,
    payload = 0,
    stroke = 0,
    frequency = 0,
    sampleLimit = 200,
    violationSampleLimit = sampleLimit,
    sampling = { strategy: 'grid' },
    random,
    onProgress,
    signal,
  } = options;

  signal?.throwIfAborted();
  const mounting = options.mounting ?? resolveMounting(layout).mounting;
  const effectiveSampling = normalizeSampling(sampling);
  const totalPoses = estimateWorkspaceSize(ranges, effectiveSampling);

  const statistics = createWorkspaceStatistics({ totalPoses, sampleLimit, violationSampleLimit,
    random: random ?? (effectiveSampling.strategy === 'halton' ? createRandom(effectiveSampling.sequenceStart) : Math.random) });

  let completed = 0;
  onProgress?.({ completed, total: totalPoses });
  await yieldToEventLoop();
  signal?.throwIfAborted();
  for (const pose of workspacePoses(ranges, effectiveSampling)) {
    const result = evaluatePose(layout, pose, {
      ballJointLimitDeg, lowerBallJointLimitDeg, upperBallJointLimitDeg, ballJointClamp,
      mounting, servoRangeRad: layout.servoRangeRad, recordLegData: false,
    });
    statistics.add(pose, result);
    completed++;
    if (completed % 256 === 0) {
      onProgress?.({ completed, total: totalPoses });
      await yieldToEventLoop();
      signal?.throwIfAborted();
    }
  }
  onProgress?.({ completed, total: totalPoses });

  return {
    ...statistics.finish(),
    constraintPolicy: { mode: ballJointClamp ? 'soft-ball-joint' : 'strict', ballJointLimitDeg,
      lowerBallJointLimitDeg, upperBallJointLimitDeg },
    sampling: effectiveSampling,
    payload, stroke, frequency,
  };
}
