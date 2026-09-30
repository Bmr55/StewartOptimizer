import { estimateWorkspaceSize } from '../workspace/sweep.js';
import { isStationary, legacyTrajectory } from '../model/trajectory.js';
import { LEGACY_CYCLE_SAMPLES } from '../model/cycle.js';

export function estimateWork({ ranges, sampling, stroke, frequency, trajectory, populationSize, generations }) {
  const workspacePosesPerLayout = estimateWorkspaceSize(ranges, sampling);
  const effectiveTrajectory = trajectory ?? legacyTrajectory({ stroke, frequency });
  const cyclePosesPerLayout = isStationary(effectiveTrajectory) ? 1 : LEGACY_CYCLE_SAMPLES;
  const posesPerLayout = workspacePosesPerLayout + cyclePosesPerLayout + 1;
  const evaluations = populationSize * (generations + 1);
  const totalPoses = posesPerLayout * evaluations;
  if (!Number.isSafeInteger(totalPoses) || totalPoses > 1000000) {
    throw new RangeError('Run exceeds 1,000,000 pose evaluations. Reduce samples, population, or generations.');
  }
  return { posesPerLayout, workspacePosesPerLayout, cyclePosesPerLayout, evaluations, totalPoses };
}
