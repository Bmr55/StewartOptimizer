import { estimateWorkspaceSize } from '../workspace/sweep.js';
import { isStationary, legacyTrajectory } from '../model/trajectory.js';
import { cycleSampleBudget, LEGACY_CYCLE_SAMPLING, normalizeCycleSampling } from '../model/cycle-sampling.js';

export function estimateWork({ ranges, sampling, stroke, frequency, trajectory,
  cycleSampling = LEGACY_CYCLE_SAMPLING, payloadSupport = null, populationSize, generations }) {
  const workspacePosesPerLayout = estimateWorkspaceSize(ranges, sampling);
  const effectiveTrajectory = trajectory ?? legacyTrajectory({ stroke, frequency });
  const cyclePosesPerLayout = cycleSampleBudget(normalizeCycleSampling(cycleSampling), isStationary(effectiveTrajectory));
  // A static payload check may follow every workspace pose; it counts against the same limit.
  const payloadChecksPerLayout = payloadSupport ? workspacePosesPerLayout : 0;
  const posesPerLayout = workspacePosesPerLayout + payloadChecksPerLayout + cyclePosesPerLayout + 1;
  const evaluations = populationSize * (generations + 1);
  const totalPoses = posesPerLayout * evaluations;
  if (!Number.isSafeInteger(totalPoses) || totalPoses > 1000000) {
    throw new RangeError('Run exceeds 1,000,000 pose evaluations. Reduce samples, population, or generations.');
  }
  return { posesPerLayout, workspacePosesPerLayout, payloadChecksPerLayout, cyclePosesPerLayout, evaluations, totalPoses };
}
