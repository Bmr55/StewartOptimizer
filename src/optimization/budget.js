import { estimateWorkspaceSize } from '../workspace/sweep.js';

export function estimateWork({ ranges, sampling, stroke, frequency, populationSize, generations }) {
  const workspacePosesPerLayout = estimateWorkspaceSize(ranges, sampling);
  const cyclePosesPerLayout = stroke > 0 && frequency > 0 ? 64 : 1;
  const posesPerLayout = workspacePosesPerLayout + cyclePosesPerLayout + 1;
  const evaluations = populationSize * (generations + 1);
  const totalPoses = posesPerLayout * evaluations;
  if (!Number.isSafeInteger(totalPoses) || totalPoses > 1000000) {
    throw new RangeError('Run exceeds 1,000,000 pose evaluations. Reduce samples, population, or generations.');
  }
  return { posesPerLayout, workspacePosesPerLayout, cyclePosesPerLayout, evaluations, totalPoses };
}
