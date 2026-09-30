import { degToRad, buildRange, rangeCount } from '../math.js';

export const MAX_WORKSPACE_POSES = 100000;
export const HALTON_BASES = Object.freeze([2, 3, 5, 7, 11, 13]);
export const DEFAULT_SAMPLE_COUNT = 1024;
export const SAMPLE_PRESETS = Object.freeze([256, DEFAULT_SAMPLE_COUNT, 4096]);
const AXES = ['x', 'y', 'z', 'rx', 'ry', 'rz'];

function toRadiansRange(range) {
  if (!range) return range;
  return {
    min: degToRad(range.min || 0),
    max: degToRad(range.max || 0),
    step: degToRad(range.step || 1),
  };
}

export function normalizeSampling(sampling = { strategy: 'grid' }) {
  const strategy = sampling?.strategy ?? 'grid';
  if (strategy === 'grid') return { strategy: 'grid' };
  if (strategy !== 'halton') throw new RangeError('Sampling strategy must be halton or grid.');
  const sampleCount = sampling.sampleCount ?? DEFAULT_SAMPLE_COUNT;
  const sequenceStart = sampling.sequenceStart ?? 1;
  if (!Number.isSafeInteger(sampleCount) || sampleCount < 1 || sampleCount > MAX_WORKSPACE_POSES) {
    throw new RangeError('Halton sample count must be an integer from 1 to 100,000.');
  }
  if (!Number.isSafeInteger(sequenceStart) || sequenceStart < 1 || sequenceStart + sampleCount - 1 > Number.MAX_SAFE_INTEGER) {
    throw new RangeError('Halton sequence start must be a positive safe integer.');
  }
  if (sampling.sequence && sampling.sequence !== 'halton-v1') throw new RangeError('Unsupported Halton sequence.');
  if (sampling.bases && JSON.stringify(sampling.bases) !== JSON.stringify(HALTON_BASES)) {
    throw new RangeError('Unsupported Halton bases.');
  }
  return { strategy, sampleCount, sequence: 'halton-v1', sequenceStart, bases: [...HALTON_BASES] };
}

export function estimateWorkspaceSize(ranges = {}, sampling = { strategy: 'grid' }) {
  const effective = normalizeSampling(sampling);
  let total = 1;
  for (const axis of AXES) {
    const range = ranges[axis];
    if (effective.strategy === 'grid') {
      total *= rangeCount(range);
      if (!Number.isSafeInteger(total) || total > MAX_WORKSPACE_POSES) {
        throw new RangeError('Workspace exceeds 100,000 poses per layout. Increase sweep steps or narrow ranges.');
      }
    } else if (range) {
      const { min, max, step } = range;
      if (![min, max].every(Number.isFinite) || max < min || (step !== undefined && (!Number.isFinite(step) || step <= 0))) {
        throw new RangeError('Sweep ranges require finite min/max, max >= min and a positive step.');
      }
    }
  }
  return effective.strategy === 'halton' ? effective.sampleCount : total;
}

export function radicalInverse(index, base) {
  let n = index;
  let inverse = 0;
  let factor = 1 / base;
  while (n > 0) {
    inverse += (n % base) * factor;
    n = Math.floor(n / base);
    factor /= base;
  }
  return inverse;
}

export function haltonPose(ranges = {}, index) {
  if (!Number.isSafeInteger(index) || index < 1) throw new RangeError('Halton index must be a positive safe integer.');
  const pose = {};
  for (let dimension = 0; dimension < AXES.length; dimension++) {
    const axis = AXES[dimension];
    const range = ranges[axis];
    const min = range?.min ?? 0;
    const max = range?.max ?? 0;
    const degrees = dimension >= 3;
    const value = min === max ? min : min + radicalInverse(index, HALTON_BASES[dimension]) * (max - min);
    pose[axis] = degrees ? degToRad(value) : value;
  }
  return pose;
}

export function* workspacePoses(ranges = {}, sampling = { strategy: 'grid' }) {
  const effective = normalizeSampling(sampling);
  if (effective.strategy === 'halton') {
    for (let index = effective.sequenceStart; index < effective.sequenceStart + effective.sampleCount; index++) {
      yield haltonPose(ranges, index);
    }
    return;
  }
  const values = AXES.map((axis, index) => buildRange(index >= 3 ? toRadiansRange(ranges[axis]) : ranges[axis], 0));
  for (const x of values[0]) for (const y of values[1]) for (const z of values[2]) {
    for (const rx of values[3]) for (const ry of values[4]) for (const rz of values[5]) {
      yield { x, y, z, rx, ry, rz };
    }
  }
}
