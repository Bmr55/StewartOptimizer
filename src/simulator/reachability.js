import { evaluatePose } from '../model/pose.js';
import { DEFAULT_SAMPLE_COUNT, SAMPLE_PRESETS, workspacePoses } from '../workspace/sampling.js';

// The reachability cloud sweeps platform-origin translations at one fixed
// orientation through the shared pose evaluator. It reports evaluated samples
// only: a point is reachable or not, and nothing between points is claimed.

export const REACHABILITY_MODES = Object.freeze(['cloud', 'slice']);
export const REACHABILITY_SAMPLE_COUNTS = SAMPLE_PRESETS;
// Settings the cloud starts with and that a saved file may leave out. Whether
// it is drawn (and swept) is the `reachabilityCloud` overlay toggle.
export const REACHABILITY_DEFAULTS = Object.freeze({ sampleCount: DEFAULT_SAMPLE_COUNT, mode: 'cloud', sliceZ: 0 });
// Poses evaluated between yields to the event loop.
export const REACHABILITY_CHUNK = 200;
// Half-width (mm) of an X/Y/Z axis the requirement ranges do not give.
export const DEFAULT_REACHABILITY_HALF_RANGE_MM = 100;

// Reads cloud settings from the UI or saved JSON: known keys only, each
// checked, so a hand-edited file cannot start a 100,000-pose sweep.
export function parseReachability(value, field = 'reachability') {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new TypeError(`${field} must be an object.`);
  const result = {};
  if (value.sampleCount !== undefined) {
    if (!REACHABILITY_SAMPLE_COUNTS.includes(value.sampleCount)) {
      throw new RangeError(`${field}.sampleCount must be one of ${REACHABILITY_SAMPLE_COUNTS.join(', ')}.`);
    }
    result.sampleCount = value.sampleCount;
  }
  if (value.mode !== undefined) {
    if (!REACHABILITY_MODES.includes(value.mode)) throw new RangeError(`${field}.mode must be one of ${REACHABILITY_MODES.join(', ')}.`);
    result.mode = value.mode;
  }
  if (value.sliceZ !== undefined) {
    if (typeof value.sliceZ !== 'number' || !Number.isFinite(value.sliceZ)) {
      throw new RangeError(`${field}.sliceZ must be a finite number of millimetres.`);
    }
    result.sliceZ = value.sliceZ;
  }
  return result;
}

// The X/Y/Z box sampled about home (mm): the requirement range for each axis
// that has one, else ±DEFAULT_REACHABILITY_HALF_RANGE_MM. Slice mode pins Z to
// `sliceZ`, so every sample lies on that plane.
export function reachabilityRanges(workspaceRanges, { mode, sliceZ }) {
  const fallback = { min: -DEFAULT_REACHABILITY_HALF_RANGE_MM, max: DEFAULT_REACHABILITY_HALF_RANGE_MM };
  const range = axis => {
    const given = workspaceRanges?.[axis];
    return given ? { min: given.min, max: given.max } : { ...fallback };
  };
  return { x: range('x'), y: range('y'), z: mode === 'slice' ? { min: sliceZ, max: sliceZ } : range('z') };
}

// Halton translations in those ranges (the same `halton-v1` sequence as the
// optimizer's workspace sampling) at the given orientation in radians.
export function* reachabilityPoses(workspaceRanges, orientation, settings) {
  const sampling = { strategy: 'halton', sampleCount: settings.sampleCount };
  for (const { x, y, z } of workspacePoses(reachabilityRanges(workspaceRanges, settings), sampling)) {
    yield { x, y, z, rx: orientation.rx, ry: orientation.ry, rz: orientation.rz };
  }
}

// Evaluates every pose in chunks of REACHABILITY_CHUNK, awaiting `yieldControl`
// before each chunk, and calls `onChunk(points)` with the new points after it.
// Each point is the evaluator's platform-origin `translation` (world mm) and
// its `reachable` flag. Stops quietly once `signal` aborts. `evaluate` is
// `evaluatePose` outside tests.
export async function sweepReachability({ layout, options = {}, workspaceRanges, orientation, settings, signal,
  yieldControl, onChunk, evaluate = evaluatePose }) {
  const poses = reachabilityPoses(workspaceRanges, orientation, settings);
  let remaining = settings.sampleCount;
  while (remaining > 0) {
    await yieldControl();
    if (signal?.aborted) return false;
    const points = [];
    for (let k = Math.min(REACHABILITY_CHUNK, remaining); k > 0; k--) {
      const assessment = evaluate(layout, poses.next().value, options);
      points.push({ at: assessment.translation.slice(), reachable: Boolean(assessment.reachable) });
    }
    remaining -= points.length;
    onChunk(points);
  }
  return true;
}
