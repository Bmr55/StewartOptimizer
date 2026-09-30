export const MAX_CYCLE_SAMPLES = 1024;
export const LEGACY_CYCLE_SAMPLING = Object.freeze({ strategy: 'uniform', samples: 64 });
export const DEFAULT_CYCLE_SAMPLING = Object.freeze({ strategy: 'adaptive', initialSamples: 32,
  maxSamples: 256, tolerance: 0.005, inconclusivePolicy: 'enforced' });

const integerIn = (value, min, max, name) => {
  if (!Number.isSafeInteger(value) || value < min || value > max) {
    throw new RangeError(`${name} must be an integer from ${min} to ${max}.`);
  }
  return value;
};

// `uniform` evaluates a fixed equally spaced schedule (64 reproduces the original
// model). `adaptive` evaluates initialSamples, inspects every initial interval
// midpoint, then bisects the interval with the largest estimated unresolved
// variation until all estimates are within tolerance or maxSamples is reached.
export function normalizeCycleSampling(input = DEFAULT_CYCLE_SAMPLING) {
  if (!input || typeof input !== 'object' || Array.isArray(input)) {
    throw new TypeError('cycleSampling must be an object.');
  }
  if (input.strategy === 'uniform') {
    return { strategy: 'uniform', samples: integerIn(input.samples ?? 64, 1, MAX_CYCLE_SAMPLES, 'cycleSampling.samples') };
  }
  if (input.strategy !== 'adaptive') throw new RangeError('cycleSampling.strategy must be uniform or adaptive.');
  const initialSamples = integerIn(input.initialSamples ?? DEFAULT_CYCLE_SAMPLING.initialSamples, 2,
    MAX_CYCLE_SAMPLES / 2, 'cycleSampling.initialSamples');
  const maxSamples = integerIn(input.maxSamples ?? DEFAULT_CYCLE_SAMPLING.maxSamples, 2 * initialSamples,
    MAX_CYCLE_SAMPLES, 'cycleSampling.maxSamples');
  const tolerance = input.tolerance ?? DEFAULT_CYCLE_SAMPLING.tolerance;
  if (!Number.isFinite(tolerance) || tolerance <= 0 || tolerance > 1) {
    throw new RangeError('cycleSampling.tolerance must be a finite number in (0, 1].');
  }
  const inconclusivePolicy = input.inconclusivePolicy ?? 'enforced';
  if (!['enforced', 'advisory'].includes(inconclusivePolicy)) {
    throw new RangeError('cycleSampling.inconclusivePolicy must be enforced or advisory.');
  }
  return { strategy: 'adaptive', initialSamples, maxSamples, tolerance, inconclusivePolicy };
}

export function cycleSampleBudget(policy, stationary) {
  if (stationary) return 1;
  return policy.strategy === 'uniform' ? policy.samples : policy.maxSamples;
}

// Trapezoidal weights for sorted sample times on one period [0, period): each
// sample owns half of its two adjacent intervals, including the wrap interval.
export function periodicSampleWeights(times, period) {
  const n = times.length;
  if (n === 1 || !(period > 0)) return times.map(() => 1);
  return times.map((time, i) => {
    const previous = i === 0 ? times[n - 1] - period : times[i - 1];
    const next = i === n - 1 ? times[0] + period : times[i + 1];
    return (next - previous) / (2 * period);
  });
}
