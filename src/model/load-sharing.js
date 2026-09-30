export const LOAD_SHARING_MODEL = 'rod-load-sharing-v1';
export const LOAD_SHARING_METRIC = 'CV = std(|f|) / mean(|f|) across the six solved rod forces at each cycle sample '
  + '(population standard deviation); cycle CV is the time-weighted mean over samples with load; balance score = 1 / (1 + CV)';
const ZERO_LOAD_N = 1e-9;

// Coefficient of variation of magnitudes; null when the mean magnitude is zero.
export function magnitudeVariation(values) {
  const magnitudes = values.map(Math.abs);
  const mean = magnitudes.reduce((sum, value) => sum + value, 0) / magnitudes.length;
  if (!(mean > ZERO_LOAD_N)) return null;
  const variance = magnitudes.reduce((sum, value) => sum + (value - mean) ** 2, 0) / magnitudes.length;
  return Math.sqrt(variance) / mean;
}

// Streams signed rod forces (N, positive = compression: the rod pushes the platform).
// Memory is fixed: per-rod peaks, weighted sums and the limiting samples.
export function createLoadSharingAccumulator() {
  const peakCompressionN = new Array(6).fill(0), peakTensionN = new Array(6).fill(0);
  let peakCompression = null, peakTension = null, worst = null;
  let weightedCv = 0, loadedWeight = 0, samples = 0, zeroLoadSamples = 0;
  return {
    add(forces, weight, time) {
      samples++;
      forces.forEach((force, rod) => {
        if (force > peakCompressionN[rod]) peakCompressionN[rod] = force;
        if (-force > peakTensionN[rod]) peakTensionN[rod] = -force;
        if (force > 0 && (!peakCompression || force > peakCompression.forceN)) peakCompression = { rod: rod + 1, time, forceN: force };
        if (force < 0 && (!peakTension || -force > peakTension.forceN)) peakTension = { rod: rod + 1, time, forceN: -force };
      });
      const cv = magnitudeVariation(forces);
      if (cv == null) { zeroLoadSamples++; return; }
      weightedCv += weight * cv;
      loadedWeight += weight;
      if (!worst || cv > worst.cv) worst = { cv, time };
    },
    finish() {
      const loaded = loadedWeight > 0;
      const meanCv = loaded ? weightedCv / loadedWeight : null;
      return {
        modelVersion: LOAD_SHARING_MODEL, metric: LOAD_SHARING_METRIC,
        // Zero load has no distribution to balance; it is not reported as perfect sharing.
        status: loaded ? 'available' : 'zero-load',
        meanCv, worstCv: worst?.cv ?? null, worstTime: worst?.time ?? null,
        balanceScore: loaded ? 1 / (1 + meanCv) : null,
        samples, zeroLoadSamples,
        signConvention: 'positive force = compression (rod pushes the platform); tension is reported as a positive magnitude',
        unit: 'N',
        peakCompressionN, peakTensionN, peakCompression, peakTension,
      };
    },
  };
}

export function unavailableLoadSharing(reason) {
  return { modelVersion: LOAD_SHARING_MODEL, metric: LOAD_SHARING_METRIC, status: 'unavailable', reason,
    meanCv: null, worstCv: null, balanceScore: null, peakCompressionN: null, peakTensionN: null };
}

// Actuator utilization is separate from rod-force balance: horn leverage and ratings
// change servo torque for the same rod load.
export function actuatorUtilization(cycle, ratings) {
  const peaks = cycle?.valid ? cycle.actuator?.perServoPeakTorqueNm ?? cycle.perServoTorqueNm : null;
  if (!peaks) return { status: 'unavailable', perServoPeakTorqueNm: null, torqueCv: null,
    perServoUtilization: null, maxUtilization: null };
  const perServoUtilization = peaks.map((peak, i) => {
    const rating = ratings?.perServo?.[i]?.torqueNm;
    return rating == null ? null : peak / rating;
  });
  const rated = perServoUtilization.filter(value => value != null);
  return {
    status: !rated.length ? 'unrated' : rated.length < 6 ? 'partial' : 'rated',
    reference: 'peak output-shaft actuator torque / peak torque rating',
    perServoPeakTorqueNm: peaks,
    torqueCv: magnitudeVariation(peaks),
    perServoUtilization,
    maxUtilization: rated.length ? Math.max(...rated) : null,
  };
}
