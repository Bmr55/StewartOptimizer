export const LOAD_SHARING_MODEL = 'rod-load-sharing-v2';
export const LOAD_SHARING_METRIC = 'share = ideal / max|f| at each cycle sample, where ideal = |F| / sum|u_i . F_hat| is the '
  + 'peak rod force if all six rods carried the required force F equally in the same sense along their directions u_i; '
  + 'balance score = time-weighted mean share over samples with load (1 = equal same-sense sharing, lower when rods '
  + 'carry uneven loads or oppose each other)';
const ZERO_LOAD_N = 1e-9;

// Coefficient of variation of magnitudes; null when the mean magnitude is zero.
// Kept as a diagnostic of magnitude spread; it is blind to sign, so it is not
// the balance score (three rods in tension fighting three in compression can
// have equal magnitudes while carrying several times the load).
export function magnitudeVariation(values) {
  const magnitudes = values.map(Math.abs);
  const mean = magnitudes.reduce((sum, value) => sum + value, 0) / magnitudes.length;
  if (!(mean > ZERO_LOAD_N)) return null;
  const variance = magnitudes.reduce((sum, value) => sum + (value - mean) ** 2, 0) / magnitudes.length;
  return Math.sqrt(variance) / mean;
}

// Ratio of the ideal equal-share peak to the actual peak rod force, in [0, 1].
// `directions` are the six unit rod directions and `force` the required force
// the rods deliver together (F = sum f_i u_i). Since |F| <= sum |f_i| |u_i . F_hat|
// <= max|f| sum|u_i . F_hat|, the ideal never exceeds the actual peak, and the
// ratio is 1 only when every rod carries the same magnitude in the same sense
// along F. Rods that oppose each other, or a pure moment (|F| = 0 with loaded
// rods), lower the ratio. Null when no rod carries load.
export function shareRatio(forces, directions, force) {
  const peak = Math.max(...forces.map(Math.abs));
  if (!(peak > ZERO_LOAD_N)) return null;
  const magnitude = Math.hypot(...force);
  if (!(magnitude > ZERO_LOAD_N)) return 0;
  const unit = force.map(value => value / magnitude);
  const denominator = directions.reduce((sum, direction) =>
    sum + Math.abs(direction[0] * unit[0] + direction[1] * unit[1] + direction[2] * unit[2]), 0);
  if (!(denominator > 0)) return 0;
  return Math.min(1, magnitude / denominator / peak);
}

// Streams signed rod forces (N, positive = compression: the rod pushes the platform).
// Memory is fixed: per-rod peaks, weighted sums and the limiting samples.
export function createLoadSharingAccumulator() {
  const peakCompressionN = new Array(6).fill(0), peakTensionN = new Array(6).fill(0);
  let peakCompression = null, peakTension = null, worst = null;
  let weightedShare = 0, weightedCv = 0, loadedWeight = 0, samples = 0, zeroLoadSamples = 0;
  return {
    add({ rodForces: forces, rodDirections: directions, requiredForce }, weight, time) {
      samples++;
      forces.forEach((force, rod) => {
        if (force > peakCompressionN[rod]) peakCompressionN[rod] = force;
        if (-force > peakTensionN[rod]) peakTensionN[rod] = -force;
        if (force > 0 && (!peakCompression || force > peakCompression.forceN)) peakCompression = { rod: rod + 1, time, forceN: force };
        if (force < 0 && (!peakTension || -force > peakTension.forceN)) peakTension = { rod: rod + 1, time, forceN: -force };
      });
      const share = shareRatio(forces, directions, requiredForce);
      if (share == null) { zeroLoadSamples++; return; }
      weightedShare += weight * share;
      weightedCv += weight * magnitudeVariation(forces);
      loadedWeight += weight;
      if (!worst || share < worst.share) worst = { share, time };
    },
    finish() {
      const loaded = loadedWeight > 0;
      return {
        modelVersion: LOAD_SHARING_MODEL, metric: LOAD_SHARING_METRIC,
        // Zero load has no distribution to balance; it is not reported as perfect sharing.
        status: loaded ? 'available' : 'zero-load',
        balanceScore: loaded ? weightedShare / loadedWeight : null,
        worstShareRatio: worst?.share ?? null, worstTime: worst?.time ?? null,
        meanCv: loaded ? weightedCv / loadedWeight : null,
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
    balanceScore: null, worstShareRatio: null, meanCv: null, peakCompressionN: null, peakTensionN: null };
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
