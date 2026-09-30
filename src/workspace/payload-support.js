export const PAYLOAD_SUPPORT_MODEL = 'static-payload-support-v1';
const FAILING_SAMPLE_LIMIT = 50;

// `continuous` compares the constant holding torque with the continuous (RMS) rating,
// the appropriate limit for sustained support. `peak` compares with the peak/stall
// rating, which a servo may only sustain briefly. A missing rating is never substituted.
export function normalizePayloadSupport(input) {
  if (input == null) return null;
  if (typeof input !== 'object' || Array.isArray(input)) throw new TypeError('workspace_payload_support must be an object.');
  const rating = input.rating === undefined ? 'continuous' : input.rating;
  if (!['continuous', 'peak'].includes(rating)) throw new RangeError('workspace_payload_support.rating must be continuous or peak.');
  const policy = input.policy === undefined ? 'enforced' : input.policy;
  if (!['enforced', 'advisory'].includes(policy)) throw new RangeError('workspace_payload_support.policy must be enforced or advisory.');
  return { rating, policy };
}

// Per-pose outcomes: supported (every servo rated and within rating), exceeded (a rated
// servo is over), partiallyRated (rated servos within but some unrated), unrated (no
// applicable rating: demand only) and unavailable (singular or nonfinite equilibrium).
export function createPayloadSupportStatistics({ settings, ratings, loadCase }) {
  const limits = ratings.perServo.map(servo => settings.rating === 'continuous' ? servo.continuousTorqueNm : servo.torqueNm);
  const ratedCount = limits.filter(value => value != null).length;
  const counts = { evaluated: 0, supported: 0, exceeded: 0, partiallyRated: 0, unrated: 0, unavailable: 0 };
  const peakHoldingTorqueNm = new Array(6).fill(null);
  const peakPoses = new Array(6).fill(null);
  const failingSamples = [];
  let worst = null;

  function add(pose, demand) {
    counts.evaluated++;
    if (!demand.valid) {
      counts.unavailable++;
      if (failingSamples.length < FAILING_SAMPLE_LIMIT) failingSamples.push({ pose, outcome: 'unavailable', reason: demand.reason });
      return;
    }
    const exceeded = [];
    demand.torque.forEach((torque, i) => {
      if (peakHoldingTorqueNm[i] == null || torque > peakHoldingTorqueNm[i]) {
        peakHoldingTorqueNm[i] = torque;
        peakPoses[i] = pose;
      }
      if (limits[i] == null) return;
      const headroomFraction = (limits[i] - torque) / limits[i];
      if (!worst || headroomFraction < worst.headroomFraction) worst = { servo: i + 1, pose, torqueNm: torque, ratingNm: limits[i], headroomFraction };
      if (torque > limits[i]) exceeded.push({ servo: i + 1, torqueNm: torque, ratingNm: limits[i] });
    });
    if (exceeded.length) {
      counts.exceeded++;
      if (failingSamples.length < FAILING_SAMPLE_LIMIT) failingSamples.push({ pose, outcome: 'exceeded', servos: exceeded });
    } else if (ratedCount === 6) counts.supported++;
    else if (ratedCount === 0) counts.unrated++;
    else counts.partiallyRated++;
  }

  function finish(totalPoses) {
    const rated = ratedCount > 0;
    const status = !counts.evaluated ? 'notEvaluated' : counts.unavailable ? 'unavailable' : counts.exceeded ? 'exceeded'
      : !rated ? 'unrated' : counts.partiallyRated ? 'partiallyRated' : 'supported';
    return {
      modelVersion: PAYLOAD_SUPPORT_MODEL,
      scope: 'Static holding at sampled strictly feasible workspace poses; dynamic feasibility is evaluated only on the cycle trajectory.',
      loadCase, rating: settings.rating, policy: settings.policy,
      ratingSemantics: settings.rating === 'continuous'
        ? 'constant holding torque against the continuous (RMS) torque rating'
        : 'holding torque against the peak/stall torque rating; sustained holding may exceed thermal limits',
      ratedServos: ratedCount,
      status,
      counts: { ...counts, notEvaluated: totalPoses - counts.evaluated },
      // Only fully rated, supported poses qualify; null when nothing is rated.
      qualifiedCoverage: rated ? counts.supported / totalPoses * 100 : null,
      perServoRatingNm: limits,
      peakHoldingTorqueNm, peakPoses,
      perServoHeadroomFraction: limits.map((limit, i) => limit == null || peakHoldingTorqueNm[i] == null ? null
        : (limit - peakHoldingTorqueNm[i]) / limit),
      worst,
      failingSamples, failingSampleLimit: FAILING_SAMPLE_LIMIT,
    };
  }

  return { add, finish };
}

export function payloadSupportSatisfied(summary) {
  if (!summary) return true;
  // No feasible pose to hold is a workspace failure, reported by that category.
  return ['supported', 'unrated', 'notEvaluated'].includes(summary.status);
}
