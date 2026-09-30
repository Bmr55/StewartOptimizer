import test from 'node:test';
import assert from 'node:assert/strict';
import { createWorkspaceStatistics } from '../../src/workspace/statistics.js';

const close = (actual, expected, tolerance = 1e-12) =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} vs ${expected}`);
const closeArray = (actual, expected, tolerance = 1e-12) => {
  assert.equal(actual.length, expected.length);
  expected.forEach((value, i) => close(actual[i], value, tolerance));
};

// Hand-built evaluatePose-shaped results: only the fields the aggregator reads.
const conditioning = (reciprocal, condition, sigmaMin, extra = {}) => ({
  available: true, satisfied: true, numericalSingularity: false, engineeringFailure: false,
  reciprocal, condition, sigmaMin, ...extra,
});
const legs = z => z.map(value => [0.3, -0.4, value]);
const result = ({ lower, upper, z, servoAngles, reachable, relaxedReachable = reachable,
  geometricallyReachable = true, violations = [], conditioning: cond }) => ({
  reachable, relaxedReachable, geometricallyReachable, violations, conditioning: cond,
  servoAngles, legDirections: legs(z), jointAngles: { lower, upper },
  ballJointAngles: lower.map((value, i) => Math.max(value, upper[i])),
});

// Three reachable poses, one soft ball-joint failure, one geometry failure, one
// numerical singularity and one engineering condition failure; the sweep is
// declared as ten poses so every ratio has a denominator distinct from each count.
const reachable = [
  result({ reachable: true, lower: [0.10, 0.20, 0.30, 0.05, 0.15, 0.25], upper: [0.30, 0.10, 0.20, 0.35, 0.05, 0.15],
    z: [0.5, 0.5, -0.5, 0.5, 0.5, -0.5], servoAngles: [0.1, -0.2, 0.3, 0.0, 0.5, -0.1],
    conditioning: conditioning(0.5, 2, 0.8) }),
  result({ reachable: true, lower: [0.40, 0.10, 0.20, 0.10, 0.50, 0.10], upper: [0.20, 0.45, 0.10, 0.20, 0.10, 0.30],
    z: [1, 0, 0, 0, 0, 0], servoAngles: [0.4, -0.5, 0.1, 0.2, -0.5, 0.3],
    conditioning: conditioning(0.2, 5, 0.4) }),
  result({ reachable: true, lower: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05], upper: [0.10, 0.10, 0.10, 0.10, 0.10, 0.60],
    z: [0.2, 0.2, 0.2, 0.4, 0.4, 0.4], servoAngles: [-0.2, 0.1, 0.3, -0.3, 0.0, 0.6],
    conditioning: conditioning(0.4, 2.5, 0.6) }),
];
const unreachable = [
  // Soft joint failure: relaxed-reachable, two socket violations on one pose.
  result({ reachable: false, relaxedReachable: true,
    lower: [0.20, 0.30, 0.10, 0.50, 0.20, 0.10], upper: [0.10, 0.20, 0.90, 0.10, 0.60, 0.20],
    z: [1, 1, 1, 1, 1, 1], servoAngles: [2, 2, 2, 2, 2, 2], conditioning: conditioning(0.3, 3.3, 0.5),
    violations: [{ type: 'ballJoint', leg: 2, joint: 'upper', value: 0.9, limit: 0.7 },
      { type: 'ballJoint', leg: 3, joint: 'lower', value: 0.5, limit: 0.45 }] }),
  // Structural failure at leg 0: no leg data at all.
  { reachable: false, relaxedReachable: false, geometricallyReachable: false,
    violations: [{ type: 'invalidGeometry', leg: 0 }], conditioning: { available: false, satisfied: false },
    servoAngles: [], legDirections: [], jointAngles: { lower: [], upper: [] }, ballJointAngles: [] },
  result({ reachable: false, lower: new Array(6).fill(0.01), upper: new Array(6).fill(0.02),
    z: [1, 1, 1, 1, 1, 1], servoAngles: [-3, -3, -3, -3, -3, -3],
    conditioning: conditioning(1e-9, 1e9, 1e-9, { satisfied: false, numericalSingularity: true }),
    violations: [{ type: 'numericalSingularity', leg: 4 }] }),
  result({ reachable: false, lower: new Array(6).fill(0.03), upper: new Array(6).fill(0.01),
    z: [1, 1, 1, 1, 1, 1], servoAngles: [3, 3, 3, 3, 3, 3],
    conditioning: conditioning(0.01, 100, 0.02, { satisfied: false, engineeringFailure: true }),
    violations: [{ type: 'conditionLimit', condition: 100, limit: 50 }] }),
];
const poses = [...reachable, ...unreachable].map((_, i) => ({ x: i, y: 0, z: 0, rx: 0, ry: 0, rz: 0 }));

function aggregate({ sampleLimit, violationSampleLimit, randomValues = [] }) {
  const queue = [...randomValues];
  let draws = 0;
  const random = () => { draws++; return queue.shift() ?? 0; };
  const statistics = createWorkspaceStatistics({ totalPoses: 10, sampleLimit, violationSampleLimit, random });
  [...reachable, ...unreachable].forEach((entry, i) => statistics.add(poses[i], entry));
  return { workspace: statistics.finish(), draws: () => draws };
}

test('every workspace statistic is aggregated from hand-built pose results', () => {
  const { workspace } = aggregate({ sampleLimit: 200 });
  const { stats } = workspace;
  assert.equal(workspace.total, 10);
  assert.equal(workspace.coverage, 30);
  assert.equal(workspace.relaxedCoverage, 40);
  assert.deepEqual(workspace.counts, { relaxedReachable: 4, reachable: 3, unreachable: 4, violationPoses: 4 });
  assert.equal(stats.reachableCount, 3);
  // Poses with any violation over all sampled poses, not leg failures over anything else.
  assert.equal(stats.violationRate, 0.4);
  assert.deepEqual(stats.violationCounts, { ballJoint: 2, invalidGeometry: 1, numericalSingularity: 1, conditionLimit: 1 });
  assert.deepEqual(stats.jointViolationCounts, { lower: 1, upper: 1 });
  assert.deepEqual(stats.conditioningCounts, { valid: 4, numericalSingularity: 1, engineeringLimit: 1, unavailable: 0 });

  // Conditioning averages and extremes use the strictly reachable poses only.
  close(stats.averageIsotropy, (0.5 + 0.2 + 0.4) / 3);
  close(stats.averageStiffness, (0.8 + 0.4 + 0.6) / 3);
  assert.equal(stats.worstReciprocal, 0.2);
  assert.equal(stats.worstCondition, 5);

  // Load balance: 1 / (1 + population std of the |z| shares) averaged over reachable poses.
  // Equal shares score 1; one leg carrying everything has std sqrt(5)/6; shares of
  // 1/9 and 2/9 have std 1/18. Negative z components count by magnitude.
  close(stats.loadBalanceScore, (1 + 1 / (1 + Math.sqrt(5) / 6) + 18 / 19) / 3);

  // Servo usage is the travelled span (max - min) per servo over reachable poses;
  // the unreachable poses at +-2 and +-3 rad do not widen it.
  closeArray(stats.servoUsage, [0.6, 0.6, 0.2, 0.5, 1.0, 0.7]);
  close(stats.servoUsageAvg, 0.6);
  close(stats.servoUsagePeak, 1.0);

  // Socket maxima over every pose that produced leg data versus reachable poses only.
  assert.deepEqual(stats.ballJointMax, [0.40, 0.45, 0.90, 0.50, 0.60, 0.60]);
  assert.deepEqual(stats.lowerJointMax, [0.40, 0.30, 0.30, 0.50, 0.50, 0.25]);
  assert.deepEqual(stats.upperJointMax, [0.30, 0.45, 0.90, 0.35, 0.60, 0.60]);
  assert.deepEqual(stats.reachableLowerJointMax, [0.40, 0.20, 0.30, 0.10, 0.50, 0.25]);
  assert.deepEqual(stats.reachableUpperJointMax, [0.30, 0.45, 0.20, 0.35, 0.10, 0.60]);
  assert.equal(stats.ballJointOverallMax, 0.90);
  // Mean over poses of each pose's worst socket: (0.35 + 0.50 + 0.60 + 0.90 + 0.02 + 0.03) / 6.
  close(stats.ballJointAverage, 0.4);

  assert.deepEqual(Object.keys(stats).sort(), ['averageIsotropy', 'averageStiffness', 'ballJointAverage',
    'ballJointMax', 'ballJointOverallMax', 'conditioningCounts', 'jointViolationCounts', 'loadBalanceScore',
    'lowerJointMax', 'reachableCount', 'reachableLowerJointMax', 'reachableUpperJointMax', 'servoUsage',
    'servoUsageAvg', 'servoUsagePeak', 'upperJointMax', 'violationCounts', 'violationRate', 'worstCondition',
    'worstReciprocal']);

  // Under the cap every pose is retained, in order, with copied violations.
  assert.deepEqual(workspace.samples.limits, { reachable: 200, unreachable: 200, violations: 200 });
  assert.deepEqual(workspace.reachable, poses.slice(0, 3).map(pose => ({ pose })));
  assert.deepEqual(workspace.unreachable, poses.slice(3).map(pose => ({ pose })));
  assert.deepEqual(workspace.violations.map(sample => sample.pose), poses.slice(3));
  assert.deepEqual(workspace.violations[0].violations, unreachable[0].violations);
  assert.notEqual(workspace.violations[0].violations[0], unreachable[0].violations[0]);
  assert.equal(workspace.samples.reachable, workspace.reachable);
  assert.equal(workspace.samples.unreachable, workspace.unreachable);
  assert.equal(workspace.samples.violations, workspace.violations);
});

test('reservoir sampling replaces floor(random * seen) and skips indices at or beyond the limit', () => {
  // Reachable: the third pose draws 0.4 -> index 1 of 3. Unreachable (limit 2): the
  // third draws 0.9 -> index 2, which is not written; the fourth draws 0.1 -> index 0.
  // Violations (limit 3): the fourth draws 0.7 -> index 2. Filling never draws.
  const { workspace, draws } = aggregate({ sampleLimit: 2.9, violationSampleLimit: 3, randomValues: [0.4, 0.9, 0.1, 0.7] });
  assert.equal(draws(), 4);
  assert.deepEqual(workspace.samples.limits, { reachable: 2, unreachable: 2, violations: 3 });
  assert.deepEqual(workspace.reachable.map(sample => sample.pose.x), [0, 2]);
  assert.deepEqual(workspace.unreachable.map(sample => sample.pose.x), [6, 4]);
  assert.deepEqual(workspace.violations.map(sample => sample.pose.x), [3, 4, 6]);
  assert.deepEqual(workspace.violations[2].violations.map(v => v.type), ['conditionLimit']);
  // The counts and coverage do not depend on what the reservoir kept.
  assert.deepEqual(workspace.counts, { relaxedReachable: 4, reachable: 3, unreachable: 4, violationPoses: 4 });
  assert.equal(workspace.coverage, 30);
});

test('sample limits are floored, capped at 200, default the violation limit and allow zero', () => {
  const limits = options => createWorkspaceStatistics({ totalPoses: 1, ...options }).finish().samples.limits;
  assert.deepEqual(limits({}), { reachable: 200, unreachable: 200, violations: 200 });
  assert.deepEqual(limits({ sampleLimit: 500 }), { reachable: 200, unreachable: 200, violations: 200 });
  assert.deepEqual(limits({ sampleLimit: 7.9 }), { reachable: 7, unreachable: 7, violations: 7 });
  assert.deepEqual(limits({ sampleLimit: 7.9, violationSampleLimit: 199.99 }), { reachable: 7, unreachable: 7, violations: 199 });
  assert.deepEqual(limits({ sampleLimit: -3 }), { reachable: 0, unreachable: 0, violations: 0 });
  const { workspace, draws } = aggregate({ sampleLimit: 0 });
  assert.equal(draws(), 0);
  assert.deepEqual([workspace.reachable, workspace.unreachable, workspace.violations], [[], [], []]);
  assert.deepEqual(workspace.counts, { relaxedReachable: 4, reachable: 3, unreachable: 4, violationPoses: 4 });
});
