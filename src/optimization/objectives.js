import { METRICS } from '../contracts.js';

export const OBJECTIVE_SETS = Object.freeze({
  compact: Object.freeze(['coverage', 'conditioningQuality', 'torque', 'speedDemand']),
  full: Object.freeze(['coverage', 'conditioningQuality', 'torque', 'speedDemand',
    'dexterity', 'stiffness', 'loadSharing', 'limitMargin', 'fatigue']),
  // Full before solved rod-force load sharing; kept to replay older runs.
  'full-v1': Object.freeze(['coverage', 'conditioningQuality', 'torque', 'speedDemand',
    'dexterity', 'stiffness', 'loadBalance', 'limitMargin', 'fatigue']),
});

const LEGACY_KEYS = ['coverage', 'relaxedCoverage', 'dexterity', 'stiffness',
  'loadBalance', 'isotropy', 'limitMargin', 'torque', 'speedDemand', 'fatigue'];
const APPROXIMATION = Object.freeze({ stiffness: 'geometric proxy',
  loadBalance: 'legacy directional proxy', loadSharing: 'solved rod-force balance over the cycle',
  limitMargin: 'sampled proxy', fatigue: 'heuristic' });

export function normalizeObjectiveSet(input = 'compact') {
  if (Array.isArray(input) && input.length === LEGACY_KEYS.length
      && input.every((key, index) => key === LEGACY_KEYS[index])) return 'legacy-v2';
  if (input === 'legacy-v2' || Object.hasOwn(OBJECTIVE_SETS, input)) return input;
  throw new RangeError('objectiveSet must be compact, full or full-v1.');
}

export function objectiveDefinitions(input = 'compact') {
  const name = normalizeObjectiveSet(input);
  const keys = name === 'legacy-v2' ? LEGACY_KEYS : OBJECTIVE_SETS[name];
  return keys.map(key => ({ key, direction: METRICS[key].direction,
    unit: METRICS[key].unit, approximation: APPROXIMATION[key] ?? null }));
}

export function objectiveValues(evaluation, input = 'compact') {
  return objectiveDefinitions(input).map(({ key, direction }) => {
    const value = evaluation[key];
    return Number.isFinite(value) ? direction === 'min' ? -value : value : -Infinity;
  });
}
