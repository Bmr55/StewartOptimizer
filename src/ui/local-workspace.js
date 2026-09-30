export const LOCAL_WORKSPACE_KEY = 'stewart-optimizer.workspace.v1';

const INPUT_IDS = [
  'requirementsInput', 'referenceLayoutInput', 'optTopology', 'homeHeightMin',
  'homeHeightMax', 'optPopulation', 'optGenerations', 'optObjectiveSet',
  'optMutationRate', 'optSampling', 'optSeed', 'ballJointLimit',
  'servoTorqueRating', 'servoSpeedRating', 'servoRatingPolicy',
  ...['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'].flatMap(axis =>
    ['Min', 'Max', 'Step'].map(suffix => `opt${axis}${suffix}`)),
  ...Array.from({ length: 6 }, (_, index) =>
    [`servoTorque${index + 1}`, `servoSpeed${index + 1}`]).flat(),
];

export function captureLocalWorkspace(document, simulator = null) {
  return {
    version: 1,
    inputs: Object.fromEntries(INPUT_IDS.map(id => [id, document.getElementById(id).value])),
    ballJointClamp: document.getElementById('ballJointClamp').checked,
    simulator,
  };
}

export function parseLocalWorkspace(raw) {
  const saved = JSON.parse(raw);
  if (saved?.version !== 1 || !saved.inputs || typeof saved.inputs !== 'object'
    || INPUT_IDS.some(id => typeof saved.inputs[id] !== 'string')
    || typeof saved.ballJointClamp !== 'boolean'
    || (saved.simulator !== null && (typeof saved.simulator !== 'object' || !saved.simulator))) {
    throw new Error('The local workspace has an unsupported or incomplete format.');
  }
  return saved;
}

export function applyLocalWorkspace(document, saved) {
  for (const id of INPUT_IDS) document.getElementById(id).value = saved.inputs[id];
  document.getElementById('ballJointClamp').checked = saved.ballJointClamp;
}
