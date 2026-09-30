import { degToRad } from '../math.js';

export const TRAJECTORY_MODEL = 'sinusoid-v1';
const TRANSLATION_AXES = ['x', 'y', 'z'];
const ROTATION_AXES = ['rx', 'ry', 'rz'];
const AXES = [...TRANSLATION_AXES, ...ROTATION_AXES];

const finiteNumber = (value, name) => {
  if (!Number.isFinite(value)) throw new RangeError(`${name} must be a finite number.`);
  return value;
};

// A common-frequency multi-axis sinusoid about home. Translation amplitudes are
// mm; rotation amplitudes are degrees of the X-Y-Z (roll, pitch, yaw) Euler
// angles used by pose evaluation, R = Rz(rz) Ry(ry) Rx(rx).
export function normalizeTrajectory(input) {
  if (!input || typeof input !== 'object' || Array.isArray(input)) {
    throw new TypeError('trajectory must be an object.');
  }
  const type = input.type ?? 'sinusoid';
  if (type !== 'sinusoid') throw new RangeError('trajectory.type must be sinusoid.');
  const frequency = finiteNumber(input.frequency_hz, 'trajectory.frequency_hz');
  if (frequency < 0) throw new RangeError('trajectory.frequency_hz must be >= 0.');
  if (!Array.isArray(input.components)) throw new TypeError('trajectory.components must be an array.');
  const seen = new Set();
  const components = input.components.map((component, index) => {
    const name = `trajectory.components[${index}]`;
    if (!component || typeof component !== 'object' || Array.isArray(component)) {
      throw new TypeError(`${name} must be an object.`);
    }
    const axis = typeof component.axis === 'string' ? component.axis.toLowerCase() : component.axis;
    if (!AXES.includes(axis)) throw new RangeError(`${name}.axis must be one of ${AXES.join(', ')}.`);
    if (seen.has(axis)) throw new RangeError(`${name}.axis ${axis} is repeated.`);
    seen.add(axis);
    const rotational = ROTATION_AXES.includes(axis);
    const amplitudeKey = rotational ? 'amplitude_deg' : 'amplitude_mm';
    const wrongKey = rotational ? 'amplitude_mm' : 'amplitude_deg';
    if (wrongKey in component) throw new RangeError(`${name} uses ${wrongKey}; ${axis} requires ${amplitudeKey}.`);
    const amplitude = finiteNumber(component[amplitudeKey], `${name}.${amplitudeKey}`);
    if (amplitude < 0) throw new RangeError(`${name}.${amplitudeKey} must be >= 0.`);
    const phase = finiteNumber(component.phase_deg ?? 0, `${name}.phase_deg`);
    return { axis, [amplitudeKey]: amplitude, phase_deg: phase };
  });
  return { type, frequency_hz: frequency, components };
}

// The legacy single-axis input is the sinusoid with half the peak-to-peak stroke and zero phase.
export function legacyTrajectory({ stroke = 0, frequency = 0, axis = 'z' } = {}) {
  if (!TRANSLATION_AXES.includes(axis)) throw new RangeError('Unsupported cycle axis.');
  return { type: 'sinusoid', frequency_hz: frequency,
    components: [{ axis, amplitude_mm: stroke / 2, phase_deg: 0 }] };
}

export function trajectoryFromRequirements(requirements = {}) {
  if (requirements.trajectory != null) return { trajectory: normalizeTrajectory(requirements.trajectory), source: 'supplied' };
  return { trajectory: legacyTrajectory({ stroke: requirements.cycle_mm ?? 0,
    frequency: requirements.frequency_hz ?? 0, axis: requirements.cycle_axis ?? 'z' }), source: 'legacy-cycle' };
}

export function isStationary(trajectory) {
  return !(trajectory.frequency_hz > 0)
    || trajectory.components.every(c => !((c.amplitude_mm ?? c.amplitude_deg) > 0));
}

export function trajectoryIdentity(trajectory) {
  const parts = trajectory.components.map(c => `${c.axis}:${c.amplitude_mm ?? c.amplitude_deg}`
    + `${'amplitude_mm' in c ? 'mm' : 'deg'}@${c.phase_deg}deg`);
  return `${TRAJECTORY_MODEL}:f=${trajectory.frequency_hz}Hz;${parts.join(';')}`;
}

// Heuristic inputs retained from the single-axis model: frequency and largest translational peak-to-peak stroke.
export function trajectorySummary(trajectory) {
  const strokes = trajectory.components.filter(c => 'amplitude_mm' in c).map(c => 2 * c.amplitude_mm);
  return { frequency: trajectory.frequency_hz, stroke: Math.max(0, ...strokes) };
}

// Euler-angle rates are not physical angular velocity. For R = Rz(c) Ry(b) Rx(a),
// omega = a' Rz Ry e_x + b' Rz e_y + c' e_z in the base frame.
export function eulerRatesToAngular([, b, c], [da, db, dc], [dda, ddb, ddc] = [0, 0, 0]) {
  const cb = Math.cos(b), sb = Math.sin(b), cc = Math.cos(c), sc = Math.sin(c);
  const c1 = [cc * cb, sc * cb, -sb];
  const c2 = [-sc, cc, 0];
  const d1 = [-sc * dc * cb - cc * sb * db, cc * dc * cb - sc * sb * db, -cb * db];
  const d2 = [-cc * dc, -sc * dc, 0];
  const omega = [0, 1, 2].map(i => da * c1[i] + db * c2[i] + (i === 2 ? dc : 0));
  const alpha = [0, 1, 2].map(i => dda * c1[i] + ddb * c2[i] + (i === 2 ? ddc : 0)
    + da * d1[i] + db * d2[i]);
  return { omega, alpha };
}

// State at time t (s). Pose is mm/radians for evaluatePose; derivatives are SI and
// expressed in the base frame at the moving-platform origin.
export function trajectoryState(trajectory, time) {
  const w = 2 * Math.PI * trajectory.frequency_hz;
  const stationary = isStationary(trajectory);
  const pose = { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 };
  const rate = { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 };
  const second = { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 };
  if (!stationary) {
    for (const component of trajectory.components) {
      const rotational = 'amplitude_deg' in component;
      // Translation in meters for derivatives, rotation in radians.
      const amplitude = rotational ? degToRad(component.amplitude_deg) : component.amplitude_mm / 1000;
      const angle = w * time + degToRad(component.phase_deg);
      const value = amplitude * Math.sin(angle);
      pose[component.axis] = rotational ? value : value * 1000;
      rate[component.axis] = w * amplitude * Math.cos(angle);
      second[component.axis] = -w * w * value;
    }
  }
  const angles = [pose.rx, pose.ry, pose.rz];
  const { omega, alpha } = eulerRatesToAngular(angles, [rate.rx, rate.ry, rate.rz],
    [second.rx, second.ry, second.rz]);
  return { time, pose, velocity: [rate.x, rate.y, rate.z], acceleration: [second.x, second.y, second.z],
    omega, angularAcceleration: alpha, eulerRates: [rate.rx, rate.ry, rate.rz] };
}
