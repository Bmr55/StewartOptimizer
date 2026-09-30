import { evaluatePose, ensureLayout } from '../model/pose.js';

export const POSE_AXES = Object.freeze(['x', 'y', 'z', 'rx', 'ry', 'rz']);
export const HOME_POSE = Object.freeze({ x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 });
export const ANIMATION_PATTERNS = Object.freeze(['none', 'wobble', 'pingpong', 'rotate', 'tilt', 'helical']);

const copy = value => value == null ? value : structuredClone(value);

// Options come from the UI, saved JSON and headless callers; a string or array
// would otherwise be spread into the option map character by character.
function plainOptions(value) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new TypeError('Simulator options must be an object.');
  return value;
}

export function normalizePose(pose = {}) {
  if (!pose || typeof pose !== 'object' || Array.isArray(pose)) {
    throw new TypeError('A requested pose must be an object with six finite coordinates.');
  }
  const result = Object.fromEntries(POSE_AXES.map(axis => [axis, pose[axis] ?? 0]));
  if (!Object.values(result).every(Number.isFinite)) {
    throw new RangeError('A requested pose must have six finite coordinates.');
  }
  return result;
}

export function animationPose(pattern, seconds, { amplitudeMm = 12, rotationRad = Math.PI / 18,
  apexMm = 20, frequencyHz = 0.25 } = {}) {
  if (!ANIMATION_PATTERNS.includes(pattern)) throw new RangeError(`Unknown animation pattern: ${pattern}`);
  const phase = 2 * Math.PI * frequencyHz * seconds;
  const sine = Math.sin(phase);
  const cosine = Math.cos(phase);
  switch (pattern) {
    case 'wobble': return { ...HOME_POSE, rx: rotationRad * sine, ry: rotationRad * cosine };
    case 'pingpong': return { ...HOME_POSE, z: apexMm * Math.sin(phase) };
    case 'rotate': return { ...HOME_POSE, rz: rotationRad * sine };
    case 'tilt': return { ...HOME_POSE, rx: rotationRad * sine, ry: rotationRad * Math.sin(phase * 0.5) };
    case 'helical': return { ...HOME_POSE, x: amplitudeMm * (cosine - 1),
      y: amplitudeMm * sine, z: apexMm * Math.sin(phase * 0.5), rz: rotationRad * sine };
    default: return { ...HOME_POSE };
  }
}

export function createSimulatorController({ onChange } = {}) {
  const listeners = new Set(onChange ? [onChange] : []);
  let layout = null;
  let source = null;
  let options = {};
  let requested = { ...HOME_POSE };
  let accepted = null;
  let assessment = null;
  let acceptedAssessment = null;
  let requestSource = 'load';
  let animation = { pattern: 'none', playing: false, seconds: 0, speed: 1, pauseReason: null };
  let markers = true;
  let tracesEnabled = false;
  let trace = [];

  function getState() {
    return copy({ layout, source, options, requested, accepted, assessment, acceptedAssessment,
      requestSource, rejected: Boolean(assessment && !assessment.reachable), animation,
      markers, tracesEnabled, trace });
  }

  function notify() {
    const snapshot = getState();
    for (const listener of listeners) listener(snapshot);
    return snapshot;
  }

  function requestPose(pose, { source: origin = 'manual' } = {}) {
    if (!layout) throw new Error('Load a layout before requesting a pose.');
    if (animation.playing && origin !== 'animation') {
      animation.playing = false;
      animation.pauseReason = 'Manual pose request';
    }
    requested = normalizePose(pose);
    requestSource = origin;
    assessment = evaluatePose(layout, requested, { ...options, recordLegData: true });
    if (assessment.reachable) {
      accepted = { ...requested };
      acceptedAssessment = assessment;
      if (tracesEnabled) {
        const center = assessment.translation;
        trace.push(center.slice());
        if (trace.length > 300) trace.shift();
      }
    } else if (origin === 'animation') {
      animation.playing = false;
      animation.pauseReason = assessment.violations.map(violation => violation.type).join(', ') || 'Pose rejected';
    }
    return notify();
  }

  function loadLayout(nextLayout, { source: nextSource = { kind: 'import' },
    options: nextOptions = {} } = {}) {
    ensureLayout(nextLayout);
    const nextLayoutCopy = copy(nextLayout);
    const nextOptionsCopy = copy(plainOptions(nextOptions));
    // Validate the options against the home pose before touching any state, as
    // setOptions does, so an invalid load leaves the previous layout intact.
    evaluatePose(nextLayoutCopy, HOME_POSE, { ...nextOptionsCopy, recordLegData: true });
    layout = nextLayoutCopy;
    source = copy(nextSource);
    options = nextOptionsCopy;
    requested = { ...HOME_POSE };
    accepted = null;
    assessment = null;
    acceptedAssessment = null;
    trace = [];
    animation = { ...animation, playing: false, seconds: 0, pauseReason: null };
    return requestPose(HOME_POSE, { source: 'load' });
  }

  function clear() {
    layout = null;
    source = null;
    options = {};
    requested = { ...HOME_POSE };
    accepted = null;
    assessment = null;
    acceptedAssessment = null;
    trace = [];
    animation = { ...animation, playing: false, seconds: 0, pauseReason: null };
    return notify();
  }

  function setOptions(patch) {
    const nextOptions = { ...options, ...copy(plainOptions(patch)) };
    if (layout) evaluatePose(layout, requested, { ...nextOptions, recordLegData: true });
    options = nextOptions;
    if (!layout) return notify();
    // A settings change must recheck the accepted pose as well as the request.
    if (accepted) {
      const checked = evaluatePose(layout, accepted, { ...options, recordLegData: true });
      if (!checked.reachable) {
        accepted = null;
        acceptedAssessment = null;
      } else acceptedAssessment = checked;
    }
    return requestPose(requested, { source: 'settings' });
  }

  function setAnimation(pattern, playing = false, settings = {}) {
    if (!ANIMATION_PATTERNS.includes(pattern)) throw new RangeError(`Unknown animation pattern: ${pattern}`);
    const speed = settings.speed ?? animation.speed;
    if (!Number.isFinite(speed) || speed <= 0) throw new RangeError('Animation speed must be positive.');
    animation = { ...animation, pattern, playing: Boolean(playing && pattern !== 'none'), speed,
      seconds: settings.reset ? 0 : animation.seconds, pauseReason: null };
    return notify();
  }

  function tick(deltaSeconds, settings = {}) {
    if (!animation.playing || !layout) return getState();
    if (!Number.isFinite(deltaSeconds) || deltaSeconds < 0) throw new RangeError('Animation step must be nonnegative.');
    animation.seconds += Math.min(deltaSeconds, 0.1) * animation.speed;
    return requestPose(animationPose(animation.pattern, animation.seconds, settings), { source: 'animation' });
  }

  return {
    loadLayout, clear, requestPose, setOptions, setAnimation, tick, getState,
    subscribe(listener) { listeners.add(listener); listener(getState()); return () => listeners.delete(listener); },
    getReferenceLayout() { return copy(layout); },
    setMarkers(enabled) { markers = Boolean(enabled); return notify(); },
    setTraces(enabled) { tracesEnabled = Boolean(enabled); return notify(); },
    clearTrace() { trace = []; return notify(); },
    dispose() { listeners.clear(); },
  };
}
