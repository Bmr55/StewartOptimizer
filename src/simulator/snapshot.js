import { evaluatePose } from '../model/pose.js';
import { ANIMATION_PATTERNS, HOME_POSE, normalizePose } from './controller.js';
import { parseCamera } from './view.js';

export const POINTER_MODES = Object.freeze(['orbit', 'platform']);
// Everything the simulator JSON `simulator.options` block may carry; other keys are dropped.
export const SIMULATOR_OPTION_KEYS = Object.freeze(['ballJointLimitDeg', 'lowerBallJointLimitDeg',
  'upperBallJointLimitDeg', 'ballJointClamp', 'conditionLimit', 'servoRangeRad', 'rodLengthTolerance']);

function plainObject(value, field) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new TypeError(`${field} must be an object.`);
  return value;
}

function pose(value, field) {
  if (value == null) return null;
  try { return normalizePose(plainObject(value, field)); }
  catch (error) { throw new (error.constructor)(`${field}: ${error.message}`); }
}

// Validates the `simulator` block of simulator JSON (Load optimizer reference,
// browser-save restore) before any of it is applied, so a rejected file leaves
// the current layout, pose, camera and animation untouched. `fallbackOptions`
// are used, and checked, when the block carries no options.
export function parseSimulatorSnapshot(saved, layout, fallbackOptions) {
  const block = saved == null ? {} : plainObject(saved, 'simulator');
  let options;
  if (block.options == null) options = { ...fallbackOptions };
  else {
    plainObject(block.options, 'simulator.options');
    options = Object.fromEntries(SIMULATOR_OPTION_KEYS.filter(key => block.options[key] !== undefined)
      .map(key => [key, structuredClone(block.options[key])]));
    if ('ballJointClamp' in options && typeof options.ballJointClamp !== 'boolean') {
      throw new TypeError('simulator.options.ballJointClamp must be true or false.');
    }
  }
  try { evaluatePose(layout, HOME_POSE, { ...options, recordLegData: true }); }
  catch (error) { throw new (error.constructor)(`simulator.options: ${error.message}`); }

  const result = { options, requested: pose(block.requested, 'simulator.requested'),
    accepted: pose(block.accepted, 'simulator.accepted'), camera: null, animation: null,
    markers: null, tracesEnabled: null, pointerMode: null };
  if (block.camera != null) result.camera = parseCamera(block.camera, 'simulator.camera');
  if (block.animation != null) {
    const animation = plainObject(block.animation, 'simulator.animation');
    // A saved idle or unknown pattern selects the playable default; speed must be usable.
    const pattern = animation.pattern !== 'none' && ANIMATION_PATTERNS.includes(animation.pattern)
      ? animation.pattern : 'wobble';
    const speed = animation.speed == null ? 1 : animation.speed;
    if (typeof speed !== 'number' || !Number.isFinite(speed) || speed <= 0) {
      throw new RangeError('simulator.animation.speed must be a positive finite number.');
    }
    result.animation = { pattern, speed };
  }
  for (const key of ['markers', 'tracesEnabled']) {
    if (block[key] === undefined) continue;
    if (typeof block[key] !== 'boolean') throw new TypeError(`simulator.${key} must be true or false.`);
    result[key] = block[key];
  }
  // An empty mode (a select without a value) counts as not saved.
  if (block.pointerMode != null && block.pointerMode !== '') {
    if (!POINTER_MODES.includes(block.pointerMode)) {
      throw new RangeError(`simulator.pointerMode must be one of ${POINTER_MODES.join(', ')}.`);
    }
    result.pointerMode = block.pointerMode;
  }
  return result;
}
