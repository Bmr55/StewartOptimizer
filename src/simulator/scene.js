import { rotateVector, vectorAdd, vectorCross, vectorNormalize, vectorScale } from '../math.js';
import { hornFrameAxes } from '../model/kinematics.js';
import { effectiveServoRange, socketNormalsInWorld } from '../model/pose.js';

// The scene is a list of builders, each `(state, layout, solved) => { lines, points }`.
// `solved` is the accepted assessment (it may be null); nothing here evaluates a
// pose. Builders run in list order and their output is concatenated, so the
// order below fixes the draw order. A builder with an `overlay` key is drawn
// only when that key is on in `state.overlays`; the rest are always drawn.

export const SCENE_COLORS = Object.freeze({
  base: [0.24, 0.64, 0.94], platform: [0.43, 0.88, 0.72],
  horn: [1, 0.69, 0.31], rod: [0.88, 0.89, 0.94],
  failure: [1, 0.26, 0.32], globalFailure: [1, 0.38, 0.8],
  servo: [1, 0.42, 0.39], trace: [0.72, 0.5, 1],
  x: [1, 0.38, 0.38], y: [0.39, 0.92, 0.47], z: [0.42, 0.62, 1],
  limitRange: [0.5, 0.56, 0.66], nearLimit: [1, 0.88, 0.2],
});
const COLORS = SCENE_COLORS;
// The canvas clear colour; the ghost dims toward it instead of blending.
export const SCENE_BACKGROUND = Object.freeze([0.055, 0.075, 0.11]);
const GHOST_BRIGHTNESS = 0.35;
const dim = color => color.map((value, k) => SCENE_BACKGROUND[k] + (value - SCENE_BACKGROUND[k]) * GHOST_BRIGHTNESS);

// Toggleable overlays and whether each is drawn when a state or saved file
// does not say. New overlays default off unless their issue says otherwise.
export const OVERLAY_DEFAULTS = Object.freeze({ servoArcs: true, jointCones: true, requestedGhost: true,
  platformAxes: true, worldAxes: true });
export const OVERLAY_NAMES = Object.freeze(Object.keys(OVERLAY_DEFAULTS));
// Limit overlays (servo travel, socket cones) tint a value this close to its
// limit as a warning before the evaluator rejects it: 5°, in radians.
export const NEAR_LIMIT_MARGIN_RAD = 5 * Math.PI / 180;
const SERVO_ARC_SEGMENTS = 24;
const JOINT_CONE_SEGMENTS = 24;
const JOINT_CONE_GENERATRICES = 4;

function polygon(lines, points, color) {
  points.forEach((point, i) => lines.push({ from: point, to: points[(i + 1) % points.length], color }));
}

const hasSolvedLegs = solved => solved?.platformPoints?.length === 6 && solved?.hornTips?.length === 6;

// Color reports failures in the requested pose, which may have been rejected by
// the shared evaluator, while the geometry stays at the accepted pose.
function failureColor(state) {
  const violations = state.assessment?.violations ?? [];
  const affectedLegs = new Set(violations.filter(violation => Number.isInteger(violation.leg))
    .map(violation => violation.leg));
  const globalFailure = violations.some(violation => !Number.isInteger(violation.leg));
  return index => affectedLegs.has(index) ? COLORS.failure : globalFailure ? COLORS.globalFailure : null;
}

function base(state, layout) {
  const lines = [], points = [];
  const legColor = failureColor(state);
  polygon(lines, layout.baseAnchors, COLORS.base);
  for (let i = 0; i < 6; i++) {
    const anchor = layout.baseAnchors[i];
    const beta = layout.betaAngles[i];
    const direction = [Math.cos(beta), Math.sin(beta), 0];
    lines.push({ from: anchor, to: vectorAdd(anchor, vectorScale(direction, Math.max(12, layout.hornLength * 0.35))), color: legColor(i) ?? COLORS.servo });
    if (state.markers) points.push({ at: anchor, color: legColor(i) ?? COLORS.servo, size: 7 });
  }
  return { lines, points };
}

function platform(state, layout, solved) {
  const lines = [];
  if (hasSolvedLegs(solved)) polygon(lines, solved.platformPoints, COLORS.platform);
  return { lines, points: [] };
}

function legs(state, layout, solved) {
  const lines = [], points = [];
  if (!hasSolvedLegs(solved)) return { lines, points };
  const legColor = failureColor(state);
  for (let i = 0; i < 6; i++) {
    lines.push({ from: layout.baseAnchors[i], to: solved.hornTips[i], color: legColor(i) ?? COLORS.horn });
    lines.push({ from: solved.hornTips[i], to: solved.platformPoints[i], color: legColor(i) ?? COLORS.rod });
    if (state.markers) {
      points.push({ at: solved.hornTips[i], color: legColor(i) ?? COLORS.horn, size: 6 });
      points.push({ at: solved.platformPoints[i], color: legColor(i) ?? COLORS.platform, size: 7 });
    }
  }
  return { lines, points };
}

// Each servo's allowed travel drawn as an arc of horn-length radius about the
// base anchor, in the horn plane the evaluator uses, with ticks at both stops
// and a marker across the arc at the accepted horn angle. Failure colour when
// the requested pose breaks this servo's range, a warning tint when the
// accepted angle is within NEAR_LIMIT_MARGIN_RAD of a stop.
function servoArcs(state, layout, solved) {
  const lines = [];
  const [min, max] = effectiveServoRange(layout, state.options ?? {});
  const violations = state.assessment?.violations ?? [];
  const at = (i, alpha, scale) => vectorAdd(layout.baseAnchors[i],
    vectorScale(hornFrameAxes(layout.betaAngles[i], alpha)[0], layout.hornLength * scale));
  for (let i = 0; i < 6; i++) {
    const angle = solved?.servoAngles?.[i];
    const color = violations.some(violation => violation.type === 'servoLimit' && violation.leg === i) ? COLORS.failure
      : Number.isFinite(angle) && Math.min(angle - min, max - angle) < NEAR_LIMIT_MARGIN_RAD ? COLORS.nearLimit
        : COLORS.limitRange;
    for (let step = 0; step < SERVO_ARC_SEGMENTS; step++) {
      const alpha = min + (max - min) * step / SERVO_ARC_SEGMENTS;
      const next = step + 1 === SERVO_ARC_SEGMENTS ? max : min + (max - min) * (step + 1) / SERVO_ARC_SEGMENTS;
      lines.push({ from: at(i, alpha, 1), to: at(i, next, 1), color });
    }
    for (const stop of [min, max]) lines.push({ from: at(i, stop, 0.85), to: at(i, stop, 1.15), color });
    if (Number.isFinite(angle)) {
      lines.push({ from: at(i, angle, 0.8), to: at(i, angle, 1.25), color: color === COLORS.limitRange ? COLORS.horn : color });
    }
  }
  return { lines, points: [] };
}

// A cone of the given half-angle about a unit axis, as a ring at a fixed slant
// length from the apex plus a few generatrix lines. A fixed slant rather than a
// fixed height keeps wide limits (up to 180°) bounded.
function cone(lines, apex, axis, halfAngle, slant, color) {
  const u = vectorNormalize(vectorCross(axis, Math.abs(axis[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0]));
  const v = vectorCross(axis, u);
  const at = turn => vectorAdd(apex, vectorScale(vectorAdd(vectorScale(axis, Math.cos(halfAngle)),
    vectorScale(vectorAdd(vectorScale(u, Math.cos(turn)), vectorScale(v, Math.sin(turn))), Math.sin(halfAngle))), slant));
  const ring = Array.from({ length: JOINT_CONE_SEGMENTS }, (_, k) => at(2 * Math.PI * k / JOINT_CONE_SEGMENTS));
  polygon(lines, ring, color);
  for (let k = 0; k < JOINT_CONE_GENERATRICES; k++) {
    lines.push({ from: apex, to: ring[k * JOINT_CONE_SEGMENTS / JOINT_CONE_GENERATRICES], color });
  }
}

// Each ball-joint socket's allowed cone at the accepted pose: apex at the rod
// end (horn tip for the lower socket, platform point for the upper), axis along
// the socket normal from the evaluator's construction, half-angle equal to that
// socket's effective limit. Failure colour when the requested pose breaks that
// socket's limit, a warning tint when the accepted angle is within
// NEAR_LIMIT_MARGIN_RAD of it. A limit picture, not a collision check.
function jointCones(state, layout, solved) {
  const lines = [];
  if (!hasSolvedLegs(solved) || !solved.mounting || !solved.jointLimits) return { lines, points: [] };
  const violations = state.assessment?.violations ?? [];
  const slant = Math.max(12, layout.hornLength * 0.35);
  for (let i = 0; i < 6; i++) {
    const mounts = { lower: solved.mounting.lower[i]?.direction, upper: solved.mounting.upper[i]?.direction };
    const alpha = solved.servoAngles?.[i];
    if (!mounts.lower || !mounts.upper || !Number.isFinite(alpha)) continue;
    const normals = socketNormalsInWorld(layout.betaAngles[i], alpha, solved.rotationMatrix, mounts);
    for (const [joint, apex] of [['lower', solved.hornTips[i]], ['upper', solved.platformPoints[i]]]) {
      const limit = solved.jointLimits[joint];
      const angle = solved.jointAngles?.[joint]?.[i];
      const failed = violations.some(violation => violation.type === 'ballJoint' && violation.leg === i
        && violation.joint === joint);
      const color = failed ? COLORS.failure
        : Number.isFinite(angle) && limit - angle < NEAR_LIMIT_MARGIN_RAD ? COLORS.nearLimit : COLORS.limitRange;
      cone(lines, apex, vectorNormalize(normals[joint]), limit, slant, color);
    }
  }
  return { lines, points: [] };
}

// The rejected request drawn faintly beside the accepted pose: the one layer
// that shows geometry the evaluator did not accept. It uses only what the
// rejected evaluation returned. The platform comes from its translation and
// rotation, which are always set; legs are drawn only where the solver reached
// a horn tip (it stops at the first structural failure). Everything is dimmed
// toward the background and has no markers, except legs named in a violation,
// which use the failure colour at full brightness, and the platform outline,
// which turns the whole-platform failure colour on a conditioning failure.
function requestedGhost(state, layout) {
  const lines = [];
  const requested = state.assessment;
  if (!requested || requested.reachable !== false || !requested.translation || !requested.rotationMatrix) {
    return { lines, points: [] };
  }
  const violations = requested.violations ?? [];
  const failedLegs = new Set(violations.filter(violation => Number.isInteger(violation.leg)).map(violation => violation.leg));
  const platformFailure = violations.some(violation => !Number.isInteger(violation.leg));
  const platformPoints = layout.platformAnchors.map(anchor =>
    vectorAdd(requested.translation, rotateVector(requested.rotationMatrix, anchor)));
  polygon(lines, platformPoints, platformFailure ? COLORS.globalFailure : dim(COLORS.platform));
  const hornTips = requested.hornTips ?? [];
  for (let i = 0; i < 6; i++) {
    if (!hornTips[i]) continue;
    const failed = failedLegs.has(i);
    lines.push({ from: layout.baseAnchors[i], to: hornTips[i], color: failed ? COLORS.failure : dim(COLORS.horn) });
    lines.push({ from: hornTips[i], to: platformPoints[i], color: failed ? COLORS.failure : dim(COLORS.rod) });
  }
  const axis = Math.max(18, layout.hornLength * 0.35);
  const column = index => requested.rotationMatrix.map(row => row[index]);
  for (const [index, color] of [[0, COLORS.x], [1, COLORS.y], [2, COLORS.z]]) {
    lines.push({ from: requested.translation, to: vectorAdd(requested.translation, vectorScale(column(index), axis)), color: dim(color) });
  }
  return { lines, points: [] };
}

function platformAxes(state, layout, solved) {
  const lines = [];
  if (!hasSolvedLegs(solved)) return { lines, points: [] };
  const center = solved.translation;
  const axis = Math.max(18, layout.hornLength * 0.35);
  const column = index => solved.rotationMatrix.map(row => row[index]);
  for (const [index, color] of [[0, COLORS.x], [1, COLORS.y], [2, COLORS.z]]) {
    lines.push({ from: center, to: vectorAdd(center, vectorScale(column(index), axis)), color });
  }
  return { lines, points: [] };
}

function worldAxes() {
  const origin = [0, 0, 0];
  return { lines: [[[30, 0, 0], COLORS.x], [[0, 30, 0], COLORS.y], [[0, 0, 30], COLORS.z]]
    .map(([direction, color]) => ({ from: origin, to: direction, color })), points: [] };
}

function trace(state) {
  const positions = state.trace ?? [];
  const lines = [];
  for (let i = 1; i < positions.length; i++) lines.push({ from: positions[i - 1], to: positions[i], color: COLORS.trace });
  return { lines, points: [] };
}

export const SCENE_BUILDERS = Object.freeze([
  { name: 'base', build: base },
  { name: 'platform', build: platform },
  { name: 'legs', build: legs },
  { name: 'servoArcs', overlay: 'servoArcs', build: servoArcs },
  { name: 'jointCones', overlay: 'jointCones', build: jointCones },
  { name: 'requestedGhost', overlay: 'requestedGhost', build: requestedGhost },
  { name: 'platformAxes', overlay: 'platformAxes', build: platformAxes },
  { name: 'worldAxes', overlay: 'worldAxes', build: worldAxes },
  { name: 'trace', build: trace },
]);

export function buildSceneGeometry(state, builders = SCENE_BUILDERS) {
  const lines = [], points = [];
  const { layout, acceptedAssessment: solved } = state;
  if (!layout) return { lines, points };
  const overlays = { ...OVERLAY_DEFAULTS, ...state.overlays };
  const scene = { ...state, markers: state.markers === undefined ? true : state.markers };
  for (const builder of builders) {
    if (builder.overlay && !overlays[builder.overlay]) continue;
    const part = builder.build(scene, layout, solved ?? null);
    lines.push(...part.lines);
    points.push(...part.points);
  }
  return { lines, points };
}

// Reads a toggle map from saved JSON: known overlay names only, each true or
// false. Unknown names are dropped so a file from a newer version still loads.
export function parseOverlays(value, field = 'overlays') {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new TypeError(`${field} must be an object.`);
  const result = {};
  for (const name of OVERLAY_NAMES) {
    if (value[name] === undefined) continue;
    if (typeof value[name] !== 'boolean') throw new TypeError(`${field}.${name} must be true or false.`);
    result[name] = value[name];
  }
  return result;
}
