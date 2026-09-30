import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { buildSceneGeometry, createWebGLRenderer, projectPoint } from '../../src/simulator/renderer.js';
import { NEAR_LIMIT_MARGIN_RAD, OVERLAY_DEFAULTS, OVERLAY_NAMES, SCENE_BUILDERS, SCENE_COLORS } from '../../src/simulator/scene.js';
import { computeHornTip } from '../../src/model/kinematics.js';
import { effectiveServoRange } from '../../src/model/pose.js';

test('scene uses solved asymmetric anchor, horn, rod and platform frames', () => {
  const controller = createSimulatorController();
  const state = controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const scene = buildSceneGeometry(state);
  assert.ok(scene.lines.some(line => line.from === undefined) === false);
  for (let leg = 0; leg < 6; leg++) {
    const base = state.layout.baseAnchors[leg];
    const horn = state.acceptedAssessment.hornTips[leg];
    const point = state.acceptedAssessment.platformPoints[leg];
    assert.ok(scene.lines.some(line => line.from === base && line.to === horn));
    assert.ok(scene.lines.some(line => line.from === horn && line.to === point));
    assert.ok(scene.points.some(marker => marker.at === horn));
  }
  assert.equal(state.acceptedAssessment.translation[2], state.layout.homeHeight);
  assert.ok(projectPoint(state.layout.baseAnchors[0], { target: [0, 0, 100] }, 800, 500));
  const rejected = controller.loadLayout(asymmetricJointFixture(),
    { options: { ballJointLimitDeg: 180, conditionLimit: 1 } });
  assert.equal(rejected.accepted, null);
  // Base, servo directions and world axes; servo arcs have no accepted angle to mark.
  assert.equal(buildSceneGeometry({ ...rejected, overlays: { ...rejected.overlays, servoArcs: false } }).lines.length, 15);
});

test('unavailable WebGL2 leaves renderer inactive with actionable error', () => {
  const renderer = createWebGLRenderer({ getContext: () => null });
  assert.equal(renderer.available, false);
  assert.match(renderer.error, /WebGL2.*hardware acceleration.*optimization remains available/i);
  renderer.render({});
});

test('projected depth keeps near/far ordering for a zoomed-out camera', () => {
  const target = [0, 0, 100];
  const yaw = 0.7, pitch = 0.4;
  const towardEye = [Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)];
  const offset = (sign, units) => target.map((value, i) => value + sign * units * towardEye[i]);
  for (const distance of [600, 1900, 2500]) {
    const camera = { target, yaw, pitch, distance };
    const [nearer, middle, farther] = [offset(1, 300), target, offset(-1, 300)]
      .map(point => projectPoint(point, camera, 800, 500)[2]);
    assert.ok(nearer < middle && middle < farther, `distance ${distance}: ${nearer} ${middle} ${farther}`);
    assert.ok(farther < 0.999 && nearer > -0.999, `distance ${distance} saturates the depth range`);
  }
  // The depth mapping is unchanged for the default camera distance.
  const home = projectPoint(target, { target, yaw, pitch, distance: 600 }, 800, 500)[2];
  assert.ok(Math.abs(home - (599 / 2000 * 2 - 1)) < 1e-12);
});

function fakeGL() {
  const calls = { createProgram: 0, drawArrays: 0, deleteProgram: 0 };
  let lost = false;
  const noop = () => {};
  return {
    calls, setLost(value) { lost = value; },
    VERTEX_SHADER: 1, FRAGMENT_SHADER: 2, COMPILE_STATUS: 3, LINK_STATUS: 4, DEPTH_TEST: 5,
    ARRAY_BUFFER: 6, DYNAMIC_DRAW: 7, FLOAT: 8, LINES: 9, POINTS: 10, COLOR_BUFFER_BIT: 16, DEPTH_BUFFER_BIT: 32,
    createShader: () => ({}), shaderSource: noop, compileShader: noop, getShaderParameter: () => true,
    createProgram() { calls.createProgram++; return {}; }, attachShader: noop, linkProgram: noop,
    getProgramParameter: () => true, createBuffer: () => ({}), getAttribLocation: () => 0,
    getUniformLocation: () => ({}), enable: noop, clearColor: noop, viewport: noop, clear: noop,
    useProgram: noop, bindBuffer: noop, bufferData: noop, vertexAttribPointer: noop,
    enableVertexAttribArray: noop, uniform1f: noop, drawArrays() { calls.drawArrays++; },
    deleteBuffer: noop, deleteProgram() { calls.deleteProgram++; }, isContextLost: () => lost,
  };
}

test('a lost context cancels the default, stops drawing, and restore rebuilds the program and redraws', () => {
  const gl = fakeGL();
  const handlers = {};
  const canvas = { width: 300, height: 200, getContext: () => gl,
    addEventListener(type, handler) { handlers[type] = handler; },
    removeEventListener(type) { delete handlers[type]; } };
  const changes = [];
  const renderer = createWebGLRenderer(canvas, { onContextChange: () => changes.push(renderer.contextLost) });
  const controller = createSimulatorController();
  const state = controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  renderer.render(state, {});
  assert.equal(gl.calls.createProgram, 1);
  assert.ok(gl.calls.drawArrays > 0);
  const drawn = gl.calls.drawArrays;
  const event = { defaultPrevented: false, preventDefault() { this.defaultPrevented = true; } };
  handlers.webglcontextlost(event);
  assert.equal(event.defaultPrevented, true, 'the browser is only allowed to restore a cancelled lost event');
  assert.equal(renderer.contextLost, true);
  gl.setLost(true);
  renderer.render(state, {});
  assert.equal(gl.calls.drawArrays, drawn, 'drew while the context was lost');
  gl.setLost(false);
  handlers.webglcontextrestored();
  assert.equal(renderer.contextLost, false);
  assert.equal(gl.calls.createProgram, 2, 'program was not rebuilt after restore');
  renderer.render(state, {});
  assert.ok(gl.calls.drawArrays > drawn);
  assert.deepEqual(changes, [true, false]);
  renderer.dispose();
  assert.deepEqual(Object.keys(handlers), []);
  assert.equal(gl.calls.deleteProgram, 1);
});

// Scene states whose output was frozen from the single-function scene before it
// was split into builders (tests/fixtures/scene-geometry.json). With only the
// overlays that scene drew switched on, the builders must keep drawing the same
// lines and points in the same order; overlays added later are switched off.
const FROZEN_OVERLAYS = { platformAxes: true, worldAxes: true };
const onlyFrozenOverlays = () => Object.fromEntries(OVERLAY_NAMES.map(name => [name, FROZEN_OVERLAYS[name] ?? false]));
function frozenSceneStates() {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  controller.setTraces(true);
  for (const z of [2, 4, 6]) controller.requestPose({ z, rx: 0.02 * z, rz: 0.01 });
  const accepted = controller.requestPose({ x: 3, y: -2, z: 5, rx: 0.05, ry: -0.03, rz: 0.04 });
  const legFailure = controller.requestPose({ z: 200 });
  const globalFailure = { ...accepted, markers: false,
    assessment: { ...accepted.assessment, violations: [{ type: 'conditioning' }] } };
  const noAcceptedPose = createSimulatorController().loadLayout(asymmetricJointFixture(),
    { options: { ballJointLimitDeg: 180, conditionLimit: 1 } });
  return { accepted, legFailure, globalFailure, noAcceptedPose };
}
const frozenScene = JSON.parse(fs.readFileSync(new URL('../fixtures/scene-geometry.json', import.meta.url), 'utf8'));
const plain = value => JSON.parse(JSON.stringify(value));

test('overlay builders reproduce the frozen single-function scene exactly', () => {
  const states = frozenSceneStates();
  assert.equal(states.legFailure.rejected, true);
  assert.deepEqual(states.accepted.overlays, OVERLAY_DEFAULTS);
  for (const [name, state] of Object.entries(states)) {
    assert.deepEqual(plain(buildSceneGeometry({ ...state, overlays: onlyFrozenOverlays() })), frozenScene[name], name);
    // A state without an overlay map (older callers) draws the defaults.
    const { overlays, ...withoutOverlays } = state;
    assert.deepEqual(plain(buildSceneGeometry(withoutOverlays)),
      plain(buildSceneGeometry({ ...state, overlays: OVERLAY_DEFAULTS })), `${name} without overlays`);
  }
});

test('each overlay toggle removes only its own builder output', () => {
  const { accepted } = frozenSceneStates();
  const full = buildSceneGeometry(accepted);
  const parts = Object.fromEntries(SCENE_BUILDERS.map(builder =>
    [builder.name, builder.build(accepted, accepted.layout, accepted.acceptedAssessment)]));
  assert.deepEqual(SCENE_BUILDERS.map(builder => builder.name),
    ['base', 'platform', 'legs', 'servoArcs', 'platformAxes', 'worldAxes', 'trace']);
  assert.deepEqual(SCENE_BUILDERS.filter(builder => builder.overlay).map(builder => builder.overlay), OVERLAY_NAMES);
  assert.equal(parts.platformAxes.lines.length, 3);
  assert.equal(parts.worldAxes.lines.length, 3);
  assert.equal(parts.trace.lines.length, accepted.trace.length - 1);
  for (const name of OVERLAY_NAMES) {
    assert.ok(parts[name].lines.length > 0, name);
    const scene = buildSceneGeometry({ ...accepted, overlays: { ...accepted.overlays, [name]: false } });
    const removed = new Set(parts[name].lines.map(line => JSON.stringify(line)));
    const expected = full.lines.filter(line => !removed.has(JSON.stringify(line)));
    assert.equal(scene.lines.length, full.lines.length - parts[name].lines.length, name);
    assert.deepEqual(plain(scene.lines), plain(expected), name);
    assert.deepEqual(plain(scene.points), plain(full.points), name);
  }
  const bare = buildSceneGeometry({ ...accepted, overlays: Object.fromEntries(OVERLAY_NAMES.map(name => [name, false])) });
  assert.equal(bare.lines.length, full.lines.length - OVERLAY_NAMES.reduce((sum, name) => sum + parts[name].lines.length, 0));
  // Unknown names in a hand-built state are ignored rather than drawn.
  assert.deepEqual(plain(buildSceneGeometry({ ...accepted, overlays: { ...accepted.overlays, ghost: true } })), plain(full));
});

test('a custom builder list is drawn in order and overlay-gated', () => {
  const { accepted } = frozenSceneStates();
  const marker = { at: [0, 0, 0], color: [1, 1, 1], size: 3 };
  const builders = [{ name: 'dot', build: () => ({ lines: [], points: [marker] }) },
    { name: 'gated', overlay: 'worldAxes', build: () => ({ lines: [], points: [marker, marker] }) }];
  assert.equal(buildSceneGeometry(accepted, builders).points.length, 3);
  assert.equal(buildSceneGeometry({ ...accepted, overlays: { worldAxes: false } }, builders).points.length, 1);
  assert.deepEqual(buildSceneGeometry({ ...accepted, layout: null }, builders), { lines: [], points: [] });
});

test('servo arcs span the effective servo range in the evaluator horn plane and mark the accepted angle', () => {
  const { accepted } = frozenSceneStates();
  const { layout, acceptedAssessment: solved } = accepted;
  const arcs = SCENE_BUILDERS.find(builder => builder.name === 'servoArcs');
  const perLeg = 24 + 2 + 1; // Arc segments, two stop ticks, current-angle marker.
  const close = (actual, expected, label) => actual.forEach((value, i) =>
    assert.ok(Math.abs(value - expected[i]) < 1e-9, `${label}: ${actual} vs ${expected}`));
  for (const options of [{}, { servoRangeRad: [-0.4, 0.9] }]) {
    const state = { ...accepted, options: { ...accepted.options, ...options } };
    const [min, max] = effectiveServoRange(layout, state.options);
    const { lines } = arcs.build(state, layout, solved);
    assert.equal(lines.length, 6 * perLeg);
    for (let leg = 0; leg < 6; leg++) {
      const own = lines.slice(leg * perLeg, (leg + 1) * perLeg);
      const tip = alpha => computeHornTip(layout.baseAnchors[leg], layout.hornLength, layout.betaAngles[leg], alpha);
      close(own[0].from, tip(min), `leg ${leg + 1} min`);
      close(own[23].to, tip(max), `leg ${leg + 1} max`);
      for (let i = 1; i < 24; i++) assert.deepEqual(own[i].from, own[i - 1].to, 'the arc is continuous');
      // Every arc point is one horn length from the anchor.
      for (const line of own.slice(0, 24)) {
        assert.ok(Math.abs(Math.hypot(...line.to.map((v, k) => v - layout.baseAnchors[leg][k])) - layout.hornLength) < 1e-9);
      }
      // The marker crosses the arc exactly at the evaluator's horn tip.
      const marker = own[26];
      const crossing = marker.from.map((v, k) => v + (marker.to[k] - v) * (0.2 / 0.45));
      close(crossing, solved.hornTips[leg], `leg ${leg + 1} marker`);
    }
  }
  assert.equal(arcs.build({ ...accepted }, layout, null).lines.length, 6 * (perLeg - 1), 'no accepted angle, no marker');
});

test('servo arcs are neutral, tinted near a stop and failure-coloured on a requested servo violation', () => {
  const { accepted } = frozenSceneStates();
  const { layout, acceptedAssessment: solved } = accepted;
  const arcs = SCENE_BUILDERS.find(builder => builder.name === 'servoArcs');
  const legColors = state => {
    const { lines } = arcs.build(state, layout, solved);
    return [0, 1, 2, 3, 4, 5].map(leg => lines.slice(leg * 27, leg * 27 + 26).map(line => line.color));
  };
  const angles = solved.servoAngles;
  const [min, max] = effectiveServoRange(layout, accepted.options);
  assert.ok(angles.every(angle => Math.min(angle - min, max - angle) > NEAR_LIMIT_MARGIN_RAD), 'fixture starts clear of the stops');
  assert.ok(legColors(accepted).flat().every(color => color === SCENE_COLORS.limitRange));
  assert.equal(arcs.build(accepted, layout, solved).lines[26].color, SCENE_COLORS.horn);
  // Put the lowest leg 3° above its stop; the next lowest is then 5.5° clear.
  const lowest = angles.indexOf(Math.min(...angles));
  const near = { ...accepted, options: { ...accepted.options,
    servoRangeRad: [angles[lowest] - NEAR_LIMIT_MARGIN_RAD * 0.6, Math.max(...angles) + 1] } };
  legColors(near).forEach((colors, leg) => {
    const expected = leg === lowest ? SCENE_COLORS.nearLimit : SCENE_COLORS.limitRange;
    assert.ok(colors.every(color => color === expected), `leg ${leg + 1}`);
  });
  assert.equal(arcs.build(near, layout, solved).lines[lowest * 27 + 26].color, SCENE_COLORS.nearLimit);
  const failing = { ...accepted, assessment: { ...accepted.assessment, violations: [{ type: 'servoLimit', leg: 2, value: 2 }] } };
  legColors(failing).forEach((colors, leg) =>
    assert.ok(colors.every(color => color === (leg === 2 ? SCENE_COLORS.failure : SCENE_COLORS.limitRange)), `leg ${leg + 1}`));
  // A non-servo failure on the same leg leaves its arc neutral.
  const otherFailure = { ...accepted, assessment: { ...accepted.assessment, violations: [{ type: 'ballJoint', leg: 2 }] } };
  assert.ok(legColors(otherFailure).flat().every(color => color === SCENE_COLORS.limitRange));
  assert.equal(NEAR_LIMIT_MARGIN_RAD, 5 * Math.PI / 180);
});
