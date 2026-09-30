import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { buildSceneGeometry, createWebGLRenderer, projectPoint } from '../../src/simulator/renderer.js';
import { OVERLAY_DEFAULTS, OVERLAY_NAMES, SCENE_BUILDERS } from '../../src/simulator/scene.js';

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
  assert.equal(buildSceneGeometry(rejected).lines.length, 15); // Base, servo directions, world axes.
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
// was split into builders (tests/fixtures/scene-geometry.json). The default
// toggles must keep drawing the same lines and points in the same order.
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

test('default overlay builders reproduce the frozen single-function scene exactly', () => {
  const states = frozenSceneStates();
  assert.equal(states.legFailure.rejected, true);
  assert.deepEqual(states.accepted.overlays, OVERLAY_DEFAULTS);
  for (const [name, state] of Object.entries(states)) {
    assert.deepEqual(plain(buildSceneGeometry(state)), frozenScene[name], name);
    // A state without an overlay map (older callers) draws the defaults.
    const { overlays, ...withoutOverlays } = state;
    assert.deepEqual(plain(buildSceneGeometry(withoutOverlays)), frozenScene[name], `${name} without overlays`);
  }
});

test('each overlay toggle removes only its own builder output', () => {
  const { accepted } = frozenSceneStates();
  const full = buildSceneGeometry(accepted);
  const parts = Object.fromEntries(SCENE_BUILDERS.map(builder =>
    [builder.name, builder.build(accepted, accepted.layout, accepted.acceptedAssessment)]));
  assert.deepEqual(SCENE_BUILDERS.map(builder => builder.name),
    ['base', 'platform', 'legs', 'platformAxes', 'worldAxes', 'trace']);
  assert.deepEqual(SCENE_BUILDERS.filter(builder => builder.overlay).map(builder => builder.overlay), OVERLAY_NAMES);
  assert.equal(parts.platformAxes.lines.length, 3);
  assert.equal(parts.worldAxes.lines.length, 3);
  assert.equal(parts.trace.lines.length, accepted.trace.length - 1);
  for (const name of OVERLAY_NAMES) {
    const scene = buildSceneGeometry({ ...accepted, overlays: { ...accepted.overlays, [name]: false } });
    const removed = new Set(parts[name].lines.map(line => JSON.stringify(line)));
    const expected = full.lines.filter(line => !removed.has(JSON.stringify(line)));
    assert.equal(scene.lines.length, full.lines.length - 3, name);
    assert.deepEqual(plain(scene.lines), plain(expected), name);
    assert.deepEqual(plain(scene.points), plain(full.points), name);
  }
  const bare = buildSceneGeometry({ ...accepted, overlays: { platformAxes: false, worldAxes: false } });
  assert.equal(bare.lines.length, full.lines.length - 6);
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
