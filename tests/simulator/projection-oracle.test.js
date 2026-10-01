import test from 'node:test';
import assert from 'node:assert/strict';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { orbitCamera, pinhole } from '../fixtures/independent-geometry.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { cameraFrame, createWebGLRenderer, projectPoint, projectSegment } from '../../src/simulator/renderer.js';
import { SCENE_COLORS } from '../../src/simulator/scene.js';

// The renderer's projection checked against a pinhole camera written from its
// definition (tests/fixtures/independent-geometry.js), plus orientation facts
// that hold for any correct camera: screen handedness, square pixels and the
// 60 degree vertical field of view.

const close = (actual, expected, tolerance, label) =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${label}: ${actual} != ${expected}`);
const closeVector = (actual, expected, tolerance, label) =>
  expected.forEach((value, k) => close(actual[k], value, tolerance, `${label}[${k}]`));
const CAMERAS = [
  { yaw: 0.7, pitch: 0.38, distance: 600, target: [0, 0, 100] },
  { yaw: -2.4, pitch: -0.6, distance: 250, target: [30, -15, 180] },
  { yaw: 3.9, pitch: 1.35, distance: 1800, target: [-40, 60, 0] },
  { yaw: -Math.PI / 2, pitch: 0, distance: 500, target: [0, 0, 0] },
];

test('the orbit camera basis matches the hand-expanded look-at frame', () => {
  for (const camera of CAMERAS) {
    const actual = cameraFrame(camera), expected = orbitCamera(camera);
    for (const key of ['eye', 'forward', 'right', 'up']) closeVector(actual[key], expected[key], 1e-12, `${JSON.stringify(camera)} ${key}`);
  }
});

test('projected points match an independent pinhole camera for any aspect ratio', () => {
  let seed = 3;
  const random = () => (seed = (seed * 48271) % 2147483647) / 2147483647;
  for (const camera of CAMERAS) {
    for (const [width, height] of [[800, 500], [375, 640], [1, 1]]) {
      for (let k = 0; k < 50; k++) {
        const point = camera.target.map(value => value + (random() * 2 - 1) * camera.distance * 0.3);
        const expected = pinhole(point, camera, width, height);
        const actual = projectPoint(point, camera, width, height);
        closeVector(actual, expected.ndc, 1e-12, `${JSON.stringify(camera)} ${width}x${height} ${point}`);
      }
    }
  }
});

test('screen orientation: +X right and +Z up from the front, +Y up from above, X to Y turns counterclockwise', () => {
  const ndc = (point, camera) => projectPoint(point, camera, 800, 500);
  // Standing on -Y looking toward +Y: +X is to the right and +Z is up.
  const front = { yaw: -Math.PI / 2, pitch: 0, distance: 500, target: [0, 0, 0] };
  assert.ok(ndc([50, 0, 0], front)[0] > 0.1);
  close(ndc([50, 0, 0], front)[1], 0, 1e-12, 'front +X height');
  assert.ok(ndc([0, 0, 50], front)[1] > 0.1);
  close(ndc([0, 0, 50], front)[0], 0, 1e-12, 'front +Z sideways');
  closeVector(ndc([0, 50, 0], front), [0, 0], 1e-12, 'front +Y straight ahead');
  // Standing on +X looking toward -X: +Y is to the right.
  assert.ok(ndc([0, 50, 0], { ...front, yaw: 0 })[0] > 0.1);
  // Looking almost straight down from the -Y side: a map view, +X right and +Y up.
  const above = { ...front, pitch: 1.4 };
  assert.ok(ndc([50, 0, 0], above)[0] > 0.1);
  assert.ok(ndc([0, 50, 0], above)[1] > 0.1);
  // From any camera above the base plane, the world axes keep their right-handed
  // turn: the X tip to the Y tip goes counterclockwise on screen.
  for (const camera of CAMERAS.filter(item => item.pitch > 0)) {
    const [o, x, y] = [[0, 0, 0], [30, 0, 0], [0, 30, 0]].map(point => ndc(point, camera));
    const turn = (x[0] - o[0]) * (y[1] - o[1]) - (x[1] - o[1]) * (y[0] - o[0]);
    assert.ok(turn > 0, `${JSON.stringify(camera)} turns clockwise: ${turn}`);
  }
});

test('the vertical field of view is 60 degrees and pixels are square', () => {
  const camera = CAMERAS[0], frame = orbitCamera(camera);
  for (const [width, height] of [[800, 500], [300, 900]]) {
    const at = (right, up) => camera.target.map((value, k) => value + frame.right[k] * right + frame.up[k] * up);
    const depth = camera.distance, half = depth * Math.tan(Math.PI / 6);
    // A point 30 degrees above the view axis lands on the top edge; the side
    // edge sits at the same angle scaled by the aspect ratio.
    close(projectPoint(at(0, half), camera, width, height)[1], 1, 1e-12, 'top edge');
    close(projectPoint(at(half * width / height, 0), camera, width, height)[0], 1, 1e-12, 'right edge');
    // Equal offsets across and up the view cover equal pixel distances.
    const across = projectPoint(at(40, 0), camera, width, height)[0] * width / 2;
    const upward = projectPoint(at(0, 40), camera, width, height)[1] * height / 2;
    close(across, upward, 1e-9, `${width}x${height} pixel aspect`);
  }
});

test('perspective keeps a drawn segment straight: its 3D midpoint projects onto the drawn line', () => {
  const camera = CAMERAS[1];
  const from = [-60, 40, 120], to = [80, -30, 260];
  const [a, b] = projectSegment(from, to, camera, 800, 500);
  const m = projectPoint(from.map((value, k) => (value + to[k]) / 2), camera, 800, 500);
  close((b[0] - a[0]) * (m[1] - a[1]) - (b[1] - a[1]) * (m[0] - a[0]), 0, 1e-12, 'midpoint off the line');
  const t = (m[0] - a[0]) / (b[0] - a[0]);
  assert.ok(t > 0 && t < 1, `midpoint outside the segment: ${t}`);
});

test('the vertex buffer sent to the GPU puts every horn and rod at the pinhole projection of the solved points', () => {
  const uploads = [];
  const noop = () => {};
  const gl = {
    ARRAY_BUFFER: 1, LINES: 2, POINTS: 3, COMPILE_STATUS: 4, LINK_STATUS: 5,
    createShader: () => ({}), shaderSource: noop, compileShader: noop, getShaderParameter: () => true,
    createProgram: () => ({}), attachShader: noop, linkProgram: noop, getProgramParameter: () => true,
    createBuffer: () => ({}), getAttribLocation: () => 0, getUniformLocation: () => ({}), enable: noop,
    clearColor: noop, viewport: noop, clear: noop, useProgram: noop, bindBuffer: noop,
    vertexAttribPointer: noop, enableVertexAttribArray: noop, uniform1f: noop,
    bufferData(target, data) { uploads.push(Array.from(data)); },
    drawArrays(primitive) { uploads.at(-1).primitive = primitive; },
  };
  const canvas = { width: 640, height: 480, getContext: () => gl, addEventListener: noop, removeEventListener: noop };
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const state = controller.requestPose({ x: 12, y: -7, z: 18, rx: 0.06, ry: -0.04, rz: 0.1 });
  assert.equal(state.acceptedAssessment.reachable, true);
  const camera = { yaw: 1.1, pitch: 0.45, distance: 650, target: [0, 0, 180] };
  createWebGLRenderer(canvas).render(state, camera);

  const lines = uploads.find(upload => upload.primitive === gl.LINES);
  const segments = [];
  for (let k = 0; k < lines.length; k += 12) segments.push({ from: lines.slice(k, k + 2), to: lines.slice(k + 6, k + 8),
    color: lines.slice(k + 3, k + 6) });
  const sameColor = (a, b) => a.every((value, k) => Math.abs(value - b[k]) < 1e-6);
  const { layout } = state;
  const { hornTips, platformPoints } = state.acceptedAssessment;
  // Vertices travel as 32-bit floats.
  const near = (a, b) => Math.abs(a[0] - b[0]) < 1e-6 && Math.abs(a[1] - b[1]) < 1e-6;
  for (let leg = 0; leg < 6; leg++) {
    const [base, tip, top] = [layout.baseAnchors[leg], hornTips[leg], platformPoints[leg]]
      .map(point => pinhole(point, camera, canvas.width, canvas.height).ndc);
    assert.ok(segments.some(segment => sameColor(segment.color, SCENE_COLORS.horn) && near(segment.from, base) && near(segment.to, tip)),
      `leg ${leg} horn not drawn at its projected position`);
    assert.ok(segments.some(segment => sameColor(segment.color, SCENE_COLORS.rod) && near(segment.from, tip) && near(segment.to, top)),
      `leg ${leg} rod not drawn at its projected position`);
  }
});
