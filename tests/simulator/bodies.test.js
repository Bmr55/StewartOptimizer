import test from 'node:test';
import assert from 'node:assert/strict';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createFakeGL } from './helpers.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { BODY_DIMENSIONS, boxTransform, buildSolidBodies, CYLINDER_SEGMENTS, cylinderTransform, PLATE_GAP_MM,
  plateMesh, unitBoxMesh, unitCylinderMesh } from '../../src/simulator/bodies.js';
import { createWebGLRenderer, lightDirection, cameraFrame } from '../../src/simulator/renderer.js';
import { rodForceColor, rodForceReference, SCENE_COLORS } from '../../src/simulator/scene.js';
import { vectorDot, vectorMagnitude, vectorSub } from '../../src/math.js';

const settings = { ballJointLimitDeg: 180 };
const closeVector = (actual, expected, tolerance = 1e-9, label = '') => expected.forEach((value, k) =>
  assert.ok(Math.abs(actual[k] - value) <= tolerance, `${label} ${actual} != ${expected}`));
// Applies a column-major 4x4 transform to a point.
const apply = (m, p) => [0, 1, 2].map(row => m[row] * p[0] + m[4 + row] * p[1] + m[8 + row] * p[2] + m[12 + row]);
const triangles = mesh => Array.from({ length: mesh.length / 18 }, (_, t) => {
  const vertex = k => mesh.slice(t * 18 + k * 6, t * 18 + k * 6 + 6);
  return [0, 1, 2].map(vertex);
});
const centroidOf = points => [0, 1, 2].map(k => points.reduce((sum, point) => sum + point[k], 0) / points.length);

function loaded(extra = {}) {
  const controller = createSimulatorController();
  return { controller, state: controller.loadLayout(asymmetricJointFixture(), { options: settings, ...extra }) };
}

test('solid bodies are the plates, horn and rod cylinders and servo boxes of the accepted pose', () => {
  assert.equal(buildSolidBodies({ layout: null }), null);
  const { state } = loaded();
  const { layout, acceptedAssessment: solved } = state;
  const bodies = buildSolidBodies(state);
  assert.equal(bodies.cylinders.length, 12);
  assert.equal(bodies.boxes.length, 6);
  assert.equal(bodies.plates.length, 2);
  const horn = Math.max(1, layout.hornLength * BODY_DIMENSIONS.hornRadius);
  const rod = Math.max(1, layout.hornLength * BODY_DIMENSIONS.rodRadius);
  for (let i = 0; i < 6; i++) {
    assert.deepEqual(bodies.cylinders[2 * i], { from: layout.baseAnchors[i], to: solved.hornTips[i], radius: horn, color: SCENE_COLORS.horn });
    assert.deepEqual(bodies.cylinders[2 * i + 1], { from: solved.hornTips[i], to: solved.platformPoints[i], radius: rod, color: SCENE_COLORS.rod });
    assert.equal(bodies.boxes[i].color, SCENE_COLORS.servo);
    // The servo box sits beside the horn plane, off the shaft axis.
    const offset = vectorSub(bodies.boxes[i].center, layout.baseAnchors[i]);
    assert.ok(Math.abs(vectorDot(offset, bodies.boxes[i].axes[1])) > bodies.boxes[i].size[1] / 2);
  }
  // The base plate hangs below the anchors and the platform plate rises above its own.
  const [base, platform] = bodies.plates;
  assert.deepEqual(base.normal, [0, 0, -1]);
  assert.equal(base.offset, PLATE_GAP_MM);
  closeVector(platform.normal, solved.rotationMatrix.map(row => row[2]));
  assert.deepEqual([...base.outline].sort(), [...layout.baseAnchors].sort());
  // Without an accepted pose only the base and servos remain.
  const homeless = buildSolidBodies(loaded({ options: { ...settings, conditionLimit: 1 } }).state);
  assert.deepEqual([homeless.cylinders.length, homeless.boxes.length, homeless.plates.length], [0, 6, 1]);
});

test('solid legs keep failure colours and take graded rod colours from the loads overlay', () => {
  const { controller, state } = loaded({ loadModel: { mass_kg: 2.5 } });
  const off = buildSolidBodies(state);
  assert.ok(off.cylinders.filter((_, k) => k % 2).every(rod => rod.color === SCENE_COLORS.rod));
  const on = buildSolidBodies({ ...state, overlays: { ...state.overlays, loads: true } });
  on.cylinders.filter((_, k) => k % 2).forEach((rod, i) =>
    assert.deepEqual(rod.color, rodForceColor(state.loads.rodForceN[i], rodForceReference(state.loads))));
  // With the ghost off, a rejected request colours the failing held legs and servos.
  const rejected = controller.requestPose({ z: 500 });
  const failing = buildSolidBodies({ ...rejected, overlays: { ...rejected.overlays, requestedGhost: false } });
  const legs = new Set(rejected.assessment.violations.filter(violation => Number.isInteger(violation.leg)).map(violation => violation.leg));
  assert.ok(legs.size > 0);
  for (const leg of legs) {
    assert.equal(failing.cylinders[2 * leg].color, SCENE_COLORS.failure);
    assert.equal(failing.cylinders[2 * leg + 1].color, SCENE_COLORS.failure);
    assert.equal(failing.boxes[leg].color, SCENE_COLORS.failure);
  }
});

test('instance transforms place the unit meshes, whose normals point outward', () => {
  const cylinder = { from: [10, -5, 3], to: [40, 20, 90], radius: 2.5 };
  const m = cylinderTransform(cylinder);
  closeVector(apply(m, [0, 0, 0]), cylinder.from);
  closeVector(apply(m, [0, 0, 1]), cylinder.to);
  const axis = vectorSub(cylinder.to, cylinder.from);
  for (const angle of [0, 1, 2.5]) {
    const offset = vectorSub(apply(m, [Math.cos(angle), Math.sin(angle), 0]), cylinder.from);
    assert.ok(Math.abs(vectorMagnitude(offset) - cylinder.radius) < 1e-9);
    assert.ok(Math.abs(vectorDot(offset, axis)) < 1e-9);
  }
  const box = { center: [1, 2, 3], axes: [[0, 1, 0], [-1, 0, 0], [0, 0, 1]], size: [4, 6, 8] };
  closeVector(apply(boxTransform(box), [0.5, 0.5, 0.5]), [1 - 3, 2 + 2, 3 + 4]);
  closeVector(apply(boxTransform(box), [0, 0, 0]), box.center);

  const cylinderMesh = unitCylinderMesh();
  assert.equal(cylinderMesh.length / 6, CYLINDER_SEGMENTS * 12);
  for (const [a, b, c] of triangles(cylinderMesh)) {
    const middle = centroidOf([a, b, c]);
    for (const vertex of [a, b, c]) {
      assert.ok(Math.abs(vectorMagnitude(vertex.slice(3)) - 1) < 1e-12);
      // Side normals point away from the axis, cap normals out of the ends.
      const outward = vertex[5] === 0 ? [middle[0], middle[1], 0] : [0, 0, middle[2] - 0.5];
      assert.ok(vectorDot(vertex.slice(3), outward) > 0);
    }
  }
  const boxMesh = unitBoxMesh();
  assert.equal(boxMesh.length / 6, 36);
  for (const [a, b, c] of triangles(boxMesh)) {
    const middle = centroidOf([a, b, c]);
    assert.ok(vectorDot(a.slice(3), middle) > 0.49);
    // Each face is flat along its normal.
    for (const vertex of [a, b, c]) assert.ok(Math.abs(vectorDot(vertex.slice(0, 3), a.slice(3)) - 0.5) < 1e-12);
  }
});

test('a plate is a closed prism off the anchor plane with outward normals', () => {
  const { state } = loaded();
  const [base] = buildSolidBodies(state).plates;
  const mesh = plateMesh(base);
  assert.equal(mesh.length / 18, 4 * 6);
  const middle = [base.centroid[0], base.centroid[1], -(base.offset + base.thickness / 2)];
  for (const [a, b, c] of triangles(mesh)) {
    for (const vertex of [a, b, c]) {
      assert.ok(vertex[2] <= -base.offset + 1e-12 && vertex[2] >= -(base.offset + base.thickness) - 1e-12);
    }
    assert.ok(vectorDot(a.slice(3), vectorSub(centroidOf([a, b, c]).slice(0, 3), middle)) > 0, 'normal points inward');
  }
});

test('solid mode draws instanced bodies before the wireframe; wireframe mode draws none', () => {
  const { state } = loaded();
  const gl = createFakeGL();
  const canvas = { width: 400, height: 300, getContext: () => gl, addEventListener() {}, removeEventListener() {} };
  const renderer = createWebGLRenderer(canvas);
  const camera = { yaw: 0.5, pitch: 0.4, distance: 600, target: [0, 0, 100] };
  renderer.render(state, camera);
  assert.equal(gl.calls.draws.filter(draw => draw.instances).length, 0);
  gl.calls.draws.length = 0;
  renderer.render(state, camera, { mode: 'solid' });
  const instanced = gl.calls.draws.filter(draw => draw.instances);
  assert.deepEqual(instanced.map(draw => draw.instances), [12, 6, 1, 1]);
  assert.deepEqual(instanced.map(draw => draw.count).slice(0, 2), [CYLINDER_SEGMENTS * 12, 36]);
  assert.ok(gl.calls.draws.findIndex(draw => draw.primitive === gl.LINES) > gl.calls.draws.indexOf(instanced.at(-1)));
  // Instance data: a transform then a colour per instance.
  const cylinders = instanced[0].data;
  assert.equal(cylinders.length, 12 * 19);
  closeVector(cylinders.slice(16, 19), SCENE_COLORS.horn, 1e-6);
  closeVector(gl.calls.uniforms.uLight, lightDirection(cameraFrame(camera)), 1e-12);
  assert.ok(Math.abs(vectorMagnitude(gl.calls.uniforms.uLight) - 1) < 1e-12);
});
