import { vectorAdd, vectorCross, vectorDot, vectorNormalize, vectorScale, vectorSub } from '../math.js';
import { buildSceneGeometry, SCENE_BACKGROUND } from './scene.js';
import { boxTransform, buildSolidBodies, cylinderTransform, plateMesh, unitBoxMesh, unitCylinderMesh } from './bodies.js';

// The scene builders live in scene.js; re-exported for existing callers.
export { buildSceneGeometry };

// Tangent of half the vertical field of view (60°).
export const VIEW_HALF_TANGENT = Math.tan(Math.PI / 6);
// Geometry nearer the eye than this (mm along the view direction) is not drawn;
// lines crossing it are clipped there rather than dropped.
export const NEAR_PLANE_MM = 1;
// Pixel size of a point whose builder gives none.
export const DEFAULT_POINT_SIZE = 7;
// The render modes: lines and points only, or lit solid bodies under them.
export const RENDER_MODES = Object.freeze(['wireframe', 'solid']);
// Solid bodies are lit by one directional light set relative to the camera
// (from above, behind and left of the viewer) plus this ambient fraction.
export const SOLID_AMBIENT = 0.35;

// Eye position and view basis of the orbit camera: it looks at `target` from
// `distance` along the yaw/pitch direction, with +Z up.
export function cameraFrame(camera) {
  const target = camera.target ?? [0, 0, 100];
  const yaw = camera.yaw ?? 0.7;
  const pitch = camera.pitch ?? 0.4;
  const distance = camera.distance ?? 600;
  const eye = vectorAdd(target, [distance * Math.cos(pitch) * Math.cos(yaw),
    distance * Math.cos(pitch) * Math.sin(yaw), distance * Math.sin(pitch)]);
  const forward = vectorNormalize(vectorSub(target, eye));
  const right = vectorNormalize(vectorCross(forward, [0, 0, 1]));
  const up = vectorCross(right, forward);
  return { target, distance, eye, forward, right, up };
}

// The view-projection the CPU and GPU share. `rows` maps a world point (mm) to
// clip coordinates [x, y, z, w]: w is the depth along the view direction, x
// and y give the 60° perspective with square pixels, and the z row is zero
// because clip z is filled from w by `depthNdc`, which keeps depth linear in
// view depth. `matrix` is the same map column-major for uniformMatrix4fv.
// The far plane follows the camera distance so a zoomed-out view (up to
// 2,500 units) still spreads the scene across the depth range instead of
// saturating every vertex at the clamp.
export function viewProjection(camera, width, height, frame = cameraFrame(camera)) {
  const sx = height / width / VIEW_HALF_TANGENT, sy = 1 / VIEW_HALF_TANGENT;
  const row = (axis, scale) => [...vectorScale(axis, scale), -vectorDot(axis, frame.eye) * scale];
  const rows = [row(frame.right, sx), row(frame.up, sy), [0, 0, 0, 0], row(frame.forward, 1)];
  const matrix = [0, 1, 2, 3].flatMap(col => rows.map(values => values[col]));
  return { frame, rows, matrix, near: NEAR_PLANE_MM, far: Math.max(2001, 2 * frame.distance + 1) };
}

const clipOf = (rows, point) => rows.map(values => values[0] * point[0] + values[1] * point[1] + values[2] * point[2] + values[3]);

// Normalized depth for a view depth `w`. `depthBias` (mm) pulls only the depth
// value toward the camera, so a line can win the depth test against geometry
// lying almost on top of it without moving on screen. The vertex shaders
// compute the same expression.
export function depthNdc(w, depthBias, { near, far }) {
  return Math.min(0.999, Math.max(-0.999, (w - depthBias - near) / (far - near) * 2 - 1));
}

function projectWith(point, projection, depthBias) {
  const [x, y, , w] = clipOf(projection.rows, point);
  if (w <= projection.near) return null;
  return [x / w, y / w, depthNdc(w, depthBias, projection)];
}

// Normalized device coordinates of a world point, or null when it is not in
// front of the near plane. This is the GPU's projection evaluated on the CPU
// for picking and tests, from the same matrix the shaders receive.
export function projectPoint(point, camera, width, height, depthBias = 0) {
  return projectWith(point, viewProjection(camera, width, height), depthBias);
}

// The part of a line in front of the near plane, in world coordinates, or null
// when none is. A close-up camera keeps the visible part of a line that passes
// beside it.
export function clipSegment(from, to, projection) {
  const clipAt = projection.near * (1 + 1e-6);
  const depthFrom = clipOf(projection.rows, from)[3], depthTo = clipOf(projection.rows, to)[3];
  if (depthFrom <= clipAt && depthTo <= clipAt) return null;
  const clip = (inside, outside, depthIn, depthOut) => {
    const t = (depthIn - clipAt) / (depthIn - depthOut);
    return inside.map((value, k) => value + (outside[k] - value) * t);
  };
  return [depthFrom > clipAt ? from : clip(to, from, depthTo, depthFrom),
    depthTo > clipAt ? to : clip(from, to, depthFrom, depthTo)];
}

// Projects a line after clipping it at the near plane. `projection` may be
// passed in when projecting many lines with one camera.
export function projectSegment(from, to, camera, width, height, depthBias = 0,
  projection = viewProjection(camera, width, height)) {
  const clipped = clipSegment(from, to, projection);
  if (!clipped) return null;
  const a = projectWith(clipped[0], projection, depthBias);
  const b = projectWith(clipped[1], projection, depthBias);
  return a && b ? [a, b] : null;
}

// World direction toward the solid-body light for a camera frame.
export function lightDirection(frame) {
  return vectorNormalize(vectorAdd(vectorAdd(vectorScale(frame.forward, -0.6), vectorScale(frame.up, 0.7)),
    vectorScale(frame.right, -0.4)));
}

// Both programs project with the shared matrix and fill clip z from w with the
// same expression as depthNdc.
const PROJECT_SOURCE = `uniform mat4 uViewProjection;
uniform vec2 uDepthRange;
vec4 project(vec3 world, float depthBias) {
  vec4 clip = uViewProjection * vec4(world, 1.0);
  float depth = clamp((clip.w - depthBias - uDepthRange.x) / (uDepthRange.y - uDepthRange.x) * 2.0 - 1.0, -0.999, 0.999);
  return vec4(clip.xy, depth * clip.w, clip.w);
}`;
const LINE_VERTEX_SOURCE = `#version 300 es
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
layout(location = 2) in float aDepthBias;
out vec3 vColor;
uniform float uPointSize;
${PROJECT_SOURCE}
void main() {
  gl_Position = project(aPosition, aDepthBias);
  gl_PointSize = uPointSize;
  vColor = aColor;
}`;
// Unit meshes drawn once per instance; the per-instance transform arrives as
// four column attributes. Normals use the inverse transpose of the transform,
// which for a rotation times a scale is the transform divided by each squared scale.
const SOLID_VERTEX_SOURCE = `#version 300 es
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
layout(location = 3) in vec4 aModel0;
layout(location = 4) in vec4 aModel1;
layout(location = 5) in vec4 aModel2;
layout(location = 6) in vec4 aModel3;
out vec3 vColor;
uniform vec3 uLight;
uniform float uAmbient;
${PROJECT_SOURCE}
void main() {
  mat4 model = mat4(aModel0, aModel1, aModel2, aModel3);
  gl_Position = project((model * vec4(aPosition, 1.0)).xyz, 0.0);
  vec3 squared = max(vec3(dot(aModel0.xyz, aModel0.xyz), dot(aModel1.xyz, aModel1.xyz), dot(aModel2.xyz, aModel2.xyz)), vec3(1e-12));
  vec3 normal = normalize(mat3(model) * (aNormal / squared));
  vColor = aColor * (uAmbient + (1.0 - uAmbient) * max(dot(normal, uLight), 0.0));
}`;
const FRAGMENT_SOURCE = `#version 300 es
precision mediump float;
in vec3 vColor;
out vec4 outColor;
void main() { outColor = vec4(vColor, 1.0); }`;

const LINE_STRIDE = 7;
const MESH_STRIDE = 6;
const INSTANCE_STRIDE = 19;
const IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

function shader(gl, type, source) {
  const result = gl.createShader(type);
  gl.shaderSource(result, source);
  gl.compileShader(result);
  if (!gl.getShaderParameter(result, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(result) || 'Shader compilation failed.');
  return result;
}

function link(gl, vertexSource) {
  const program = gl.createProgram();
  gl.attachShader(program, shader(gl, gl.VERTEX_SHADER, vertexSource));
  gl.attachShader(program, shader(gl, gl.FRAGMENT_SHADER, FRAGMENT_SOURCE));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program) || 'WebGL2 program failed to link.');
  return program;
}

// Per-instance data: the column-major transform, then the colour.
const instance = (transform, color) => [...transform, ...color];

export function createWebGLRenderer(canvas, { window, onContextChange } = {}) {
  const gl = canvas.getContext?.('webgl2', { antialias: true, alpha: false });
  if (!gl) return { available: false, contextLost: false,
    error: 'WebGL2 is unavailable. Enable hardware acceleration or use a modern desktop browser; optimization remains available.',
    render() {}, dispose() {} };
  let lines, solids, meshes;
  let lost = false;

  const uniforms = (program, names) => Object.fromEntries(names.map(name => [name, gl.getUniformLocation(program, name)]));
  function setup() {
    const lineProgram = link(gl, LINE_VERTEX_SOURCE);
    const solidProgram = link(gl, SOLID_VERTEX_SOURCE);
    lines = { program: lineProgram, buffer: gl.createBuffer(), vao: gl.createVertexArray(),
      uniforms: uniforms(lineProgram, ['uViewProjection', 'uDepthRange', 'uPointSize']) };
    gl.bindVertexArray(lines.vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, lines.buffer);
    for (const [location, count, offset] of [[0, 3, 0], [1, 3, 3], [2, 1, 6]]) {
      gl.enableVertexAttribArray(location);
      gl.vertexAttribPointer(location, count, gl.FLOAT, false, LINE_STRIDE * 4, offset * 4);
    }
    solids = { program: solidProgram, uniforms: uniforms(solidProgram, ['uViewProjection', 'uDepthRange', 'uLight', 'uAmbient']) };
    // Each mesh has its own vertex array: a vertex buffer (static for the unit
    // meshes, rewritten each frame for the plates) and an instance buffer.
    const mesh = vertices => {
      const result = { vao: gl.createVertexArray(), vertices: gl.createBuffer(), instances: gl.createBuffer(),
        count: vertices ? vertices.length / MESH_STRIDE : 0 };
      gl.bindVertexArray(result.vao);
      gl.bindBuffer(gl.ARRAY_BUFFER, result.vertices);
      if (vertices) gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(vertices), gl.STATIC_DRAW);
      for (const [location, offset] of [[0, 0], [1, 3]]) {
        gl.enableVertexAttribArray(location);
        gl.vertexAttribPointer(location, 3, gl.FLOAT, false, MESH_STRIDE * 4, offset * 4);
      }
      gl.bindBuffer(gl.ARRAY_BUFFER, result.instances);
      for (const [location, count, offset] of [[2, 3, 16], [3, 4, 0], [4, 4, 4], [5, 4, 8], [6, 4, 12]]) {
        gl.enableVertexAttribArray(location);
        gl.vertexAttribPointer(location, count, gl.FLOAT, false, INSTANCE_STRIDE * 4, offset * 4);
        gl.vertexAttribDivisor(location, 1);
      }
      return result;
    };
    meshes = { cylinder: mesh(unitCylinderMesh()), box: mesh(unitBoxMesh()), plate: mesh(null) };
    gl.bindVertexArray(null);
    gl.enable(gl.DEPTH_TEST);
    gl.clearColor(...SCENE_BACKGROUND, 1);
  }
  setup();

  function release() {
    for (const item of [meshes.cylinder, meshes.box, meshes.plate]) {
      gl.deleteBuffer(item.vertices);
      gl.deleteBuffer(item.instances);
      gl.deleteVertexArray(item.vao);
    }
    gl.deleteBuffer(lines.buffer);
    gl.deleteVertexArray(lines.vao);
    gl.deleteProgram(lines.program);
    gl.deleteProgram(solids.program);
  }

  // The browser restores a lost context (GPU reset, driver crash, backgrounded
  // tab, too many contexts) only when the lost event is cancelled. GPU objects
  // do not survive, so programs, buffers and vertex arrays are rebuilt on restore.
  const contextLost = event => {
    event?.preventDefault?.();
    lost = true;
    onContextChange?.();
  };
  const contextRestored = () => {
    try { setup(); lost = false; }
    catch { return; } // Stay lost; the status keeps reporting it.
    onContextChange?.();
  };
  canvas.addEventListener?.('webglcontextlost', contextLost);
  canvas.addEventListener?.('webglcontextrestored', contextRestored);

  function setProjection(program, projection) {
    gl.uniformMatrix4fv(program.uniforms.uViewProjection, false, new Float32Array(projection.matrix));
    gl.uniform2f(program.uniforms.uDepthRange, projection.near, projection.far);
  }

  function drawLines(vertices, primitive, size) {
    if (!vertices.length) return;
    gl.bindBuffer(gl.ARRAY_BUFFER, lines.buffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(vertices), gl.DYNAMIC_DRAW);
    gl.uniform1f(lines.uniforms.uPointSize, size);
    gl.drawArrays(primitive, 0, vertices.length / LINE_STRIDE);
  }

  function drawInstances(item, instances, vertexCount = item.count) {
    if (!instances.length || !vertexCount) return;
    gl.bindVertexArray(item.vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, item.instances);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(instances), gl.DYNAMIC_DRAW);
    gl.drawArraysInstanced(gl.TRIANGLES, 0, vertexCount, instances.length / INSTANCE_STRIDE);
  }

  function drawSolids(state, projection) {
    const bodies = buildSolidBodies(state);
    if (!bodies) return;
    gl.useProgram(solids.program);
    setProjection(solids, projection);
    gl.uniform3f(solids.uniforms.uLight, ...lightDirection(projection.frame));
    gl.uniform1f(solids.uniforms.uAmbient, SOLID_AMBIENT);
    drawInstances(meshes.cylinder, bodies.cylinders.flatMap(body => instance(cylinderTransform(body), body.color)));
    drawInstances(meshes.box, bodies.boxes.flatMap(body => instance(boxTransform(body), body.color)));
    // Plates are world-space meshes drawn as one identity instance each.
    for (const plate of bodies.plates) {
      const vertices = plateMesh(plate);
      gl.bindVertexArray(meshes.plate.vao);
      gl.bindBuffer(gl.ARRAY_BUFFER, meshes.plate.vertices);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(vertices), gl.DYNAMIC_DRAW);
      drawInstances(meshes.plate, instance(IDENTITY, plate.color), vertices.length / MESH_STRIDE);
    }
    gl.bindVertexArray(null);
  }

  // `mode` is one of RENDER_MODES; solid draws the bodies first and the
  // wireframe scene (every overlay included) over them with the same depth.
  function render(state, camera = {}, { mode = 'wireframe' } = {}) {
    if (lost || gl.isContextLost?.()) return;
    const pixelRatio = Math.min(2, window?.devicePixelRatio || 1);
    const width = Math.max(1, Math.floor((canvas.clientWidth || canvas.width) * pixelRatio));
    const height = Math.max(1, Math.floor((canvas.clientHeight || canvas.height) * pixelRatio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    gl.viewport(0, 0, width, height);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    const projection = viewProjection(camera, width, height);
    if (mode === 'solid') drawSolids(state, projection);
    gl.useProgram(lines.program);
    gl.bindVertexArray(lines.vao);
    setProjection(lines, projection);
    const geometry = buildSceneGeometry(state);
    const lineData = [];
    for (const line of geometry.lines) {
      const segment = clipSegment(line.from, line.to, projection);
      const bias = line.depthBias ?? 0;
      if (segment) lineData.push(...segment[0], ...line.color, bias, ...segment[1], ...line.color, bias);
    }
    drawLines(lineData, gl.LINES, 1);
    // One draw per point size, so reachability samples stay smaller than markers.
    const pointData = new Map();
    for (const point of geometry.points) {
      if (!(clipOf(projection.rows, point.at)[3] > projection.near)) continue;
      const size = point.size ?? DEFAULT_POINT_SIZE;
      if (!pointData.has(size)) pointData.set(size, []);
      pointData.get(size).push(...point.at, ...point.color, 0);
    }
    for (const [size, vertices] of pointData) drawLines(vertices, gl.POINTS, size);
    gl.bindVertexArray(null);
  }

  return { available: true, get contextLost() { return lost; }, render,
    dispose() {
      canvas.removeEventListener?.('webglcontextlost', contextLost);
      canvas.removeEventListener?.('webglcontextrestored', contextRestored);
      release();
    } };
}
