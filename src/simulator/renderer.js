import { vectorAdd, vectorCross, vectorDot, vectorNormalize, vectorSub } from '../math.js';
import { buildSceneGeometry, SCENE_BACKGROUND } from './scene.js';

// The scene builders live in scene.js; re-exported for existing callers.
export { buildSceneGeometry };

// `depthBias` (mm) pulls only the depth value toward the camera, so a line can
// win the depth test against geometry lying almost on top of it without moving
// on screen.
// Tangent of half the vertical field of view (60°).
export const VIEW_HALF_TANGENT = Math.tan(Math.PI / 6);
// Geometry nearer the eye than this (mm along the view direction) is not drawn;
// lines crossing it are clipped there rather than dropped.
export const NEAR_PLANE_MM = 1;

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

const depthOf = (point, frame) => vectorDot(vectorSub(point, frame.eye), frame.forward);

function projectInFrame(point, frame, width, height, depthBias) {
  const offset = vectorSub(point, frame.eye);
  const depth = vectorDot(offset, frame.forward);
  if (depth <= NEAR_PLANE_MM) return null;
  const scale = 1 / (depth * VIEW_HALF_TANGENT);
  // The far plane follows the camera distance so a zoomed-out view (up to
  // 2,500 units) still spreads the scene across the depth range instead of
  // saturating every vertex at the clamp.
  const far = Math.max(2001, 2 * frame.distance + 1);
  return [vectorDot(offset, frame.right) * scale * height / width,
    vectorDot(offset, frame.up) * scale,
    Math.min(0.999, Math.max(-0.999, (depth - depthBias - NEAR_PLANE_MM) / (far - NEAR_PLANE_MM) * 2 - 1))];
}

// `depthBias` (mm) pulls only the depth value toward the camera, so a line can
// win the depth test against geometry lying almost on top of it without moving
// on screen.
export function projectPoint(point, camera, width, height, depthBias = 0) {
  return projectInFrame(point, cameraFrame(camera), width, height, depthBias);
}

// Projects a line, first clipping it where it crosses the near plane, so a
// close-up camera keeps the visible part of a line that passes beside it.
export function projectSegment(from, to, camera, width, height, depthBias = 0, frame = cameraFrame(camera)) {
  const clipAt = NEAR_PLANE_MM * (1 + 1e-6);
  const depthFrom = depthOf(from, frame), depthTo = depthOf(to, frame);
  if (depthFrom <= clipAt && depthTo <= clipAt) return null;
  const clip = (inside, outside, depthIn, depthOut) => {
    const t = (depthIn - clipAt) / (depthIn - depthOut);
    return inside.map((value, k) => value + (outside[k] - value) * t);
  };
  const start = depthFrom > clipAt ? from : clip(to, from, depthTo, depthFrom);
  const end = depthTo > clipAt ? to : clip(from, to, depthFrom, depthTo);
  const a = projectInFrame(start, frame, width, height, depthBias);
  const b = projectInFrame(end, frame, width, height, depthBias);
  return a && b ? [a, b] : null;
}

const VERTEX_SOURCE = `#version 300 es
in vec3 aPosition;
in vec3 aColor;
out vec3 vColor;
uniform float uPointSize;
void main() {
  gl_Position = vec4(aPosition, 1.0);
  gl_PointSize = uPointSize;
  vColor = aColor;
}`;
const FRAGMENT_SOURCE = `#version 300 es
precision mediump float;
in vec3 vColor;
out vec4 outColor;
void main() { outColor = vec4(vColor, 1.0); }`;

function shader(gl, type, source) {
  const result = gl.createShader(type);
  gl.shaderSource(result, source);
  gl.compileShader(result);
  if (!gl.getShaderParameter(result, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(result) || 'Shader compilation failed.');
  return result;
}

export function createWebGLRenderer(canvas, { window, onContextChange } = {}) {
  const gl = canvas.getContext?.('webgl2', { antialias: true, alpha: false });
  if (!gl) return { available: false, contextLost: false,
    error: 'WebGL2 is unavailable. Enable hardware acceleration or use a modern desktop browser; optimization remains available.',
    render() {}, dispose() {} };
  let program, buffer, position, color, pointSize;
  let lost = false;

  function setup() {
    program = gl.createProgram();
    gl.attachShader(program, shader(gl, gl.VERTEX_SHADER, VERTEX_SOURCE));
    gl.attachShader(program, shader(gl, gl.FRAGMENT_SHADER, FRAGMENT_SOURCE));
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program) || 'WebGL2 program failed to link.');
    buffer = gl.createBuffer();
    position = gl.getAttribLocation(program, 'aPosition');
    color = gl.getAttribLocation(program, 'aColor');
    pointSize = gl.getUniformLocation(program, 'uPointSize');
    gl.enable(gl.DEPTH_TEST);
    gl.clearColor(...SCENE_BACKGROUND, 1);
  }
  setup();

  // The browser restores a lost context (GPU reset, driver crash, backgrounded
  // tab, too many contexts) only when the lost event is cancelled. GPU objects
  // do not survive, so the program, buffer and locations are rebuilt on restore.
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

  function draw(vertices, primitive, size) {
    if (!vertices.length) return;
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(vertices), gl.DYNAMIC_DRAW);
    gl.vertexAttribPointer(position, 3, gl.FLOAT, false, 24, 0);
    gl.vertexAttribPointer(color, 3, gl.FLOAT, false, 24, 12);
    gl.enableVertexAttribArray(position);
    gl.enableVertexAttribArray(color);
    gl.uniform1f(pointSize, size);
    gl.drawArrays(primitive, 0, vertices.length / 6);
  }

  function render(state, camera = {}) {
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
    gl.useProgram(program);
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    const geometry = buildSceneGeometry(state);
    const frame = cameraFrame(camera);
    const lineData = [];
    for (const line of geometry.lines) {
      const segment = projectSegment(line.from, line.to, camera, width, height, line.depthBias, frame);
      if (segment) lineData.push(...segment[0], ...line.color, ...segment[1], ...line.color);
    }
    draw(lineData, gl.LINES, 1);
    const pointData = [];
    for (const point of geometry.points) {
      const projected = projectInFrame(point.at, frame, width, height, 0);
      if (projected) pointData.push(...projected, ...point.color);
    }
    draw(pointData, gl.POINTS, 7);
  }

  return { available: true, get contextLost() { return lost; }, render,
    dispose() {
      canvas.removeEventListener?.('webglcontextlost', contextLost);
      canvas.removeEventListener?.('webglcontextrestored', contextRestored);
      gl.deleteBuffer(buffer);
      gl.deleteProgram(program);
    } };
}
