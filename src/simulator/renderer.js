import { vectorAdd, vectorCross, vectorDot, vectorNormalize, vectorScale, vectorSub } from '../math.js';

const COLORS = Object.freeze({
  base: [0.24, 0.64, 0.94], platform: [0.43, 0.88, 0.72],
  horn: [1, 0.69, 0.31], rod: [0.88, 0.89, 0.94],
  servo: [1, 0.42, 0.39], trace: [0.72, 0.5, 1],
  x: [1, 0.38, 0.38], y: [0.39, 0.92, 0.47], z: [0.42, 0.62, 1],
});

function polygon(lines, points, color) {
  points.forEach((point, i) => lines.push({ from: point, to: points[(i + 1) % points.length], color }));
}

export function buildSceneGeometry(state) {
  const { layout, acceptedAssessment: solved, markers = true, trace = [] } = state;
  const lines = [], points = [];
  if (!layout) return { lines, points };
  polygon(lines, layout.baseAnchors, COLORS.base);
  for (let i = 0; i < 6; i++) {
    const base = layout.baseAnchors[i];
    const beta = layout.betaAngles[i];
    const direction = [Math.cos(beta), Math.sin(beta), 0];
    lines.push({ from: base, to: vectorAdd(base, vectorScale(direction, Math.max(12, layout.hornLength * 0.35))), color: COLORS.servo });
    if (markers) points.push({ at: base, color: COLORS.servo, size: 7 });
  }
  if (solved?.platformPoints?.length === 6 && solved?.hornTips?.length === 6) {
    polygon(lines, solved.platformPoints, COLORS.platform);
    for (let i = 0; i < 6; i++) {
      lines.push({ from: layout.baseAnchors[i], to: solved.hornTips[i], color: COLORS.horn });
      lines.push({ from: solved.hornTips[i], to: solved.platformPoints[i], color: COLORS.rod });
      if (markers) {
        points.push({ at: solved.hornTips[i], color: COLORS.horn, size: 6 });
        points.push({ at: solved.platformPoints[i], color: COLORS.platform, size: 7 });
      }
    }
    const center = solved.translation;
    const axis = Math.max(18, layout.hornLength * 0.35);
    for (const [direction, color] of [[solved.rotationMatrix[0].map((_, row) => solved.rotationMatrix[row][0]), COLORS.x],
      [solved.rotationMatrix[0].map((_, row) => solved.rotationMatrix[row][1]), COLORS.y],
      [solved.rotationMatrix[0].map((_, row) => solved.rotationMatrix[row][2]), COLORS.z]]) {
      lines.push({ from: center, to: vectorAdd(center, vectorScale(direction, axis)), color });
    }
  }
  const origin = [0, 0, 0];
  for (const [direction, color] of [[[30, 0, 0], COLORS.x], [[0, 30, 0], COLORS.y], [[0, 0, 30], COLORS.z]]) {
    lines.push({ from: origin, to: direction, color });
  }
  for (let i = 1; i < trace.length; i++) lines.push({ from: trace[i - 1], to: trace[i], color: COLORS.trace });
  return { lines, points };
}

export function projectPoint(point, camera, width, height) {
  const target = camera.target ?? [0, 0, 100];
  const yaw = camera.yaw ?? 0.7;
  const pitch = camera.pitch ?? 0.4;
  const distance = camera.distance ?? 600;
  const eye = vectorAdd(target, [distance * Math.cos(pitch) * Math.cos(yaw),
    distance * Math.cos(pitch) * Math.sin(yaw), distance * Math.sin(pitch)]);
  const forward = vectorNormalize(vectorSub(target, eye));
  const right = vectorNormalize(vectorCross(forward, [0, 0, 1]));
  const up = vectorCross(right, forward);
  const offset = vectorSub(point, eye);
  const depth = vectorDot(offset, forward);
  if (depth <= 1) return null;
  const scale = 1 / (depth * Math.tan(Math.PI / 6));
  return [vectorDot(offset, right) * scale * height / width,
    vectorDot(offset, up) * scale,
    Math.min(0.999, Math.max(-0.999, (depth - 1) / 2000 * 2 - 1))];
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

export function createWebGLRenderer(canvas, { window } = {}) {
  const gl = canvas.getContext?.('webgl2', { antialias: true, alpha: false });
  if (!gl) return { available: false,
    error: 'WebGL2 is unavailable. Enable hardware acceleration or use a modern desktop browser; optimization remains available.',
    render() {}, dispose() {} };
  const program = gl.createProgram();
  gl.attachShader(program, shader(gl, gl.VERTEX_SHADER, VERTEX_SOURCE));
  gl.attachShader(program, shader(gl, gl.FRAGMENT_SHADER, FRAGMENT_SOURCE));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program) || 'WebGL2 program failed to link.');
  const buffer = gl.createBuffer();
  const position = gl.getAttribLocation(program, 'aPosition');
  const color = gl.getAttribLocation(program, 'aColor');
  const pointSize = gl.getUniformLocation(program, 'uPointSize');
  gl.enable(gl.DEPTH_TEST);
  gl.clearColor(0.055, 0.075, 0.11, 1);

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
    const lineData = [];
    for (const line of geometry.lines) {
      const from = projectPoint(line.from, camera, width, height);
      const to = projectPoint(line.to, camera, width, height);
      if (from && to) lineData.push(...from, ...line.color, ...to, ...line.color);
    }
    draw(lineData, gl.LINES, 1);
    const pointData = [];
    for (const point of geometry.points) {
      const projected = projectPoint(point.at, camera, width, height);
      if (projected) pointData.push(...projected, ...point.color);
    }
    draw(pointData, gl.POINTS, 7);
  }

  return { available: true, render, dispose() { gl.deleteBuffer(buffer); gl.deleteProgram(program); } };
}
