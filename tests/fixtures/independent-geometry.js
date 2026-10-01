// Geometry written from the documented conventions alone, for tests that check
// src/ against something other than itself. Nothing here imports src/: each
// formula is spelled out from its definition so a shared bug cannot cancel out.

const { cos, sin, tan, hypot, PI } = Math;

// Right-handed elementary rotations: a positive angle turns counterclockwise
// when looking back down the axis toward the origin.
export const rotX = a => [[1, 0, 0], [0, cos(a), -sin(a)], [0, sin(a), cos(a)]];
export const rotY = a => [[cos(a), 0, sin(a)], [0, 1, 0], [-sin(a), 0, cos(a)]];
export const rotZ = a => [[cos(a), -sin(a), 0], [sin(a), cos(a), 0], [0, 0, 1]];

export const matMul = (a, b) => a.map(row => [0, 1, 2].map(j => row[0] * b[0][j] + row[1] * b[1][j] + row[2] * b[2][j]));
export const matVec = (m, v) => m.map(row => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);
export const add = (a, b) => a.map((value, k) => value + b[k]);
export const sub = (a, b) => a.map((value, k) => value - b[k]);
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const norm = v => hypot(v[0], v[1], v[2]);

// Platform orientation as docs/architecture.md and docs/JOINT_MODEL.md state it:
// R = Rz(rz) Ry(ry) Rx(rx), so roll is applied first and yaw last (about the
// fixed base axes).
export const eulerZYX = (rx, ry, rz) => matMul(rotZ(rz), matMul(rotY(ry), rotX(rx)));

// World position of platform anchor `anchor` at `pose` (mm, radians).
export function platformPoint(layout, pose, anchor) {
  const R = eulerZYX(pose.rx ?? 0, pose.ry ?? 0, pose.rz ?? 0);
  return add([pose.x ?? 0, pose.y ?? 0, (pose.z ?? 0) + (layout.homeHeight ?? 0)], matVec(R, anchor));
}

// Horn tip from docs/JOINT_MODEL.md: the horn turns in the vertical plane
// through its base anchor that contains the horizontal direction beta, and a
// positive servo angle raises the tip.
export const hornTip = (base, beta, hornLength, alpha) =>
  add(base, [hornLength * cos(alpha) * cos(beta), hornLength * cos(alpha) * sin(beta), hornLength * sin(alpha)]);

// Every servo angle in (-pi, pi] at which a rod of `rodLength` joins the horn
// tip to `point`, found by a sign-change scan and bisection rather than the
// closed form the evaluator uses. Each root reports the slope of the squared
// rod gap |point - tip|^2 there, which tells the two branches apart.
export function servoAngleRoots(base, beta, hornLength, rodLength, point, steps = 1440) {
  const gap = alpha => {
    const span = sub(point, hornTip(base, beta, hornLength, alpha));
    return dot(span, span) - rodLength * rodLength;
  };
  const roots = [];
  let previousAlpha = -PI, previous = gap(previousAlpha);
  for (let k = 1; k <= steps; k++) {
    const alpha = -PI + 2 * PI * k / steps;
    const value = gap(alpha);
    if (previous === 0) roots.push(previousAlpha);
    else if (previous * value < 0) {
      let low = previousAlpha, high = alpha, lowValue = previous;
      for (let i = 0; i < 80; i++) {
        const mid = (low + high) / 2, midValue = gap(mid);
        if (lowValue * midValue <= 0) high = mid;
        else { low = mid; lowValue = midValue; }
      }
      roots.push((low + high) / 2);
    }
    previousAlpha = alpha;
    previous = value;
  }
  const h = 1e-6;
  return roots.map(alpha => ({ alpha, slope: (gap(alpha + h) - gap(alpha - h)) / (2 * h) }));
}

// Orbit camera as docs/SIMULATOR.md (Rendering checks) defines it: the eye sits
// `distance` from `target` at azimuth `yaw` (from +X toward +Y) and elevation
// `pitch`, looking at the target with +Z up. The basis is the hand-expanded
// look-at result: right is horizontal and perpendicular to the azimuth, up is
// the world Z axis tilted back by the pitch.
export function orbitCamera({ yaw, pitch, distance, target }) {
  const [cy, sy, cp, sp] = [cos(yaw), sin(yaw), cos(pitch), sin(pitch)];
  return {
    eye: add(target, [distance * cp * cy, distance * cp * sy, distance * sp]),
    forward: [-cp * cy, -cp * sy, -sp],
    right: [-sy, cy, 0],
    up: [-sp * cy, -sp * sy, cp],
  };
}

// Pinhole projection with a 60 degree vertical field of view and square
// pixels, to normalized device coordinates (x right, y up, both -1..1 across
// the viewport) and to GL window pixels (origin at the bottom-left corner).
export const VERTICAL_FOV_RAD = PI / 3;
export function pinhole(point, camera, width, height) {
  const frame = orbitCamera(camera);
  const offset = sub(point, frame.eye);
  const depth = dot(offset, frame.forward);
  const halfHeight = depth * tan(VERTICAL_FOV_RAD / 2);
  const halfWidth = halfHeight * width / height;
  const ndc = [dot(offset, frame.right) / halfWidth, dot(offset, frame.up) / halfHeight];
  return { depth, ndc, pixel: [(ndc[0] + 1) / 2 * width, (ndc[1] + 1) / 2 * height] };
}
