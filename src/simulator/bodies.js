import { rotateVector, vectorAdd, vectorCross, vectorDot, vectorMagnitude, vectorNormalize, vectorScale,
  vectorSub } from '../math.js';
import { hornFrameAxes } from '../model/kinematics.js';
import { failureColor, hasSolvedLegs, OVERLAY_DEFAULTS, rodForceColor, rodForceReference, SCENE_COLORS } from './scene.js';

// Solid display volumes for the Solid render mode: plates, horn and rod
// cylinders and servo boxes at the accepted pose. They are display volumes
// only, sized by these constants (multiples of horn length, with a floor in
// mm), not layout fields: nothing here is exported with a layout, enters the
// evaluator or checks collisions.
export const BODY_DIMENSIONS = Object.freeze({
  hornRadius: 0.06, rodRadius: 0.04, plateThickness: 0.08,
  // Servo box length (along the base direction), depth (along the shaft) and height.
  servoBox: Object.freeze([0.8, 0.45, 0.5]),
  // Gap between the horn plane and the servo box face.
  servoGap: 0.08,
});
export const BODY_MIN_SIZE_MM = 1;
// Plates sit this far (mm) off the anchor plane, below the base anchors and
// above the platform anchors, so the wireframe outlines drawn on that plane
// stay visible on the near side instead of tying with a plate face.
export const PLATE_GAP_MM = 0.5;
export const CYLINDER_SEGMENTS = 16;

const size = (layout, factor) => Math.max(BODY_MIN_SIZE_MM, layout.hornLength * factor);

// Outline points ordered by angle about their centroid in the plane spanned by
// `u` and `v`, so a fan from the centroid fills a star-shaped hexagon whatever
// order the anchors are listed in.
function ordered(points, u, v) {
  const centroid = points.reduce((sum, point) => vectorAdd(sum, vectorScale(point, 1 / points.length)), [0, 0, 0]);
  const angle = point => Math.atan2(vectorDot(vectorSub(point, centroid), v), vectorDot(vectorSub(point, centroid), u));
  return { centroid, outline: points.slice().sort((a, b) => angle(a) - angle(b)) };
}

// The bodies to draw for a scene state, or null without a layout. Colours
// follow the wireframe: a leg the held pose failure-colours keeps that colour,
// and with the loads overlay on and solved, rods take their graded force colour.
export function buildSolidBodies(state) {
  const { layout } = state;
  if (!layout) return null;
  const solved = state.acceptedAssessment ?? null;
  const overlays = { ...OVERLAY_DEFAULTS, ...state.overlays };
  const legColor = failureColor({ ...state, overlays });
  const loads = overlays.loads && state.loads?.valid ? state.loads : null;
  const cylinders = [], boxes = [], plates = [];
  const thickness = size(layout, BODY_DIMENSIONS.plateThickness);
  const base = ordered(layout.baseAnchors, [1, 0, 0], [0, 1, 0]);
  plates.push({ ...base, normal: [0, 0, -1], offset: PLATE_GAP_MM, thickness, color: SCENE_COLORS.base });
  const [length, depth, height] = BODY_DIMENSIONS.servoBox.map(factor => size(layout, factor));
  for (let i = 0; i < 6; i++) {
    const [radial, shaft] = hornFrameAxes(layout.betaAngles[i], 0);
    const center = vectorAdd(layout.baseAnchors[i], vectorScale(shaft, -(depth / 2 + size(layout, BODY_DIMENSIONS.servoGap))));
    boxes.push({ center, axes: [radial, shaft, [0, 0, 1]], size: [length, depth, height], color: legColor(i) ?? SCENE_COLORS.servo });
  }
  if (hasSolvedLegs(solved)) {
    const normal = rotateVector(solved.rotationMatrix, [0, 0, 1]);
    const u = rotateVector(solved.rotationMatrix, [1, 0, 0]), v = rotateVector(solved.rotationMatrix, [0, 1, 0]);
    plates.push({ ...ordered(solved.platformPoints, u, v), normal, offset: PLATE_GAP_MM, thickness, color: SCENE_COLORS.platform });
    const reference = loads ? rodForceReference(loads) : 0;
    for (let i = 0; i < 6; i++) {
      const failed = legColor(i);
      cylinders.push({ from: layout.baseAnchors[i], to: solved.hornTips[i], radius: size(layout, BODY_DIMENSIONS.hornRadius),
        color: failed ?? SCENE_COLORS.horn });
      cylinders.push({ from: solved.hornTips[i], to: solved.platformPoints[i], radius: size(layout, BODY_DIMENSIONS.rodRadius),
        color: failed ?? (loads ? rodForceColor(loads.rodForceN[i], reference) : SCENE_COLORS.rod) });
    }
  }
  return { cylinders, boxes, plates };
}

// Column-major 4x4 transforms (as WebGL takes them) that place the unit meshes.
// A cylinder runs from `from` to `to` with the given radius; the unit cylinder
// has radius 1 about +Z from z = 0 to 1.
export function cylinderTransform({ from, to, radius }) {
  const axis = vectorSub(to, from);
  const direction = vectorMagnitude(axis) > 0 ? vectorNormalize(axis) : [0, 0, 1];
  const u = vectorNormalize(vectorCross(direction, Math.abs(direction[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0]));
  const v = vectorCross(direction, u);
  return [...vectorScale(u, radius), 0, ...vectorScale(v, radius), 0, ...axis, 0, ...from, 1];
}

// A box centred at `center` with unit `axes` and full side lengths `size`; the
// unit box spans -0.5 to 0.5 on each axis.
export function boxTransform({ center, axes, size: sides }) {
  return [...vectorScale(axes[0], sides[0]), 0, ...vectorScale(axes[1], sides[1]), 0,
    ...vectorScale(axes[2], sides[2]), 0, ...center, 1];
}

const triangle = (mesh, points, normal) => {
  for (const point of points) mesh.push(...point, ...normal);
};

// Unit meshes as interleaved [x, y, z, nx, ny, nz] triangle vertices with
// outward normals (smooth on the cylinder side, flat elsewhere).
export function unitCylinderMesh(segments = CYLINDER_SEGMENTS) {
  const mesh = [];
  const ring = k => [Math.cos(2 * Math.PI * k / segments), Math.sin(2 * Math.PI * k / segments)];
  for (let k = 0; k < segments; k++) {
    const [a, b] = [ring(k), ring(k + 1)];
    const side = (point, z) => [point[0], point[1], z, point[0], point[1], 0];
    mesh.push(...side(a, 0), ...side(b, 0), ...side(b, 1), ...side(a, 0), ...side(b, 1), ...side(a, 1));
    triangle(mesh, [[0, 0, 0], [b[0], b[1], 0], [a[0], a[1], 0]], [0, 0, -1]);
    triangle(mesh, [[0, 0, 1], [a[0], a[1], 1], [b[0], b[1], 1]], [0, 0, 1]);
  }
  return mesh;
}

export function unitBoxMesh() {
  const mesh = [];
  for (let axis = 0; axis < 3; axis++) {
    for (const sign of [-1, 1]) {
      const normal = [0, 0, 0];
      normal[axis] = sign;
      const [p, q] = [(axis + 1) % 3, (axis + 2) % 3];
      const corner = (s, t) => { const point = [0, 0, 0]; point[axis] = sign / 2; point[p] = s / 2; point[q] = t / 2 * sign; return point; };
      const quad = [corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)];
      triangle(mesh, [quad[0], quad[1], quad[2]], normal);
      triangle(mesh, [quad[0], quad[2], quad[3]], normal);
    }
  }
  return mesh;
}

// A plate as world-space triangles: the outline pushed `offset` along
// `normal`, extruded a further `thickness`, with both faces fanned from the
// centroid and one quad per side, each with its outward normal.
export function plateMesh({ outline, centroid, normal, offset, thickness }) {
  const mesh = [];
  const near = point => vectorAdd(point, vectorScale(normal, offset));
  const far = point => vectorAdd(point, vectorScale(normal, offset + thickness));
  const flipped = vectorScale(normal, -1);
  for (let k = 0; k < outline.length; k++) {
    const a = outline[k], b = outline[(k + 1) % outline.length];
    triangle(mesh, [far(centroid), far(a), far(b)], normal);
    triangle(mesh, [near(centroid), near(b), near(a)], flipped);
    const middle = vectorScale(vectorAdd(a, b), 0.5);
    let outward = vectorNormalize(vectorCross(vectorSub(b, a), normal));
    if (vectorDot(outward, vectorSub(middle, centroid)) < 0) outward = vectorScale(outward, -1);
    triangle(mesh, [near(a), near(b), far(b)], outward);
    triangle(mesh, [near(a), far(b), far(a)], outward);
  }
  return mesh;
}
