import { clamp, vectorAdd, vectorDot, vectorMagnitude, vectorScale, vectorSub } from '../math.js';

// Link interference between legs. Each leg is two straight links: the horn from
// its base anchor (servo shaft) to the horn tip, and the rod from the horn tip
// to its platform joint. Every link of one leg is checked against every link of
// each other leg; the two links of a leg share their joint and are not checked
// against each other. The clearance is the minimum distance allowed between
// link centre lines, so it stands for the sum of the two links' half-widths.
// Servo bodies, joint housings, the plates and the payload are not modelled.
export const LINK_TYPES = Object.freeze(['horn', 'rod']);

export function validateLinkClearance(value) {
  if (typeof value !== 'number' || !Number.isFinite(value) || value < 0) {
    throw new RangeError('linkClearanceMm must be a finite nonnegative length in mm.');
  }
  return value;
}

// Closest distance between segments p1-q1 and p2-q2 (Ericson, Real-Time
// Collision Detection, 5.1.9), including degenerate zero-length segments.
export function segmentDistance(p1, q1, p2, q2) {
  const d1 = vectorSub(q1, p1);
  const d2 = vectorSub(q2, p2);
  const r = vectorSub(p1, p2);
  const a = vectorDot(d1, d1);
  const e = vectorDot(d2, d2);
  const f = vectorDot(d2, r);
  const tiny = 1e-12;
  let s, t;
  if (a <= tiny && e <= tiny) return vectorMagnitude(r);
  if (a <= tiny) {
    s = 0;
    t = clamp(f / e, 0, 1);
  } else {
    const c = vectorDot(d1, r);
    if (e <= tiny) {
      t = 0;
      s = clamp(-c / a, 0, 1);
    } else {
      const b = vectorDot(d1, d2);
      const denominator = a * e - b * b;
      s = denominator > tiny ? clamp((b * f - c * e) / denominator, 0, 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = clamp(-c / a, 0, 1);
      } else if (t > 1) {
        t = 1;
        s = clamp((b - c) / a, 0, 1);
      }
    }
  }
  const closest1 = vectorAdd(p1, vectorScale(d1, s));
  const closest2 = vectorAdd(p2, vectorScale(d2, t));
  return vectorMagnitude(vectorSub(closest1, closest2));
}

// Checks one solved pose. Returns the closest pair of links across all legs and
// one violation per pair of links closer than the clearance.
export function assessLinkClearance(baseAnchors, hornTips, platformPoints, clearanceMm) {
  const link = (leg, type) => type === 'horn' ? [baseAnchors[leg], hornTips[leg]] : [hornTips[leg], platformPoints[leg]];
  let closest = null;
  const violations = [];
  for (let leg = 0; leg < 6; leg++) {
    for (let otherLeg = leg + 1; otherLeg < 6; otherLeg++) {
      for (const type of LINK_TYPES) {
        for (const otherType of LINK_TYPES) {
          const distance = segmentDistance(...link(leg, type), ...link(otherLeg, otherType));
          if (!closest || distance < closest.distanceMm) {
            closest = { distanceMm: distance, legs: [leg, otherLeg], links: [type, otherType] };
          }
          if (distance < clearanceMm) {
            violations.push({ type: 'linkCollision', leg, otherLeg, link: type, otherLink: otherType,
              value: distance, limit: clearanceMm });
          }
        }
      }
    }
  }
  return { clearanceMm, closest, violations };
}

// The legs a violation names: its own leg and, for a collision, the other leg.
export function violationLegs(violation) {
  return [violation.leg, violation.otherLeg].filter(Number.isInteger);
}
