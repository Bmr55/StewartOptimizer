export function jointFixture() {
  const anchors = radius => Array.from({ length: 6 }, (_, i) => [radius * Math.cos(i * Math.PI / 3), radius * Math.sin(i * Math.PI / 3), 0]);
  return { baseAnchors: anchors(100), platformAnchors: anchors(50),
    betaAngles: Array.from({ length: 6 }, (_, i) => i * Math.PI / 3 + Math.PI / 2),
    hornLength: 50, rodLength: 200, homeHeight: 200, servoRangeRad: [-Math.PI, Math.PI] };
}

export function asymmetricJointFixture() {
  const layout = jointFixture();
  layout.platformAnchors[0][0] += 8;
  layout.platformAnchors[2][1] -= 12;
  layout.baseAnchors[3][0] -= 14;
  layout.betaAngles[1] += 0.2;
  layout.betaAngles[4] -= 0.3;
  return layout;
}

// D3-symmetric paired layout: mirrored leg pairs every 120 degrees, nonsingular at home.
export function pairedFixture() {
  const rad = Math.PI / 180;
  const baseAnchors = [], platformAnchors = [], betaAngles = [];
  for (let k = 0; k < 3; k++) {
    const axis = 120 * k;
    for (const side of [-1, 1]) {
      const base = axis + side * 15, platform = axis + side * 45, beta = axis + 75;
      baseAnchors.push([100 * Math.cos(base * rad), 100 * Math.sin(base * rad), 0]);
      platformAnchors.push([50 * Math.cos(platform * rad), 50 * Math.sin(platform * rad), 0]);
      betaAngles.push((side < 0 ? beta : 2 * axis - beta) * rad);
    }
  }
  return { baseAnchors, platformAnchors, betaAngles, hornLength: 50, rodLength: 200, homeHeight: 200,
    servoRangeRad: [-Math.PI, Math.PI] };
}
