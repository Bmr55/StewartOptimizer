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
