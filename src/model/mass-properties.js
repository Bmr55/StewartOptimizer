import { jacobiEigenvaluesSymmetric } from '../math.js';

export const GRAVITY = Object.freeze([0, 0, -9.81]);
export const RIGID_BODY_FIELDS = Object.freeze(['center_of_mass_mm', 'inertia_kg_m2',
  'external_force_n', 'external_moment_nm']);

const vector3 = (value, name) => {
  if (!Array.isArray(value) || value.length !== 3 || !value.every(Number.isFinite)) {
    throw new RangeError(`${name} must be three finite numbers.`);
  }
  return value.slice();
};

// Accepts a 3x3 tensor or {ixx, iyy, izz, ixy, ixz, iyz}. Off-diagonal values are
// tensor entries (I_xy = -integral of x y dm), not positive products of inertia.
function inertiaTensor(value) {
  let tensor;
  if (Array.isArray(value)) {
    if (value.length !== 3 || value.some(row => !Array.isArray(row) || row.length !== 3)) {
      throw new RangeError('inertia_kg_m2 must be a 3x3 matrix or an ixx/iyy/izz/ixy/ixz/iyz object.');
    }
    tensor = value.map(row => row.slice());
  } else if (value && typeof value === 'object') {
    // Omitted products default to zero; null is rejected by the finite check like every other parser path.
    const get = key => value[key] === undefined ? 0 : value[key];
    for (const key of ['ixx', 'iyy', 'izz']) {
      if (!(key in value)) throw new RangeError(`inertia_kg_m2.${key} is required.`);
    }
    tensor = [[get('ixx'), get('ixy'), get('ixz')], [get('ixy'), get('iyy'), get('iyz')],
      [get('ixz'), get('iyz'), get('izz')]];
  } else {
    throw new RangeError('inertia_kg_m2 must be a 3x3 matrix or an ixx/iyy/izz/ixy/ixz/iyz object.');
  }
  if (!tensor.flat().every(Number.isFinite)) throw new RangeError('inertia_kg_m2 entries must be finite.');
  const scale = Math.max(1e-12, ...tensor.flat().map(Math.abs));
  for (let i = 0; i < 3; i++) {
    for (let j = i + 1; j < 3; j++) {
      if (Math.abs(tensor[i][j] - tensor[j][i]) > 1e-9 * scale) {
        throw new RangeError('inertia_kg_m2 must be symmetric.');
      }
    }
  }
  const principal = jacobiEigenvaluesSymmetric(tensor, 1e-14 * scale, 100);
  const tolerance = 1e-9 * scale;
  if (principal.some(value => value < -tolerance)) {
    throw new RangeError('inertia_kg_m2 must be positive semidefinite.');
  }
  // Principal moments of a real mass distribution satisfy the triangle inequality.
  const [large, middle, small] = principal;
  if (middle + small < large - tolerance) {
    throw new RangeError('inertia_kg_m2 principal moments violate the triangle inequality.');
  }
  return { tensor, principal };
}

// Combined moving mass (platform plus payload). The center of mass is in the
// platform frame relative to the moving origin (mm); inertia is about that center
// of mass in platform axes. External force acts at the center of mass; both
// external force and moment are base-frame vectors applied to the moving body.
export function normalizeMassProperties(requirements = {}) {
  const mass = requirements.mass_kg ?? 0;
  if (!Number.isFinite(mass) || mass < 0) throw new RangeError('mass_kg must be a finite number >= 0.');
  const supplied = RIGID_BODY_FIELDS.filter(key => requirements[key] != null);
  const centerMm = requirements.center_of_mass_mm == null ? [0, 0, 0]
    : vector3(requirements.center_of_mass_mm, 'center_of_mass_mm');
  const inertia = requirements.inertia_kg_m2 == null ? { tensor: [[0, 0, 0], [0, 0, 0], [0, 0, 0]], principal: [0, 0, 0] }
    : inertiaTensor(requirements.inertia_kg_m2);
  if (mass === 0 && inertia.principal.some(value => value > 0)) {
    throw new RangeError('inertia_kg_m2 requires a positive mass_kg.');
  }
  const externalForce = requirements.external_force_n == null ? [0, 0, 0]
    : vector3(requirements.external_force_n, 'external_force_n');
  const externalMoment = requirements.external_moment_nm == null ? [0, 0, 0]
    : vector3(requirements.external_moment_nm, 'external_moment_nm');
  return {
    mode: supplied.length ? 'rigid-body' : 'legacy-point',
    massKg: mass,
    centerOfMassM: centerMm.map(value => value / 1000),
    inertiaKgM2: inertia.tensor,
    principalInertiaKgM2: inertia.principal,
    externalForceN: externalForce,
    externalMomentNm: externalMoment,
    supplied,
  };
}

export function massPropertiesDescription(massProperties) {
  return massProperties.mode === 'legacy-point'
    ? 'legacy centered point mass at the moving origin'
    : 'rigid body: supplied center of mass, inertia about center of mass, and external wrench';
}
