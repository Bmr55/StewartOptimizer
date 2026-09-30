import { jacobiEigenvaluesSymmetric } from '../math.js';
import { physicalMotionJacobian, solveLinear } from './cycle.js';

export const COMPLIANCE_MODEL = 'cartesian-compliance-v1';
export const COMPLIANCE_ASSUMPTIONS = 'Unloaded small-deflection stiffness at the home pose: ideal frictionless joints, '
  + 'servo output torsional springs in series with axial rod springs, no preload or geometric (load-dependent) stiffness, '
  + 'no joint, horn, base or platform structural compliance, no backlash and no control-loop stiffness. '
  + 'Not a buckling or loaded-stability analysis.';
const RELATIVE_SINGULAR = 1e-10;

const positive = (value, name) => {
  if (!Number.isFinite(value) || value <= 0) throw new RangeError(`${name} must be a finite positive number.`);
  return value;
};

const vector3 = (value, name) => {
  if (value === undefined) return [0, 0, 0];
  if (!Array.isArray(value) || value.length !== 3 || !value.every(Number.isFinite)) {
    throw new RangeError(`${name} must be three finite numbers.`);
  }
  return value.slice();
};

// Inputs are SI except rod section data (GPa, mm^2 or mm) and the characteristic
// length (mm). Rods are `"rigid"`, a direct axial stiffness, or E A / L from material
// data with L the candidate's rod length.
export function normalizeStiffnessModel(input) {
  if (input == null) return null;
  if (typeof input !== 'object' || Array.isArray(input)) throw new TypeError('stiffness_model must be an object.');
  const name = key => `stiffness_model.${key}`;
  const servo = input.servo_torsional_stiffness_nm_per_rad;
  if (servo == null) throw new RangeError(`${name('servo_torsional_stiffness_nm_per_rad')} is required.`);
  let servoStiffness;
  if (Array.isArray(servo)) {
    if (servo.length !== 6) throw new RangeError(`${name('servo_torsional_stiffness_nm_per_rad')} must be a number or six numbers.`);
    servoStiffness = servo.map((value, i) => positive(value, `${name('servo_torsional_stiffness_nm_per_rad')}[${i}]`));
  } else {
    servoStiffness = new Array(6).fill(positive(servo, name('servo_torsional_stiffness_nm_per_rad')));
  }
  const rodSources = ['rod_axial_stiffness_n_per_m', 'rod_material', 'rods'].filter(key => input[key] !== undefined);
  if (rodSources.length !== 1) {
    throw new RangeError('stiffness_model needs exactly one of rod_axial_stiffness_n_per_m, rod_material, or rods: "rigid".');
  }
  let rod;
  if (input.rods !== undefined) {
    if (input.rods !== 'rigid') throw new RangeError(`${name('rods')} must be "rigid".`);
    rod = { kind: 'rigid' };
  } else if (input.rod_axial_stiffness_n_per_m !== undefined) {
    rod = { kind: 'supplied', stiffnessNPerM: positive(input.rod_axial_stiffness_n_per_m, name('rod_axial_stiffness_n_per_m')) };
  } else {
    const material = input.rod_material;
    if (!material || typeof material !== 'object' || Array.isArray(material)) throw new TypeError(`${name('rod_material')} must be an object.`);
    const modulusPa = positive(material.youngs_modulus_gpa, name('rod_material.youngs_modulus_gpa')) * 1e9;
    if ((material.area_mm2 == null) === (material.diameter_mm == null)) {
      throw new RangeError(`${name('rod_material')} needs exactly one of area_mm2 or diameter_mm.`);
    }
    const areaM2 = material.area_mm2 != null ? positive(material.area_mm2, name('rod_material.area_mm2')) * 1e-6
      : Math.PI * (positive(material.diameter_mm, name('rod_material.diameter_mm')) / 2000) ** 2;
    rod = { kind: 'material', modulusPa, areaM2 };
  }
  const length = input.characteristic_length_mm;
  const characteristicLengthM = length === undefined ? null : positive(length, name('characteristic_length_mm')) / 1000;
  const wrenches = input.test_wrenches === undefined ? [] : input.test_wrenches;
  if (!Array.isArray(wrenches)) throw new TypeError(`${name('test_wrenches')} must be an array.`);
  const testWrenches = wrenches.map((wrench, i) => {
    if (!wrench || typeof wrench !== 'object') throw new TypeError(`${name('test_wrenches')}[${i}] must be an object.`);
    if (wrench.name === null) throw new TypeError(`${name('test_wrenches')}[${i}].name must not be null.`);
    return { name: String(wrench.name ?? `wrench ${i + 1}`),
      forceN: vector3(wrench.force_n, `${name('test_wrenches')}[${i}].force_n`),
      momentNm: vector3(wrench.moment_nm, `${name('test_wrenches')}[${i}].moment_nm`) };
  });
  const useAsObjective = input.use_as_objective === undefined ? false : input.use_as_objective;
  if (typeof useAsObjective !== 'boolean') throw new TypeError(`${name('use_as_objective')} must be a boolean.`);
  return { servoStiffnessNmPerRad: servoStiffness, rod, characteristicLengthM, testWrenches, useAsObjective };
}

const unavailable = (reason, extra = {}) => ({ modelVersion: COMPLIANCE_MODEL, status: 'unavailable', reason,
  stiffnessMatrix: null, complianceMatrix: null, minScaledStiffnessNPerM: null, testWrenches: [], ...extra });

function rmsAnchorRadiusM(layout) {
  const anchors = layout.platformAnchors;
  const centroid = [0, 1, 2].map(axis => anchors.reduce((sum, a) => sum + a[axis], 0) / anchors.length);
  return Math.sqrt(anchors.reduce((sum, a) => sum + a.reduce((s, v, i) => s + (v - centroid[i]) ** 2, 0), 0)
    / anchors.length) / 1000;
}

// K = A^T diag(k_leg) A about the moving origin in base-frame axes, with each leg's
// servo spring (referred through D_ii) in series with its rod spring:
// 1 / k_leg = 1 / k_rod + D_ii^2 / k_servo. With rigid rods, K = G^T K_servo G.
export function evaluateCompliance(layout, poseResult, model) {
  if (!model) return unavailable('No stiffness_model supplied; the physical stiffness is not modeled.');
  if (!poseResult?.reachable) return unavailable('The operating pose is not valid.');
  const motion = physicalMotionJacobian(layout, poseResult);
  if (!motion.rows) return unavailable(motion.reason);
  const rodLengthM = layout.rodLength / 1000;
  const rodStiffness = model.rod.kind === 'rigid' ? Infinity
    : model.rod.kind === 'supplied' ? model.rod.stiffnessNPerM : model.rod.modulusPa * model.rod.areaM2 / rodLengthM;
  const legStiffness = motion.transmissions.map((d, i) => 1 / (1 / rodStiffness + d * d / model.servoStiffnessNmPerRad[i]));
  const servoShare = motion.transmissions.map((d, i) => (d * d / model.servoStiffnessNmPerRad[i]) * legStiffness[i]);
  const K = Array.from({ length: 6 }, (_, r) => Array.from({ length: 6 }, (_, c) =>
    motion.A.reduce((sum, row, i) => sum + row[r] * legStiffness[i] * row[c], 0)));
  const length = model.characteristicLengthM ?? rmsAnchorRadiusM(layout);
  // Rotations scale by 1/L so every entry of S K S is N/m; raw mixed-unit eigenvalues are not compared.
  const S = [1, 1, 1, 1 / length, 1 / length, 1 / length];
  const scaled = K.map((row, r) => row.map((value, c) => value * S[r] * S[c]));
  const eigenvalues = jacobiEigenvaluesSymmetric(scaled, 1e-12 * Math.max(...scaled.flat().map(Math.abs)), 1000);
  const base = {
    modelVersion: COMPLIANCE_MODEL, operatingPose: 'home',
    reference: 'moving-platform origin; base-frame axes; twist [dx, dy, dz (m), rx, ry, rz (rad)]; wrench [F (N), M (N m)]',
    units: { translational: 'N/m', coupling: 'N/rad (upper right), N (lower left)', rotational: 'N m/rad', scaled: 'N/m' },
    assumptions: COMPLIANCE_ASSUMPTIONS,
    rodModel: model.rod.kind, rodStiffnessNPerM: Number.isFinite(rodStiffness) ? rodStiffness : null,
    servoStiffnessNmPerRad: model.servoStiffnessNmPerRad, legStiffnessNPerM: legStiffness,
    servoComplianceFraction: servoShare,
    characteristicLengthM: length,
    characteristicLengthSource: model.characteristicLengthM == null ? 'platform RMS anchor radius' : 'supplied',
    scaledEigenvaluesNPerM: eigenvalues,
    stiffnessMatrix: K,
  };
  const minimum = eigenvalues.at(-1);
  if (!(minimum > RELATIVE_SINGULAR * eigenvalues[0])) {
    return { ...base, status: 'singular', reason: 'Stiffness matrix has a compliant (near-zero) direction.',
      complianceMatrix: null, minScaledStiffnessNPerM: null, testWrenches: [] };
  }
  const columns = Array.from({ length: 6 }, (_, c) => solveLinear(K, Array.from({ length: 6 }, (_, r) => r === c ? 1 : 0)));
  if (columns.some(column => !column)) {
    return { ...base, status: 'singular', reason: 'Stiffness matrix could not be inverted.',
      complianceMatrix: null, minScaledStiffnessNPerM: null, testWrenches: [] };
  }
  const C = Array.from({ length: 6 }, (_, r) => columns.map(column => column[r]));
  const testWrenches = model.testWrenches.map(wrench => {
    const W = [...wrench.forceN, ...wrench.momentNm];
    const twist = C.map(row => row.reduce((sum, value, k) => sum + value * W[k], 0));
    const energy = 0.5 * twist.reduce((sum, value, k) => sum + value * W[k], 0);
    return { name: wrench.name, forceN: wrench.forceN, momentNm: wrench.momentNm,
      translationMm: twist.slice(0, 3).map(value => value * 1000),
      rotationDeg: twist.slice(3).map(value => value * 180 / Math.PI), energyJ: energy };
  });
  return { ...base, status: 'available', complianceMatrix: C, minScaledStiffnessNPerM: minimum, testWrenches };
}
