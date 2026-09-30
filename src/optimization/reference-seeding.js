import { cloneLayout } from './layout-operators.js';
import { radToDeg } from '../math.js';

export function seedComposition(populationSize) {
  if (!Number.isSafeInteger(populationSize) || populationSize < 4) {
    throw new RangeError('Population must be an integer >= 4.');
  }
  const fresh = Math.round((populationSize - 1) / 4);
  return { reference: 1, variations: populationSize - 1 - fresh, fresh };
}

function outside(value, [min, max]) { return value < min || value > max; }

// These are search bounds, not physical validity. The reference is retained
// unchanged for diagnosis while every generated candidate uses the bounds.
export function referenceBoundsConflicts(layout, designSpace, servoRangeRad) {
  const conflicts = [];
  const check = (field, value, bounds) => {
    if (outside(value, bounds)) conflicts.push({ field, value, bounds: bounds.slice() });
  };
  check('horn_length', layout.hornLength, designSpace.hornLengthBounds);
  check('rod_length', layout.rodLength, designSpace.rodLengthBounds);
  check('home_height', layout.homeHeight, designSpace.homeHeightBounds);
  if (layout.servoRangeRad[0] < servoRangeRad[0] || layout.servoRangeRad[1] > servoRangeRad[1]) {
    conflicts.push({ field: 'servo_range', value: layout.servoRangeDeg ?? layout.servoRangeRad.map(radToDeg),
      bounds: servoRangeRad.map(radToDeg), unit: 'deg' });
  }
  if (layout.topology === 'free') {
    for (const [field, anchors, bounds] of [
      ['base_anchors', layout.baseAnchors, designSpace.baseRadius],
      ['platform_anchors', layout.platformAnchors, designSpace.platformRadius],
    ]) {
      anchors.forEach((point, i) => check(`${field}[${i}].radius`, Math.hypot(point[0], point[1]), bounds));
    }
    layout.baseAnchors.forEach((point, i) => check(`base_anchors[${i}][2]`, point[2],
      [-designSpace.baseZJitter, designSpace.baseZJitter]));
  } else {
    const p = layout.topologyParameters;
    check('topology_parameters.base_radius', p.base_radius, designSpace.baseRadius);
    check('topology_parameters.platform_radius', p.platform_radius, designSpace.platformRadius);
    if (layout.topology === 'c3_paired') {
      check('topology_parameters.base_pair_gap', p.base_pair_gap, designSpace.pairGapBounds);
      check('topology_parameters.platform_pair_gap', p.platform_pair_gap, designSpace.pairGapBounds);
    }
    if (layout.topology === 'rectangular_paired') {
      check('topology_parameters.base_aspect', p.base_aspect, designSpace.rectangularAspectBounds);
      check('topology_parameters.platform_aspect', p.platform_aspect, designSpace.rectangularAspectBounds);
    }
  }
  return conflicts;
}

export function initialPopulation(optimizer) {
  if (!optimizer.referenceLayout) {
    return Array.from({ length: optimizer.populationSize }, () => optimizer.createRandomLayout());
  }
  const composition = seedComposition(optimizer.populationSize);
  const reference = cloneLayout(optimizer.referenceLayout);
  reference.id = optimizer.nextLayoutId++;
  reference.seedOrigin = 'reference';
  const population = [reference];
  for (let i = 0; i < composition.variations; i++) {
    const variation = optimizer.mutateLayout(reference);
    variation.id = optimizer.nextLayoutId++;
    variation.seedOrigin = 'variation';
    delete variation.servoRangeDeg;
    delete variation.referenceDiagnostics;
    delete variation.migration;
    population.push(variation);
  }
  for (let i = 0; i < composition.fresh; i++) {
    const fresh = optimizer.createRandomLayout();
    fresh.seedOrigin = 'fresh';
    population.push(fresh);
  }
  return population;
}
