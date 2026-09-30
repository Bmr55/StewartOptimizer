import { evaluatePose } from '../model/pose.js';
import { resultFeasibility } from './results.js';
import { DEFAULT_BALL_JOINT_LIMIT_DEG } from '../contracts.js';

const HOME = { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 };
const finitePoint = point => Array.isArray(point) && point.length === 3 && point.every(Number.isFinite);
const midpoint = points => [0, 1, 2].map(axis => points.reduce((sum, point) => sum + point[axis], 0) / points.length);
const subtract = (a, b) => a.map((value, i) => value - b[i]);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = point => Math.hypot(...point);

function planeGuide(base, horn, platform) {
  const hornVector = subtract(horn, base);
  const rodVector = subtract(platform, horn);
  if (norm(cross(hornVector, rodVector)) > 1e-7) return null;
  const axis = Math.abs(hornVector[2]) < norm(hornVector) * 0.9 ? [0, 0, 1] : [0, 1, 0];
  const perpendicular = cross(hornVector, axis);
  const size = norm(perpendicular);
  if (size < 1e-9) throw new Error('Cannot construct a leg sketch plane from coincident home points.');
  return base.map((value, i) => value + 10 * perpendicular[i] / size);
}

function validHome(evaluation) {
  const result = evaluation.homePose || evaluatePose(evaluation.layout, HOME, {
    ballJointLimitDeg: evaluation.workspace?.constraintPolicy?.ballJointLimitDeg ?? DEFAULT_BALL_JOINT_LIMIT_DEG,
    recordLegData: true,
  });
  const layout = evaluation.layout;
  if (!(result.geometricallyReachable ?? result.reachable)
      || !Array.isArray(result.hornTips) || result.hornTips.length !== 6
      || !Array.isArray(result.platformPoints) || result.platformPoints.length !== 6
      || !layout.baseAnchors?.every(finitePoint) || !result.hornTips.every(finitePoint)
      || !result.platformPoints.every(finitePoint)) {
    throw new Error('CAD export requires six valid solved legs at the home pose. Export diagnostic JSON for this candidate instead.');
  }
  return result;
}

export function canExportCad(evaluation) {
  try { validHome(evaluation); return true; }
  catch { return false; }
}

export function buildConstructionSkeleton(evaluation, run = {}) {
  const home = validHome(evaluation);
  const layout = evaluation.layout;
  const points = [];
  const legs = [];
  const addPoint = (name, kind, frame, xyz) => points.push({ name, kind, frame, xyz: xyz.slice() });

  for (let index = 0; index < 6; index++) {
    const leg = index + 1;
    const base = layout.baseAnchors[index];
    const horn = home.hornTips[index];
    const platform = home.platformPoints[index];
    addPoint(`Base anchor ${leg}`, 'base_anchor', 'base', base);
    addPoint(`Horn tip ${leg}`, 'horn_tip', 'world', horn);
    addPoint(`Platform anchor ${leg}`, 'platform_anchor', 'world', platform);
    const guide = planeGuide(base, horn, platform);
    if (guide) addPoint(`Plane guide ${leg}`, 'plane_guide', 'world', guide);
    legs.push({ index: leg, base: `Base anchor ${leg}`, horn: `Horn tip ${leg}`,
      platform: `Platform anchor ${leg}`, planeThird: guide ? `Plane guide ${leg}` : `Platform anchor ${leg}` });
  }
  addPoint('Base centroid', 'base_centroid', 'base', midpoint(layout.baseAnchors));
  addPoint('Platform centroid', 'platform_centroid', 'world', midpoint(home.platformPoints));

  return {
    candidateId: layout.id ?? null,
    run,
    diagnostic: !resultFeasibility(evaluation).passing,
    failedCategories: resultFeasibility(evaluation).failedCategories,
    units: 'mm',
    points,
    legs,
  };
}

const csvCell = value => `"${String(value ?? '').replaceAll('"', '""')}"`;

export function skeletonToCSV(skeleton) {
  const columns = ['candidate_id', 'name', 'kind', 'frame', 'x_mm', 'y_mm', 'z_mm'];
  return [columns.join(','), ...skeleton.points.map(point => [skeleton.candidateId, point.name,
    point.kind, point.frame, ...point.xyz].map(csvCell).join(','))].join('\r\n') + '\r\n';
}

export function skeletonToFusionScript(skeleton) {
  const data = JSON.stringify(skeleton);
  return `# Stewart Optimizer construction skeleton. All source coordinates are millimeters.\n`
    + `# Fusion design API uses centimeters; point_cm converts each coordinate.\n`
    + `# Candidate ${String(skeleton.candidateId ?? 'unknown')}; diagnostic=${skeleton.diagnostic}. No solids are created.\n`
    + `import adsk.core, adsk.fusion, json, traceback\n\n`
    + `DATA = json.loads(${JSON.stringify(data)})\n\n`
    + `def point_cm(values_mm):\n`
    + `    return adsk.core.Point3D.create(*(value / 10.0 for value in values_mm))\n\n`
    + `def run(context):\n`
    + `    app = adsk.core.Application.get()\n`
    + `    ui = app.userInterface\n`
    + `    try:\n`
    + `        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)\n`
    + `        design = adsk.fusion.Design.cast(app.activeProduct)\n`
    + `        design.designType = adsk.fusion.DesignTypes.DirectDesignType\n`
    + `        root = design.rootComponent\n`
    + `        coordinates = {point['name']: point['xyz'] for point in DATA['points']}\n`
    + `        points = {}\n`
    + `        for entry in DATA['points']:\n`
    + `            point_input = root.constructionPoints.createInput()\n`
    + `            if not point_input.setByPoint(point_cm(entry['xyz'])):\n`
    + `                raise RuntimeError('Cannot define construction point ' + entry['name'])\n`
    + `            point = root.constructionPoints.add(point_input)\n`
    + `            if not point:\n`
    + `                raise RuntimeError('Cannot create construction point ' + entry['name'])\n`
    + `            point.name = entry['name']\n`
    + `            points[entry['name']] = point\n`
    + `        for leg in DATA['legs']:\n`
    + `            plane_input = root.constructionPlanes.createInput()\n`
    + `            if not plane_input.setByThreePoints(points[leg['base']], points[leg['horn']], points[leg['planeThird']]):\n`
    + `                raise RuntimeError('Cannot define plane for leg ' + str(leg['index']))\n`
    + `            plane = root.constructionPlanes.add(plane_input)\n`
    + `            if not plane:\n`
    + `                raise RuntimeError('Cannot create plane for leg ' + str(leg['index']))\n`
    + `            plane.name = 'Leg ' + str(leg['index']) + ' plane'\n`
    + `            for name, first, second in [('Horn', leg['base'], leg['horn']), ('Rod', leg['horn'], leg['platform'])]:\n`
    + `                sketch = root.sketches.add(plane)\n`
    + `                sketch.name = 'Leg ' + str(leg['index']) + ' ' + name\n`
    + `                start = sketch.modelToSketchSpace(point_cm(coordinates[first]))\n`
    + `                end = sketch.modelToSketchSpace(point_cm(coordinates[second]))\n`
    + `                line = sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)\n`
    + `                if not line:\n`
    + `                    raise RuntimeError('Cannot create ' + name + ' line for leg ' + str(leg['index']))\n`
    + `                line.isConstruction = True\n`
    + `        ui.messageBox('Imported ' + str(len(DATA['points'])) + ' construction points and 12 horn/rod sketch lines. Source units: mm.')\n`
    + `    except:\n`
    + `        ui.messageBox('Stewart construction import failed:\\n' + traceback.format_exc())\n`
    + `        raise\n`;
}
