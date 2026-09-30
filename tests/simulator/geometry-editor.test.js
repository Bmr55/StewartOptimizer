import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { evaluatePose } from '../../src/model/pose.js';
import { topologyGeometry, validateTopology } from '../../src/optimization/topology.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { createGeometryEditor, editGeometry, geometryMode,
  PARAMETER_FIELDS } from '../../src/simulator/geometry-editor.js';
import { suggestedParameters } from '../../src/simulator/geometry-controls.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';

const make = topology => new Optimizer({}, { topology, seed: 37 }).createRandomLayout();

test('every parametric geometry control regenerates the intended shared topology coordinates', () => {
  for (const topology of ['circular', 'c3_paired', 'rectangular_paired']) {
    const original = make(topology);
    for (const field of PARAMETER_FIELDS[topology]) {
      const before = structuredClone(original);
      const changed = editGeometry(original, { type: 'parameter', field,
        value: before.topologyParameters[field] + (field.includes('orientation') || field === 'beta_offset' ? 0.05 : 0.5) });
      assert.equal(validateTopology(changed), topology);
      assert.deepEqual(changed.topologyParameters[field], before.topologyParameters[field]
        + (field.includes('orientation') || field === 'beta_offset' ? 0.05 : 0.5));
      const expected = topologyGeometry(topology, changed.topologyParameters);
      assert.deepEqual(changed.baseAnchors, expected.baseAnchors);
      assert.deepEqual(changed.platformAnchors, expected.platformAnchors);
      assert.deepEqual(changed.betaAngles, expected.betaAngles);
      if (field === 'platform_orientation' || field === 'platform_radius'
          || field === 'platform_pair_gap' || field === 'platform_aspect') {
        assert.deepEqual(changed.baseAnchors, before.baseAnchors);
        assert.deepEqual(changed.betaAngles, before.betaAngles);
      }
      if (field === 'beta_offset') {
        assert.deepEqual(changed.baseAnchors, before.baseAnchors);
        assert.deepEqual(changed.platformAnchors, before.platformAnchors);
        assert.notDeepEqual(changed.betaAngles, before.betaAngles);
      }
      assert.deepEqual(original, before);
    }
  }
});

test('length, height, servo bounds, and explicit anchors change only the requested fields', () => {
  const original = asymmetricJointFixture();
  const source = structuredClone(original);
  assert.equal(geometryMode(original), 'explicit');
  const anchor = editGeometry(original, { type: 'anchor', plate: 'platform', leg: 2, axis: 1, value: 42.5 });
  assert.equal(anchor.platformAnchors[2][1], 42.5);
  assert.deepEqual(anchor.baseAnchors, source.baseAnchors);
  assert.deepEqual(anchor.betaAngles, source.betaAngles);
  const beta = editGeometry(anchor, { type: 'betaAngle', leg: 3, degrees: 45 });
  assert.equal(beta.betaAngles[3], Math.PI / 4);
  assert.deepEqual(beta.platformAnchors, anchor.platformAnchors);
  for (const field of ['hornLength', 'rodLength', 'homeHeight']) {
    const changed = editGeometry(beta, { type: 'scalar', field, value: beta[field] + 1 });
    assert.deepEqual(changed.baseAnchors, beta.baseAnchors);
    assert.deepEqual(changed.platformAnchors, beta.platformAnchors);
    assert.deepEqual(changed.betaAngles, beta.betaAngles);
    assert.equal(changed[field], beta[field] + 1);
  }
  const servo = editGeometry(beta, { type: 'servoRange', degrees: [-135, 115] });
  assert.deepEqual(servo.servoRangeRad, [-135, 115].map(value => value * Math.PI / 180));
  assert.deepEqual(servo.baseAnchors, beta.baseAnchors);
  assert.throws(() => editGeometry(beta, { type: 'parameter', field: 'base_radius', value: 120 }), /parametric/);
  assert.throws(() => editGeometry(beta, { type: 'servoRange', degrees: [10, -10] }), /servoRange/);
  assert.deepEqual(original, source);
});

test('mode switching is explicit and controller edits copy candidates, reset, and reevaluate', () => {
  const candidate = make('c3_paired');
  const retained = structuredClone(candidate);
  const controller = createSimulatorController();
  controller.loadLayout(candidate, { source: { kind: 'candidate', candidateId: 19 },
    options: { ballJointLimitDeg: 180 } });
  const editor = createGeometryEditor(controller);
  const initialAssessment = controller.getState().assessment;
  let state = editor.edit({ type: 'scalar', field: 'homeHeight', value: candidate.homeHeight + 10 });
  assert.equal(state.source.kind, 'editable');
  assert.equal(state.source.candidateId, 19);
  assert.equal(state.layout.homeHeight, candidate.homeHeight + 10);
  assert.deepEqual(state.assessment,
    evaluatePose(state.layout, {}, { ballJointLimitDeg: 180, recordLegData: true }));
  assert.notDeepEqual(state.assessment, initialAssessment);
  assert.deepEqual(candidate, retained);

  const exactBeforeSwitch = structuredClone(state.layout);
  state = editor.edit({ type: 'explicitMode' });
  assert.equal(state.layout.topology, 'free');
  assert.deepEqual(state.layout.baseAnchors, exactBeforeSwitch.baseAnchors);
  assert.deepEqual(state.layout.platformAnchors, exactBeforeSwitch.platformAnchors);
  assert.deepEqual(state.layout.betaAngles, exactBeforeSwitch.betaAngles);
  const explicit = editor.edit({ type: 'anchor', plate: 'base', leg: 0, axis: 0,
    value: state.layout.baseAnchors[0][0] + 2 });
  assert.equal(explicit.layout.baseAnchors[0][0], exactBeforeSwitch.baseAnchors[0][0] + 2);
  state = editor.edit({ type: 'generateParametric', topology: 'rectangular_paired',
    parameters: suggestedParameters(explicit.layout, 'rectangular_paired') });
  assert.equal(validateTopology(state.layout), 'rectangular_paired');
  assert.notDeepEqual(state.layout.baseAnchors, explicit.layout.baseAnchors);

  state = editor.reset();
  assert.deepEqual(state.layout, retained);
  assert.deepEqual(state.source, { kind: 'candidate', candidateId: 19 });
  const next = make('circular');
  controller.loadLayout(next, { source: { kind: 'candidate', candidateId: 20 } });
  editor.edit({ type: 'scalar', field: 'rodLength', value: next.rodLength + 1 });
  controller.setOptions({ conditionLimit: 400 });
  state = editor.reset();
  assert.deepEqual(state.layout, next);
  assert.equal(state.source.candidateId, 20);
  assert.equal(state.options.conditionLimit, 400);
  controller.loadLayout(next, { source: { kind: 'candidate', candidateId: 21 } });
  editor.edit({ type: 'scalar', field: 'rodLength', value: next.rodLength + 2 });
  state = editor.reset();
  assert.equal(state.source.candidateId, 21);
  editor.dispose();
});
