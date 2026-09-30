import { radToDeg } from '../math.js';
import { DEFAULT_BALL_JOINT_LIMIT_DEG } from '../contracts.js';
import { NUMERICAL_RECIPROCAL_CUTOFF } from '../model/conditioning.js';

const numeric = value => Number.isFinite(value) ? value : null;
const rounded = (value, digits = 3) => value == null ? '—' : Number(value.toFixed(digits)).toString();
const escapeHtml = value => String(value).replace(/[&<>"']/g, character => ({
  '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
})[character]);
const angleDeg = value => numeric(value) == null ? null : radToDeg(value);

export function buildPoseDiagnostics(state) {
  const options = state?.options ?? {};
  const lowerLimitDeg = options.lowerBallJointLimitDeg ?? options.ballJointLimitDeg ?? DEFAULT_BALL_JOINT_LIMIT_DEG;
  const upperLimitDeg = options.upperBallJointLimitDeg ?? options.ballJointLimitDeg ?? DEFAULT_BALL_JOINT_LIMIT_DEG;
  const rodLengthToleranceMm = options.rodLengthTolerance ?? 0.5;
  const conditionLimit = options.conditionLimit ?? null;
  const assessment = state?.assessment ?? null;
  const layout = state?.layout ?? null;
  const violations = assessment?.violations ?? [];
  const legs = Array.from({ length: 6 }, (_, index) => {
    const lowerDeg = angleDeg(assessment?.jointAngles?.lower?.[index]);
    const upperDeg = angleDeg(assessment?.jointAngles?.upper?.[index]);
    const rodLengthMm = numeric(assessment?.rodLengths?.[index]);
    const servoAngleDeg = angleDeg(assessment?.servoAngles?.[index]);
    const rodDeviationMm = rodLengthMm == null || !layout ? null : rodLengthMm - layout.rodLength;
    const failures = violations.filter(violation => violation.leg === index).map(violation => ({
      type: violation.type, joint: violation.joint ?? null,
      value: numeric(violation.value), limit: numeric(violation.limit),
      reason: violation.reason ?? null,
    }));
    return { leg: index + 1, lowerDeg, upperDeg,
      maximumDeflectionDeg: lowerDeg == null || upperDeg == null ? null : Math.max(lowerDeg, upperDeg),
      lowerLimitDeg, upperLimitDeg, rodLengthMm, targetRodLengthMm: layout?.rodLength ?? null,
      rodDeviationMm, rodLengthToleranceMm,
      rodWithinTolerance: rodDeviationMm == null ? null : Math.abs(rodDeviationMm) <= rodLengthToleranceMm,
      servoAngleDeg, failures,
      lowerFailed: failures.some(failure => failure.joint === 'lower'),
      upperFailed: failures.some(failure => failure.joint === 'upper'),
    };
  });
  return {
    requested: state?.requested ?? null,
    accepted: state?.accepted ?? null,
    requestedStatus: !layout ? 'no-layout' : state.rejected ? 'rejected' : 'accepted',
    acceptedStatus: state?.accepted ? 'retained' : 'none',
    source: state?.requestSource ?? null,
    lowerLimitDeg, upperLimitDeg, rodLengthToleranceMm, conditionLimit,
    // Pose conditioning applies the shared cutoff but does not echo it back.
    numericalThreshold: NUMERICAL_RECIPROCAL_CUTOFF,
    legs,
    globalFailures: violations.filter(violation => !Number.isInteger(violation.leg))
      .map(violation => ({ type: violation.type, reason: violation.reason ?? null })),
    affectedLegs: legs.filter(leg => leg.failures.length).map(leg => leg.leg),
  };
}

function poseText(pose) {
  if (!pose) return 'None';
  return `X ${rounded(pose.x)} Y ${rounded(pose.y)} Z ${rounded(pose.z)} mm; `
    + `Rx ${rounded(angleDeg(pose.rx))} Ry ${rounded(angleDeg(pose.ry))} Rz ${rounded(angleDeg(pose.rz))}°`;
}

export function mountSimulatorDiagnostics({ document, controller, host = document.getElementById('simDiagnostics') }) {
  host.innerHTML = `<h2>Pose diagnostics</h2>
    <p id="simRequestedDiagnostic" class="status" aria-live="polite"></p>
    <p id="simAcceptedDiagnostic" class="status"></p>
    <div class="grid sim-diagnostic-settings">
      <label class="field">Lower joint limit (deg)<input id="simLowerJointLimit" type="number" min="0" max="180" step="any"></label>
      <label class="field">Upper joint limit (deg)<input id="simUpperJointLimit" type="number" min="0" max="180" step="any"></label>
      <label class="field">Rod-length tolerance (mm)<input id="simRodTolerance" type="number" min="0" step="any"></label>
    </div>
    <p id="simConditionPolicy" class="status"></p>
    <p id="simDiagnosticError" class="status error" role="alert" hidden></p>
    <p id="simGlobalFailures" class="status" aria-live="polite"></p>
    <div class="sim-diagnostic-table-wrap"><table class="sim-diagnostic-table"><thead><tr>
      <th>Leg</th><th>Lower</th><th>Upper</th><th>Max</th><th>Rod deviation</th><th>Servo</th><th>Requested-pose failure</th>
    </tr></thead><tbody id="simDiagnosticRows"></tbody></table></div>`;
  const field = id => document.getElementById(id);
  const error = field('simDiagnosticError');

  // Redraws arrive on every animation frame, so a focused limit field keeps the
  // user's text unless a rejected edit forces the stored value back.
  function redraw(state, force = false) {
    const diagnostics = buildPoseDiagnostics(state);
    const active = document.activeElement;
    const setValue = (id, value) => {
      const input = field(id);
      if (force || input !== active) input.value = String(value);
    };
    const requested = field('simRequestedDiagnostic');
    requested.textContent = `Requested pose (${diagnostics.source ?? 'manual'}): ${poseText(diagnostics.requested)} — ${diagnostics.requestedStatus}.`;
    requested.classList.toggle('error', diagnostics.requestedStatus === 'rejected');
    field('simAcceptedDiagnostic').textContent = `Rendered accepted pose: ${poseText(diagnostics.accepted)}.`;
    setValue('simLowerJointLimit', diagnostics.lowerLimitDeg);
    setValue('simUpperJointLimit', diagnostics.upperLimitDeg);
    setValue('simRodTolerance', diagnostics.rodLengthToleranceMm);
    field('simConditionPolicy').textContent = `Condition limit: ${diagnostics.conditionLimit ?? 'none'}; mandatory reciprocal cutoff: ${diagnostics.numericalThreshold}. Joint limits ${diagnostics.lowerLimitDeg}° lower / ${diagnostics.upperLimitDeg}° upper; rod tolerance ±${diagnostics.rodLengthToleranceMm} mm.`;
    field('simGlobalFailures').textContent = diagnostics.globalFailures.length
      ? `Whole-platform failure: ${diagnostics.globalFailures.map(failure => failure.type).join(', ')}.` : 'No whole-platform failure.';
    field('simDiagnosticRows').innerHTML = diagnostics.legs.map(leg => {
      const labels = leg.failures.map(failure => `${failure.type}${failure.joint ? ` (${failure.joint})` : ''}`);
      return `<tr data-leg="${leg.leg}" class="${leg.failures.length ? 'sim-leg-failure' : ''}">
        <th scope="row">${leg.leg}</th>
        <td class="${leg.lowerFailed ? 'sim-joint-failure' : ''}">${rounded(leg.lowerDeg)}° / ${rounded(leg.lowerLimitDeg)}°</td>
        <td class="${leg.upperFailed ? 'sim-joint-failure' : ''}">${rounded(leg.upperDeg)}° / ${rounded(leg.upperLimitDeg)}°</td>
        <td>${rounded(leg.maximumDeflectionDeg)}°</td>
        <td class="${leg.rodWithinTolerance === false ? 'sim-joint-failure' : ''}">${rounded(leg.rodDeviationMm, 6)} / ±${rounded(leg.rodLengthToleranceMm)} mm</td>
        <td>${rounded(leg.servoAngleDeg)}°</td>
        <td>${escapeHtml(labels.join(', ') || '—')}</td>
      </tr>`;
    }).join('');
  }

  const unsubscribe = controller.subscribe(state => redraw(state));
  for (const [id, key, maximum] of [
    ['simLowerJointLimit', 'lowerBallJointLimitDeg', 180],
    ['simUpperJointLimit', 'upperBallJointLimitDeg', 180],
    ['simRodTolerance', 'rodLengthTolerance', Infinity],
  ]) {
    field(id).addEventListener('change', () => {
      try {
        const input = field(id).value.trim();
        const value = input ? Number(input) : NaN;
        if (!Number.isFinite(value) || value < 0 || value > maximum) {
          throw new RangeError(`${key} must be a finite number from 0 to ${maximum === Infinity ? 'infinity' : maximum}.`);
        }
        error.hidden = true;
        controller.setOptions({ [key]: value });
      } catch (problem) {
        error.textContent = problem.message;
        error.hidden = false;
        redraw(controller.getState(), true);
      }
    });
  }
  return { dispose: unsubscribe, redraw: () => redraw(controller.getState()) };
}
