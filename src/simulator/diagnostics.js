import { radToDeg } from '../math.js';
import { DEFAULT_BALL_JOINT_LIMIT_DEG, DEFAULT_LINK_CLEARANCE_MM } from '../contracts.js';
import { violationLegs } from '../model/collision.js';
import { NUMERICAL_RECIPROCAL_CUTOFF } from '../model/conditioning.js';
import { markCommitted, syncInput } from './geometry-controls.js';

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
  const linkClearanceMm = options.linkClearanceMm ?? DEFAULT_LINK_CLEARANCE_MM;
  const assessment = state?.assessment ?? null;
  const layout = state?.layout ?? null;
  const violations = assessment?.violations ?? [];
  // Loads belong to the accepted pose, not the request.
  const loads = state?.loads ?? null;
  const solvedLoads = loads?.valid ? loads : null;
  const legs = Array.from({ length: 6 }, (_, index) => {
    const lowerDeg = angleDeg(assessment?.jointAngles?.lower?.[index]);
    const upperDeg = angleDeg(assessment?.jointAngles?.upper?.[index]);
    const rodLengthMm = numeric(assessment?.rodLengths?.[index]);
    const servoAngleDeg = angleDeg(assessment?.servoAngles?.[index]);
    const rodDeviationMm = rodLengthMm == null || !layout ? null : rodLengthMm - layout.rodLength;
    // A collision is listed on both legs it names, each against the other.
    const failures = violations.filter(violation => violationLegs(violation).includes(index)).map(violation => ({
      type: violation.type, joint: violation.joint ?? null,
      collision: violation.type === 'linkCollision' ? (violation.leg === index
        ? { link: violation.link, otherLeg: violation.otherLeg + 1, otherLink: violation.otherLink }
        : { link: violation.otherLink, otherLeg: violation.leg + 1, otherLink: violation.link }) : null,
      value: numeric(violation.value), limit: numeric(violation.limit),
      reason: violation.reason ?? null,
    }));
    return { leg: index + 1, lowerDeg, upperDeg,
      maximumDeflectionDeg: lowerDeg == null || upperDeg == null ? null : Math.max(lowerDeg, upperDeg),
      lowerLimitDeg, upperLimitDeg, rodLengthMm, targetRodLengthMm: layout?.rodLength ?? null,
      rodDeviationMm, rodLengthToleranceMm,
      rodWithinTolerance: rodDeviationMm == null ? null : Math.abs(rodDeviationMm) <= rodLengthToleranceMm,
      servoAngleDeg, failures,
      rodForceN: numeric(solvedLoads?.rodForceN?.[index]),
      servoTorqueNm: numeric(solvedLoads?.servoTorqueNm?.[index]),
      ratedTorqueNm: numeric(solvedLoads?.ratedTorqueNm?.[index]),
      torqueUtilization: numeric(solvedLoads?.utilization?.[index]),
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
    lowerLimitDeg, upperLimitDeg, rodLengthToleranceMm, conditionLimit, linkClearanceMm,
    // Closest links of different legs in the requested pose, when every leg solved.
    closestLinks: assessment?.clearance?.closest ?? null,
    // Pose conditioning applies the shared cutoff but does not echo it back.
    numericalThreshold: NUMERICAL_RECIPROCAL_CUTOFF,
    legs,
    globalFailures: violations.filter(violation => !Number.isInteger(violation.leg))
      .map(violation => ({ type: violation.type, reason: violation.reason ?? null })),
    affectedLegs: legs.filter(leg => leg.failures.length).map(leg => leg.leg),
    loadStatus: !state?.loadModel ? 'none' : !loads ? 'noPose' : loads.valid ? loads.motion : 'unavailable',
    loadReason: loads?.reason ?? null,
  };
}

const LOAD_STATUS_TEXT = Object.freeze({
  none: 'Loads: no payload or servo ratings loaded. A candidate from a run, or simulator JSON with a loadModel, supplies them.',
  noPose: 'Loads: no accepted pose to solve.',
  static: 'Loads at the accepted pose: static (zero velocity and acceleration).',
  animation: 'Loads at the accepted pose: dynamic, from the animation velocity and acceleration.',
  unavailable: 'Loads unavailable at the accepted pose:',
});

// Signed output-shaft torque, with the peak rating and utilisation when rated.
function torqueText(leg) {
  if (leg.servoTorqueNm == null) return '—';
  const torque = `${rounded(leg.servoTorqueNm)} N m`;
  return leg.ratedTorqueNm == null ? `${torque} / unrated`
    : `${torque} / ${rounded(leg.ratedTorqueNm)} N m (${Math.round(leg.torqueUtilization * 100)}%)`;
}

function closestText(closest) {
  if (!closest) return '';
  const [leg, otherLeg] = closest.legs;
  const [link, otherLink] = closest.links;
  return ` (closest: leg ${leg + 1} ${link} to leg ${otherLeg + 1} ${otherLink}, ${rounded(closest.distanceMm, 1)} mm)`;
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
      <label class="field">Link clearance (mm)<input id="simLinkClearance" type="number" min="0" step="any"></label>
    </div>
    <p id="simConditionPolicy" class="status"></p>
    <p id="simDiagnosticError" class="status error" role="alert" hidden></p>
    <p id="simGlobalFailures" class="status" aria-live="polite"></p>
    <p id="simLoadDiagnostic" class="status"></p>
    <div class="sim-diagnostic-table-wrap"><table class="sim-diagnostic-table"><thead><tr>
      <th>Leg</th><th>Lower</th><th>Upper</th><th>Max</th><th>Rod deviation</th><th>Servo</th><th>Rod force (+ compression)</th><th>Servo torque / rating</th><th>Requested-pose failure</th>
    </tr></thead><tbody id="simDiagnosticRows"></tbody></table></div>`;
  const field = id => document.getElementById(id);
  const error = field('simDiagnosticError');
  const synced = new WeakMap();

  // Redraws arrive on every animation frame, so a limit field the user is typing
  // into keeps its text unless a rejected edit forces the stored value back; a
  // focused but untouched field still follows a load or settings change.
  function redraw(state, force = false) {
    const diagnostics = buildPoseDiagnostics(state);
    const setValue = (id, value) => syncInput(document, field(id), String(value), synced, force);
    const requested = field('simRequestedDiagnostic');
    requested.textContent = `Requested pose (${diagnostics.source ?? 'manual'}): ${poseText(diagnostics.requested)} — ${diagnostics.requestedStatus}.`;
    requested.classList.toggle('error', diagnostics.requestedStatus === 'rejected');
    field('simAcceptedDiagnostic').textContent = `Rendered accepted pose: ${poseText(diagnostics.accepted)}.`;
    setValue('simLowerJointLimit', diagnostics.lowerLimitDeg);
    setValue('simUpperJointLimit', diagnostics.upperLimitDeg);
    setValue('simRodTolerance', diagnostics.rodLengthToleranceMm);
    setValue('simLinkClearance', diagnostics.linkClearanceMm);
    field('simConditionPolicy').textContent = `Condition limit: ${diagnostics.conditionLimit ?? 'none'}; mandatory reciprocal cutoff: ${diagnostics.numericalThreshold}. Joint limits ${diagnostics.lowerLimitDeg}° lower / ${diagnostics.upperLimitDeg}° upper; rod tolerance ±${diagnostics.rodLengthToleranceMm} mm; link clearance ${diagnostics.linkClearanceMm} mm${closestText(diagnostics.closestLinks)}.`;
    const globalFailures = diagnostics.globalFailures.length
      ? `Whole-platform failure: ${diagnostics.globalFailures.map(failure => failure.type).join(', ')}.` : 'No whole-platform failure.';
    // aria-live: rewrite only on change so animation frames do not re-announce it.
    if (field('simGlobalFailures').textContent !== globalFailures) field('simGlobalFailures').textContent = globalFailures;
    field('simLoadDiagnostic').textContent = LOAD_STATUS_TEXT[diagnostics.loadStatus]
      + (diagnostics.loadReason ? ` ${diagnostics.loadReason}` : '');
    field('simDiagnosticRows').innerHTML = diagnostics.legs.map(leg => {
      const labels = leg.failures.map(failure => failure.collision
        ? `${failure.type} (${failure.collision.link} with leg ${failure.collision.otherLeg} ${failure.collision.otherLink})`
        : `${failure.type}${failure.joint ? ` (${failure.joint})` : ''}`);
      return `<tr data-leg="${leg.leg}" class="${leg.failures.length ? 'sim-leg-failure' : ''}">
        <th scope="row">${leg.leg}</th>
        <td class="${leg.lowerFailed ? 'sim-joint-failure' : ''}">${rounded(leg.lowerDeg)}° / ${rounded(leg.lowerLimitDeg)}°</td>
        <td class="${leg.upperFailed ? 'sim-joint-failure' : ''}">${rounded(leg.upperDeg)}° / ${rounded(leg.upperLimitDeg)}°</td>
        <td>${rounded(leg.maximumDeflectionDeg)}°</td>
        <td class="${leg.rodWithinTolerance === false ? 'sim-joint-failure' : ''}">${rounded(leg.rodDeviationMm, 6)} / ±${rounded(leg.rodLengthToleranceMm)} mm</td>
        <td>${rounded(leg.servoAngleDeg)}°</td>
        <td>${leg.rodForceN == null ? '—' : `${rounded(leg.rodForceN, 2)} N`}</td>
        <td class="${leg.torqueUtilization > 1 ? 'sim-joint-failure' : ''}">${torqueText(leg)}</td>
        <td>${escapeHtml(labels.join(', ') || '—')}</td>
      </tr>`;
    }).join('');
  }

  const unsubscribe = controller.subscribe(state => redraw(state));
  for (const [id, key, maximum] of [
    ['simLowerJointLimit', 'lowerBallJointLimitDeg', 180],
    ['simUpperJointLimit', 'upperBallJointLimitDeg', 180],
    ['simRodTolerance', 'rodLengthTolerance', Infinity],
    ['simLinkClearance', 'linkClearanceMm', Infinity],
  ]) {
    field(id).addEventListener('change', () => {
      markCommitted(field(id), synced);
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
