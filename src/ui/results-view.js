import { METRICS } from '../contracts.js';
import { failureCategories, isPassing, rankCandidates } from '../io/results.js';

const AXES = ['torque', 'speedDemand', 'coverage', 'conditioningQuality', 'dexterity',
  'stiffness', 'loadBalance', 'isotropy', 'limitMargin', 'fatigue'];
const label = key => `${key.replace(/([A-Z])/g, ' $1')} (${METRICS[key].unit})`;
const printable = value => Number.isFinite(value) ? Number(value).toPrecision(4) : 'unavailable';
const escapeHtml = value => String(value).replace(/[&<>"']/g, char => ({
  '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
})[char]);

function axisDomain(values) {
  const finite = values.filter(Number.isFinite);
  if (!finite.length) return [0, 1];
  const min = Math.min(...finite);
  const max = Math.max(...finite);
  if (min === max) return [min - 1, max + 1];
  return [min, max];
}

export function chartSvg(candidates, selectedId, xKey = 'torque', yKey = 'speedDemand') {
  const width = 640;
  const height = 300;
  const margin = 40;
  const [xMin, xMax] = axisDomain(candidates.map(candidate => candidate[xKey]));
  const [yMin, yMax] = axisDomain(candidates.map(candidate => candidate[yKey]));
  const x = value => margin + (value - xMin) / (xMax - xMin) * (width - 2 * margin);
  const y = value => height - margin - (value - yMin) / (yMax - yMin) * (height - 2 * margin);
  const coincident = new Map();
  const points = candidates.map(candidate => {
    const xv = candidate[xKey];
    const yv = candidate[yKey];
    if (!Number.isFinite(xv) || !Number.isFinite(yv)) return '';
    const key = `${xv}:${yv}`;
    const offset = coincident.get(key) || 0;
    coincident.set(key, offset + 1);
    const spread = offset ? 4 * Math.ceil(offset / 2) * (offset % 2 ? 1 : -1) : 0;
    const id = escapeHtml(candidate.layout.id);
    const title = `Candidate ${id}: ${escapeHtml(label(xKey))} ${printable(xv)}, ${escapeHtml(label(yKey))} ${printable(yv)}; ${isPassing(candidate) ? 'passing' : 'diagnostic'}`;
    return `<circle data-candidate-id="${id}" cx="${x(xv) + spread}" cy="${y(yv) - spread}" r="${String(candidate.layout.id) === String(selectedId) ? 8 : 6}" fill="${isPassing(candidate) ? '#4dabf7' : '#ffad5c'}" stroke="${String(candidate.layout.id) === String(selectedId) ? '#fff' : 'none'}" tabindex="0" role="button" aria-label="${title}"><title>${title}</title></circle>`;
  }).join('');
  return `<svg viewBox="0 0 ${width} ${height}" role="img" aria-label="Candidate chart, ${escapeHtml(label(xKey))} versus ${escapeHtml(label(yKey))}"><line x1="${margin}" y1="${height - margin}" x2="${width - margin}" y2="${height - margin}" stroke="#aaa"/><line x1="${margin}" y1="${margin}" x2="${margin}" y2="${height - margin}" stroke="#aaa"/><text x="${width / 2}" y="${height - 8}" fill="#ddd" text-anchor="middle">${escapeHtml(label(xKey))}</text><text x="12" y="${height / 2}" fill="#ddd" transform="rotate(-90 12 ${height / 2})" text-anchor="middle">${escapeHtml(label(yKey))}</text>${points}</svg>`;
}

export function createResultsView(document, onSelect) {
  const candidateSelect = document.getElementById('candidateSelect');
  const xAxis = document.getElementById('chartXAxis');
  const yAxis = document.getElementById('chartYAxis');
  const chart = document.getElementById('paretoChart');
  const summary = document.getElementById('candidateSummary');
  let candidates = [];
  let selectedId = null;

  for (const [element, initial] of [[xAxis, 'torque'], [yAxis, 'speedDemand']]) {
    element.innerHTML = AXES.map(key => `<option value="${key}">${escapeHtml(label(key))}</option>`).join('');
    element.value = initial;
  }

  function redraw() {
    chart.innerHTML = chartSvg(candidates, selectedId, xAxis.value, yAxis.value);
    const selected = candidates.find(candidate => String(candidate.layout.id) === String(selectedId));
    if (!selected) {
      summary.textContent = 'No completed candidate is available.';
      return;
    }
    const flags = selected.feasibility || {};
    const failures = failureCategories(selected);
    const reference = selected.layout.seedOrigin === 'reference' ? selected.referenceDiagnostics : null;
    const details = reference
      ? ` Exact reference. Bounds conflicts: ${reference.boundsConflicts.map(item => item.field).join(', ') || 'none'}. Home violations: ${reference.homePoseViolations.map(item => `${item.type} leg ${item.leg + 1}`).join(', ') || 'none'}.`
      : '';
    const capacity = selected.servoCapacity;
    const capacityText = capacity?.hasRatings
      ? ` Servo capacity ${capacity.status} (${capacity.policy}${capacity.policy === 'advisory' && ['above', 'unavailable'].includes(capacity.status) ? ' warning' : ''}); worst headroom ${printable(capacity.worstHeadroomFraction)} of rating.`
      : '';
    summary.textContent = `Candidate ${selectedId}: ${isPassing(selected) ? 'passing' : `diagnostic (${failures.join(', ') || 'requirements failed'})`}. Feasible sampled coverage ${printable(selected.coverage)}%. Home ${flags.homePoseSatisfied ? 'pass' : 'fail'}; workspace ${flags.sampledWorkspaceSatisfied ? 'pass' : 'fail'}; cycle ${flags.cycleSatisfied ? 'pass' : 'fail'}. Torque ${printable(selected.torque)} N m; speed ${printable(selected.speedDemand)} rad/s.${capacityText}${details}`;
  }

  function choose(id) {
    const candidate = candidates.find(item => String(item.layout.id) === String(id));
    if (!candidate) return;
    selectedId = candidate.layout.id;
    candidateSelect.value = String(selectedId);
    redraw();
    onSelect(candidate);
  }

  candidateSelect.addEventListener('change', () => choose(candidateSelect.value));
  xAxis.addEventListener('change', redraw);
  yAxis.addEventListener('change', redraw);
  chart.addEventListener('click', event => choose(event.target?.dataset?.candidateId));
  chart.addEventListener('keydown', event => {
    if (event.key === 'Enter' || event.key === ' ') choose(event.target?.dataset?.candidateId);
  });

  return {
    render(retained, initialId) {
      candidates = rankCandidates(retained);
      selectedId = initialId ?? candidates[0]?.layout.id ?? null;
      candidateSelect.innerHTML = candidates.map(candidate => {
        const id = escapeHtml(candidate.layout.id);
        const state = isPassing(candidate) ? 'passing' : `diagnostic: ${failureCategories(candidate).join(', ')}`;
        const prefix = candidate.layout.seedOrigin === 'reference' ? 'Exact reference' : `Candidate ${id}`;
        return `<option value="${id}">${escapeHtml(prefix)} — ${escapeHtml(state)}</option>`;
      }).join('');
      candidateSelect.disabled = !candidates.length;
      candidateSelect.value = selectedId === null ? '' : String(selectedId);
      redraw();
    },
    clear() { this.render([], null); },
  };
}
