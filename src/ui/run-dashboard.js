const MIN_UPDATE_MS = 100;
const count = value => Number.isFinite(value) ? Math.max(0, Math.floor(value)).toLocaleString() : '—';
const metric = (value, digits = 3) => Number.isFinite(value) ? Number(value).toPrecision(digits) : 'unavailable';

export function formatDuration(milliseconds) {
  if (!Number.isFinite(milliseconds) || milliseconds < 0) return 'Unavailable';
  const seconds = milliseconds / 1000;
  if (seconds < 59.95) return `${seconds.toFixed(1)} s`;
  // Round whole seconds before splitting so the remainder never reads 60.
  const wholeSeconds = Math.round(seconds);
  const minutes = Math.floor(wholeSeconds / 60);
  return `${minutes}m ${wholeSeconds - minutes * 60}s`;
}

export function bestCandidateText(best) {
  if (!best) return 'No completed population yet';
  const result = best.passing ? 'passing' : `diagnostic${best.failedCategories?.length ? ` (${best.failedCategories.join(', ')})` : ''}`;
  const coverage = Number.isFinite(best.coverage) ? `${metric(best.coverage)}%` : 'unavailable';
  return `#${best.id} · ${result} · coverage ${coverage} · torque ${metric(best.torque)} N m · speed ${metric(best.speedDemand)} rad/s · conditioning ${metric(best.conditioningQuality)}`;
}

export function createRunDashboard(document, { now = () => performance.now() } = {}) {
  const fields = Object.fromEntries([
    'runPhase', 'runElapsed', 'runCandidates', 'runGeneration', 'runFrontSize',
    'runBestCandidate', 'runEta', 'runPoseWork',
  ].map(id => [id, document.getElementById(id)]));
  let activeRunId = null;
  let startedAt = null;
  let lastPublishedAt = -Infinity;
  let totalCandidates = 0;
  let totalGenerations = 0;
  let budgetedPoseWork = 0;

  function render(snapshot) {
    fields.runElapsed.textContent = formatDuration(snapshot.elapsedMs);
    fields.runCandidates.textContent = `${count(snapshot.completedCandidates)} / ${count(snapshot.totalCandidates ?? totalCandidates)}`;
    fields.runGeneration.textContent = `${count(snapshot.generation)} / ${count(totalGenerations)}`;
    fields.runFrontSize.textContent = count(snapshot.frontSize);
    fields.runBestCandidate.textContent = bestCandidateText(snapshot.bestCandidate);
    fields.runEta.textContent = snapshot.etaApproximate && Number.isFinite(snapshot.etaMs)
      ? `≈ ${formatDuration(snapshot.etaMs)} (approximate)`
      : snapshot.completedCandidates >= 3 ? 'Unavailable (timing data)' : 'Unavailable until 3 completed candidates';
    fields.runPoseWork.textContent = `${count(snapshot.actualCompletedPoseWork)} actual / ${count(snapshot.budgetedPoseWork ?? budgetedPoseWork)} budgeted`;
  }

  function reset() {
    activeRunId = null;
    startedAt = null;
    lastPublishedAt = -Infinity;
    fields.runPhase.textContent = 'Idle';
    render({ elapsedMs: 0, completedCandidates: 0, totalCandidates: 0,
      generation: 0, frontSize: 0, bestCandidate: null, etaMs: null,
      actualCompletedPoseWork: 0, budgetedPoseWork: 0 });
  }

  function start(runId, { candidates, generations, poseWork }) {
    activeRunId = runId;
    startedAt = now();
    lastPublishedAt = -Infinity;
    totalCandidates = candidates;
    totalGenerations = generations;
    budgetedPoseWork = poseWork;
    fields.runPhase.textContent = 'Running';
    render({ elapsedMs: 0, completedCandidates: 0, totalCandidates: candidates,
      generation: 0, frontSize: 0, bestCandidate: null, etaMs: null,
      actualCompletedPoseWork: 0, budgetedPoseWork: poseWork });
  }

  function publish(runId, snapshot) {
    if (runId !== activeRunId || !snapshot) return false;
    const time = now();
    if (time - lastPublishedAt < MIN_UPDATE_MS) return false;
    lastPublishedAt = time;
    render(snapshot);
    return true;
  }

  function cancelRequested(runId) {
    if (runId !== activeRunId) return false;
    fields.runPhase.textContent = 'Cancelling';
    return true;
  }

  function finish(runId, { status, partialResults = false, snapshot = null }) {
    if (runId !== activeRunId) return false;
    render(snapshot ?? { elapsedMs: now() - startedAt, completedCandidates: 0,
      totalCandidates, generation: 0, frontSize: 0, bestCandidate: null,
      etaMs: null, actualCompletedPoseWork: 0, budgetedPoseWork });
    fields.runPhase.textContent = `${status}${partialResults ? ' · partial results' : ''}`;
    if (status === 'completed') fields.runEta.textContent = 'Complete';
    else fields.runEta.textContent = 'Unavailable';
    activeRunId = null;
    return true;
  }

  reset();
  return { start, publish, cancelRequested, finish, reset,
    elapsedMs: () => startedAt === null ? 0 : now() - startedAt };
}
