import { Optimizer } from '../optimization/optimizer.js';
import { selectBest } from '../io/results.js';
import { ensureLayout } from '../model/pose.js';

let nextRunId = 1;
const WORKER_URL = new URL('./optimizer-worker.js', import.meta.url);

export class WorkerStartupError extends Error {
  constructor(message) { super(message); this.name = 'WorkerStartupError'; this.startupFailure = true; }
}

function errorText(error, fallback) {
  if (typeof error === 'string' && error) return error;
  return error?.message || fallback;
}

// A checkpoint candidate must carry a layout the selection and export paths
// can use; anything else is a corrupt reply rather than partial data.
function validateCandidate(candidate, index) {
  if (!candidate || typeof candidate !== 'object') throw new TypeError(`Checkpoint candidate ${index} is not an object.`);
  try { ensureLayout(candidate.layout); }
  catch (error) { throw new TypeError(`Checkpoint candidate ${index}: ${error.message}`); }
  const { id, servoRangeRad } = candidate.layout;
  if (id === undefined || id === null) throw new TypeError(`Checkpoint candidate ${index} layout has no id.`);
  if (!Array.isArray(servoRangeRad) || servoRangeRad.length !== 2 || !servoRangeRad.every(Number.isFinite)) {
    throw new TypeError(`Checkpoint candidate ${index} layout must provide two finite servo bounds.`);
  }
}

// Browser adapter: numerical work remains owned by Optimizer inside the worker.
// The inherited selection and export API operates on completed checkpoints only.
export class WorkerOptimizer extends Optimizer {
  constructor(requirements, options = {}, {
    workerFactory = (url, settings) => new Worker(url, settings),
    startupTimeoutMs = 5000,
  } = {}) {
    super(requirements, options);
    this.workerFactory = workerFactory;
    this.startupTimeoutMs = startupTimeoutMs;
    this.mode = null;
    this.startupFailure = false;
    this.lastProgress = null;
  }

  // Incomplete populations are ignored; an invalid candidate throws before any
  // state changes, so the last valid checkpoint survives a corrupt reply.
  applyCheckpoint(snapshot) {
    if (!snapshot || !Array.isArray(snapshot.fitness)
      || snapshot.fitness.length !== this.populationSize) return;
    snapshot.fitness.forEach(validateCandidate);
    this.fitness = snapshot.fitness;
    this.population = snapshot.fitness.map(candidate => candidate.layout);
    const ids = new Set(snapshot.paretoIds || []);
    this.pareto = snapshot.fitness.filter(candidate => ids.has(candidate.layout.id));
    this.generation = snapshot.generation;
    this.completedEvaluations = snapshot.completedCandidates;
    this.selectedCandidateId = selectBest(this.pareto, this.fitness)?.layout.id ?? null;
  }

  start(callback) {
    if (this.running) return Promise.reject(new Error('An optimization is already running.'));
    this.estimateWork();
    this.mode = 'worker';
    this.running = true;
    this.runStatus = 'running';
    this.startupFailure = false;
    this.fitness = [];
    this.pareto = [];
    this.population = [];
    this.selectedCandidateId = null;
    this.generation = 0;
    this.completedEvaluations = 0;
    this.lastProgress = null;
    const runId = `run-${nextRunId++}`;
    this.runId = runId;

    return new Promise((resolve, reject) => {
      let worker;
      let started = false;
      let settled = false;
      let timer;
      const cleanup = () => {
        clearTimeout(timer);
        if (worker) {
          worker.onmessage = null;
          worker.onerror = null;
          worker.terminate();
        }
        if (this.worker === worker) this.worker = null;
        this.running = false;
      };
      const failStartup = error => {
        if (settled) return;
        settled = true;
        this.runStatus = 'failed';
        this.startupFailure = true;
        cleanup();
        reject(new WorkerStartupError(errorText(error, 'Worker failed to start.')));
      };
      const failRuntime = (message, snapshot, summary) => {
        if (settled) return;
        settled = true;
        // A corrupt final checkpoint must not hide the failure; keep the last valid one.
        try { this.applyCheckpoint(snapshot); } catch { /* keep the previous checkpoint */ }
        if (summary) this.lastProgress = summary;
        this.runStatus = 'failed';
        const outcome = { status: 'failed', completedGenerations: this.generation,
          completedEvaluations: this.completedEvaluations, partialResults: this.fitness.length > 0,
          error: message };
        cleanup();
        try { callback?.(this, outcome); resolve(outcome); }
        catch (error) { reject(error); }
      };
      const handle = message => {
        switch (message.type) {
          case 'started':
            started = true;
            clearTimeout(timer);
            break;
          case 'progress':
            if (!started) break;
            this.lastProgress = message.snapshot;
            this.onProgress?.({ ...message.snapshot,
              completed: message.snapshot.actualCompletedPoseWork,
              total: message.snapshot.budgetedPoseWork });
            break;
          case 'checkpoint':
            if (started) this.applyCheckpoint(message.snapshot);
            break;
          case 'result': {
            if (!started) break;
            const outcome = message.outcome;
            if (!outcome || typeof outcome !== 'object' || typeof outcome.status !== 'string') {
              throw new TypeError('Worker result is missing its outcome.');
            }
            this.applyCheckpoint(message.snapshot);
            if (message.summary) this.lastProgress = message.summary;
            settled = true;
            this.runStatus = outcome.status;
            cleanup();
            try { callback?.(this, outcome); resolve(outcome); }
            catch (error) { reject(error); }
            break;
          }
          case 'error':
            if (!started || message.phase === 'startup') failStartup(new Error(message.message));
            else failRuntime(message.message, message.snapshot, message.summary);
            break;
        }
      };
      try {
        worker = this.workerFactory(WORKER_URL, { type: 'module' });
        if (!worker || typeof worker.postMessage !== 'function') {
          throw new TypeError('The worker factory did not return a worker.');
        }
        this.worker = worker;
        worker.onmessage = event => {
          const message = event?.data;
          if (!message || message.runId !== this.runId || settled) return;
          // A malformed reply is a failure of this run, never an exception that
          // escapes onmessage and leaves the run pending forever.
          try { handle(message); }
          catch (error) {
            if (!started) failStartup(error);
            else failRuntime(errorText(error, 'Worker reply could not be processed.'));
          }
        };
        worker.onerror = error => {
          if (!started) failStartup(error);
          else failRuntime(errorText(error, 'Worker execution failed.'));
        };
        timer = setTimeout(() => failStartup(new Error('Worker startup timed out.')), this.startupTimeoutMs);
        worker.postMessage({ type: 'start', runId, settings: this.effectiveSettings() });
      } catch (error) {
        failStartup(error);
      }
    });
  }

  startFallback(callback) {
    if (!this.startupFailure || this.running) {
      return Promise.reject(new Error('Main-thread fallback is available only after worker startup fails.'));
    }
    this.mode = 'fallback';
    this.startupFailure = false;
    return super.start(callback);
  }

  stop() {
    if (this.mode === 'worker' && this.running) {
      this.worker?.postMessage({ type: 'cancel', runId: this.runId });
    } else super.stop();
  }
}
