import { Optimizer } from '../optimization/optimizer.js';
import { selectBest } from '../io/results.js';

let nextRunId = 1;
const WORKER_URL = new URL('./optimizer-worker.js', import.meta.url);

export class WorkerStartupError extends Error {
  constructor(message) { super(message); this.name = 'WorkerStartupError'; this.startupFailure = true; }
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

  applyCheckpoint(snapshot) {
    if (!snapshot || !Array.isArray(snapshot.fitness)
      || snapshot.fitness.length !== this.populationSize) return;
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
        reject(new WorkerStartupError(error?.message || String(error)));
      };
      const failRuntime = (message, snapshot) => {
        if (settled) return;
        settled = true;
        this.applyCheckpoint(snapshot);
        this.runStatus = 'failed';
        const outcome = { status: 'failed', completedGenerations: this.generation,
          completedEvaluations: this.completedEvaluations, partialResults: this.fitness.length > 0,
          error: message };
        cleanup();
        try { callback?.(this, outcome); resolve(outcome); }
        catch (error) { reject(error); }
      };
      try {
        worker = this.workerFactory(WORKER_URL, { type: 'module' });
        this.worker = worker;
        worker.onmessage = event => {
          const message = event.data;
          if (!message || message.runId !== this.runId || settled) return;
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
              this.applyCheckpoint(message.snapshot);
              settled = true;
              this.runStatus = message.outcome.status;
              const outcome = message.outcome;
              cleanup();
              try { callback?.(this, outcome); resolve(outcome); }
              catch (error) { reject(error); }
              break;
            }
            case 'error':
              if (!started || message.phase === 'startup') failStartup(new Error(message.message));
              else failRuntime(message.message, message.snapshot);
              break;
          }
        };
        worker.onerror = error => {
          if (!started) failStartup(error);
          else failRuntime(error?.message || 'Worker execution failed.');
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
