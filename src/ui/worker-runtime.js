import { Optimizer } from '../optimization/optimizer.js';
import { checkpointSnapshot, optionsFromEffectiveSettings, progressSnapshot } from './worker-protocol.js';

// Injectable ports and clock make the actual module-worker protocol testable in Node.
export function createWorkerRuntime({ postMessage, now = () => performance.now(), OptimizerClass = Optimizer }) {
  let active = null;

  async function handleMessage(message) {
    if (message?.type === 'cancel') {
      // A cancel without a runId while idle must not match the missing active run.
      if (active && active.runId === message.runId) active.optimizer.stop();
      return;
    }
    if (message?.type !== 'start') return;
    const { runId, settings } = message;
    if (active) {
      // A repeated start for the live run is a duplicate, not a failure of that run.
      if (active.runId === runId) return;
      postMessage({ type: 'error', runId, phase: 'overlap', message: 'An optimization is already running.' });
      return;
    }
    if (!settings || typeof settings !== 'object' || Array.isArray(settings)
      || !settings.requirements || typeof settings.requirements !== 'object') {
      postMessage({ type: 'error', runId, phase: 'startup', name: 'TypeError',
        message: 'start settings must be an object with a requirements object.' });
      return;
    }

    let optimizer;
    let startedAt;
    let lastProgressAt = -Infinity;
    try {
      optimizer = new OptimizerClass(settings.requirements, {
        ...optionsFromEffectiveSettings(settings),
        onProgress(progress) {
          const time = now();
          if (time - lastProgressAt < 100) return;
          lastProgressAt = time;
          postMessage({ type: 'progress', runId,
            snapshot: progressSnapshot(optimizer, progress, time - startedAt) });
        },
        onCheckpoint() {
          postMessage({ type: 'checkpoint', runId, snapshot: checkpointSnapshot(optimizer) });
        },
      });
      optimizer.estimateWork();
      startedAt = now();
      active = { runId, optimizer };
      postMessage({ type: 'started', runId, budgetedPoseWork: optimizer.estimateWork().totalPoses });
    } catch (error) {
      postMessage({ type: 'error', runId, phase: 'startup', name: error.name, message: error.message });
      return;
    }

    try {
      const outcome = await optimizer.run();
      postMessage({ type: 'result', runId, outcome, snapshot: checkpointSnapshot(optimizer),
        summary: progressSnapshot(optimizer, {
          completed: optimizer.completedPoseWork, generation: optimizer.generation,
        }, now() - startedAt) });
    } catch (error) {
      postMessage({ type: 'error', runId, phase: 'runtime', name: error.name, message: error.message,
        snapshot: checkpointSnapshot(optimizer),
        summary: progressSnapshot(optimizer, {
          completed: optimizer.completedPoseWork, generation: optimizer.generation,
        }, now() - startedAt) });
    } finally {
      if (active?.runId === runId) active = null;
    }
  }

  return { handleMessage, get activeRunId() { return active?.runId ?? null; } };
}
