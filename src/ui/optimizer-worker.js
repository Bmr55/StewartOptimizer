import { createWorkerRuntime } from './worker-runtime.js';

const runtime = createWorkerRuntime({ postMessage: message => self.postMessage(message) });
self.addEventListener('message', event => { void runtime.handleMessage(event.data); });
