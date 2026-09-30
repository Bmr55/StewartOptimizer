// Compatibility entry point for callers using the original root module.
export { evaluatePose } from './src/model/pose.js';
export { computeWorkspace, estimateWorkspaceSize, MAX_WORKSPACE_POSES, yieldToEventLoop } from './src/workspace/sweep.js';
