import { selectBest, displayResult, layoutToJSON } from '../io/results.js';
import { importLayout } from '../io/layout-import.js';
import { download } from './download.js';
import { parseRequirements } from '../model/requirements.js';
import { loadDefaultRequirements as loadSample } from '../io/sample-requirements.js';
import { Optimizer as DefaultOptimizer } from '../optimization/optimizer.js';
import { WorkerOptimizer } from './worker-optimizer.js';
import { progressSnapshot } from './worker-protocol.js';
import { createRunDashboard } from './run-dashboard.js';
import { createControls } from './controls.js';
import { installTooltips } from './tooltips.js';
import { createResultsView } from './results-view.js';
import { buildConstructionSkeleton, canExportCad, skeletonToCSV, skeletonToFusionScript } from '../io/cad.js';
import { createServoRatingControls } from './servo-ratings-controls.js';
import { createSimulatorController, ANIMATION_PATTERNS } from '../simulator/controller.js';
import { createSimulatorView } from '../simulator/view.js';
import { createGeometryControls } from '../simulator/geometry-controls.js';
import { mountSimulatorDiagnostics } from '../simulator/diagnostics.js';
import { LOCAL_WORKSPACE_KEY, captureLocalWorkspace, parseLocalWorkspace,
    applyLocalWorkspace } from './local-workspace.js';

function simulatorOptions(settings = {}, layout = {}) {
    return {
        ballJointLimitDeg: settings.ballJointLimitDeg ?? 45,
        lowerBallJointLimitDeg: settings.lowerBallJointLimitDeg ?? settings.ballJointLimitDeg ?? 45,
        upperBallJointLimitDeg: settings.upperBallJointLimitDeg ?? settings.ballJointLimitDeg ?? 45,
        conditionLimit: settings.conditionLimit ?? null,
        servoRangeRad: layout.servoRangeRad,
        rodLengthTolerance: settings.rodLengthTolerance ?? 0.5,
    };
}

export function createApp({ document, window, Optimizer = DefaultOptimizer, workerFactory,
    loadDefaultRequirements = loadSample, downloadFile = download,
    now = () => performance.now() }) {
    const requirementsInput = document.getElementById('requirementsInput');
    const referenceLayoutInput = document.getElementById('referenceLayoutInput');
    const statusEl = document.getElementById('optStatus');
    const resultOutput = document.getElementById('resultOutput');
    const ballJointClampCheckbox = document.getElementById('ballJointClamp');
    const ballJointLimitInput = document.getElementById('ballJointLimit');
    let currentOptimizer = null;
    let lastOutcome = null;
    let simulatorRun = null;
    let runSerial = 0;
    let runReferenceNote = '';
    const dashboard = createRunDashboard(document, { now });
    const fallbackButton = document.getElementById('runMainThreadFallback');
    const downloadFormat = document.getElementById('downloadFormat');
    const downloadButton = document.getElementById('downloadSelected');
    function offerFallback(available) {
        fallbackButton.hidden = !available;
        fallbackButton.disabled = !available;
    }
    offerFallback(false);
    let activeTab = 'optimize';
    const simCandidateSelect = document.getElementById('simCandidateSelect');
    const simulatorController = createSimulatorController();
    const simulatorView = createSimulatorView({ document, window, controller: simulatorController,
        isActive: () => activeTab === 'simulate' });
    const geometryContainer = document.getElementById('simGeometryControls');
    const geometryControls = geometryContainer?.appendChild && geometryContainer?.replaceChildren
        ? createGeometryControls({ document, container: geometryContainer, controller: simulatorController })
        : null;
    const simulatorDiagnostics = mountSimulatorDiagnostics({ document, controller: simulatorController });
    function setTab(tab) {
        activeTab = tab;
        for (const [name, buttonId, panelId] of [['optimize', 'optimizeTab', 'optimizePanel'],
            ['simulate', 'simulateTab', 'simulatePanel']]) {
            const selected = name === tab;
            const button = document.getElementById(buttonId);
            button.classList.toggle('active', selected);
            button.setAttribute('aria-selected', String(selected));
            document.getElementById(panelId).hidden = !selected;
        }
        if (tab === 'simulate') simulatorView.render();
    }
    document.getElementById('optimizeTab').addEventListener('click', () => setTab('optimize'));
    document.getElementById('simulateTab').addEventListener('click', () => setTab('simulate'));
    const { populateRequirementsDefaults, readWorkspaceRanges, readHomeHeightBounds,
        readSamplingSettings, randomizeSeed } = createControls(document);
    const ratingControls = createServoRatingControls(document);
    function populate(parsed, preserveEdits = false) {
        populateRequirementsDefaults(parsed, preserveEdits);
        ratingControls.populate(parsed.normalized, preserveEdits);
    }
    installTooltips(document, window);
    simCandidateSelect.addEventListener('change', () => {
        if (simCandidateSelect.value) resultsView.select(simCandidateSelect.value);
    });
    function clearSimulatorSelection() {
        simCandidateSelect.innerHTML = '';
        simCandidateSelect.disabled = true;
    }
    function loadCandidate(candidate) {
        simulatorRun = lastOutcome;
        simCandidateSelect.value = String(candidate.layout.id);
        simulatorController.loadLayout(candidate.layout, {
            source: { kind: 'candidate', candidateId: candidate.layout.id },
            options: simulatorOptions(lastOutcome?.effective_settings ?? currentOptimizer?.effectiveSettings?.(), candidate.layout),
        });
        document.getElementById('simDownload').disabled = false;
    }
    const resultsView = createResultsView(document, (candidate) => {
        if (!currentOptimizer || currentOptimizer.running) return;
        currentOptimizer.selectCandidate(candidate.layout.id);
        resultOutput.value = JSON.stringify({ run: lastOutcome, result: displayResult(candidate) }, null, 2);
        loadCandidate(candidate);
        setRunning(false);
    });
    resultsView.clear();
    clearSimulatorSelection();

    function showStatus(message, isError = false) {
        statusEl.textContent = message;
        statusEl.classList.toggle('error', isError);
    }

    document.getElementById('loadSampleRequirements').addEventListener('click', async () => {
        try {
            const json = await loadDefaultRequirements();
            requirementsInput.value = json;
            populate(parseRequirements(json));
            showStatus('Sample requirements loaded.');
        } catch (error) {
            console.error(error);
            showStatus(error.message, true);
        }
    });

    document.getElementById('clearRequirements').addEventListener('click', () => {
        requirementsInput.value = '';
        resultOutput.value = '';
        currentOptimizer = null;
        lastOutcome = null;
        offerFallback(false);
        simulatorRun = null;
        simulatorController.clear();
        clearSimulatorSelection();
        document.getElementById('simDownload').disabled = true;
        resultsView.clear();
        dashboard.reset();
        setRunning(false);
        showStatus('Requirements cleared.');
    });

    document.getElementById('randomizeSeed').addEventListener('click', () => {
        try {
            showStatus(`Run seed set to ${randomizeSeed(window.crypto)}.`);
        } catch (error) { showStatus(error.message, true); }
    });

    document.getElementById('clearReferenceLayout').addEventListener('click', () => {
        referenceLayoutInput.value = '';
        document.getElementById('referenceLayoutFile').value = '';
        showStatus('Reference layout cleared.');
    });

    document.getElementById('referenceLayoutFile').addEventListener('change', async event => {
        const file = event.target?.files?.[0];
        if (!file) return;
        try {
            referenceLayoutInput.value = await file.text();
            showStatus(`Reference layout loaded: ${file.name}.`);
        } catch (error) {
            showStatus(`Could not read reference layout: ${error.message}`, true);
        }
    });

    function updateDownloadControls(running) {
        const selected = currentOptimizer?.getSelectedCandidate?.();
        const hasResult = Boolean(selected || currentOptimizer?.fitness?.length);
        downloadFormat.disabled = running || !hasResult;
        downloadButton.disabled = running || !hasResult
            || (downloadFormat.value !== 'json' && !canExportCad(selected));
    }

    function setRunning(running) {
        ratingControls.setDisabled(running);
        for (const id of ['runOptimization', 'loadSampleRequirements', 'clearRequirements',
            'optSampling', 'optSeed', 'randomizeSeed', 'optPopulation', 'optGenerations',
            'optObjectiveSet', 'optMutationRate', 'clearReferenceLayout', 'referenceLayoutFile',
            'saveLocalWorkspace', 'restoreLocalWorkspace', 'deleteLocalWorkspace']) {
            document.getElementById(id).disabled = running;
        }
        document.getElementById('cancelOptimization').disabled = !running;
        if (running) fallbackButton.disabled = true;
        updateDownloadControls(running);
    }

    downloadFormat.addEventListener('change', () => updateDownloadControls(Boolean(currentOptimizer?.running)));

    document.getElementById('cancelOptimization').addEventListener('click', () => {
        currentOptimizer?.stop();
        dashboard.cancelRequested(runSerial);
        showStatus('Cancelling optimization...');
    });

    function finalSnapshot() {
        if (currentOptimizer?.lastProgress) return currentOptimizer.lastProgress;
        if (!Number.isInteger(currentOptimizer?.completedEvaluations)) return null;
        return progressSnapshot(currentOptimizer, {
            completed: currentOptimizer.completedPoseWork ?? 0,
            generation: currentOptimizer.generation,
        }, dashboard.elapsedMs());
    }

    function presentOutcome(outcome, thisRun) {
        const pareto = currentOptimizer.pareto?.length ? currentOptimizer.pareto : currentOptimizer.fitness;
        const best = currentOptimizer.getSelectedCandidate?.()
            ?? selectBest(currentOptimizer.pareto, currentOptimizer.fitness);
        lastOutcome = { ...outcome, effective_settings: currentOptimizer.effectiveSettings?.() };
        resultsView.render(currentOptimizer.fitness, best?.layout.id);
        simCandidateSelect.innerHTML = currentOptimizer.fitness.map(item =>
            `<option value="${item.layout.id}">Candidate ${item.layout.id}</option>`).join('');
        simCandidateSelect.disabled = !currentOptimizer.fitness.length;
        resultOutput.value = best ? JSON.stringify({ run: lastOutcome, result: displayResult(best) }, null, 2) : '';
        dashboard.finish(thisRun, { status: outcome.status, partialResults: outcome.partialResults,
            snapshot: finalSnapshot() });
        if (best) loadCandidate(best);
        if (outcome.status === 'cancelled') {
            showStatus(best ? 'Optimization cancelled. Showing partial results from the last completed population.' : 'Optimization cancelled before a population completed.');
        } else if (outcome.status === 'failed') {
            showStatus(`${outcome.error || 'Worker execution failed.'}${best ? ' Showing partial results from the last completed population.' : ''}`, true);
        } else {
            showStatus(`Optimization complete. Feasible coverage: ${best?.coverage ?? 0}%. Pareto front contains ${currentOptimizer.pareto.length || pareto.length} layouts. Coverage applies only to sampled poses and modeled constraints.${runReferenceNote}`);
        }
    }

    function reportProgress(thisRun, progress) {
        if (thisRun !== runSerial) return;
        const snapshot = Number.isFinite(progress.elapsedMs) && Number.isInteger(progress.completedCandidates)
            ? progress : progressSnapshot(currentOptimizer, progress, dashboard.elapsedMs());
        if (!dashboard.publish(thisRun, snapshot)) return;
        const completed = snapshot.actualCompletedPoseWork;
        const total = snapshot.budgetedPoseWork;
        showStatus(`Generation ${snapshot.generation}: ${completed.toLocaleString()} / ${total.toLocaleString()} pose evaluations (${total ? (100 * completed / total).toFixed(1) : '0.0'}%).`);
    }

    fallbackButton.addEventListener('click', async () => {
        if (!(currentOptimizer instanceof WorkerOptimizer) || !currentOptimizer.startupFailure || currentOptimizer.running) return;
        const thisRun = ++runSerial;
        offerFallback(false);
        currentOptimizer.onProgress = progress => reportProgress(thisRun, progress);
        const work = currentOptimizer.estimateWork();
        dashboard.start(thisRun, { candidates: work.evaluations,
            generations: currentOptimizer.generations, poseWork: work.totalPoses });
        showStatus('Running explicit main-thread fallback. The page yields between pose batches.');
        setRunning(true);
        try {
            const outcome = await currentOptimizer.startFallback();
            if (thisRun === runSerial) presentOutcome(outcome, thisRun);
        } catch (error) {
            if (thisRun === runSerial) {
                dashboard.finish(thisRun, { status: 'failed', snapshot: finalSnapshot() });
                showStatus(error.message, true);
            }
        } finally {
            if (thisRun === runSerial) setRunning(false);
        }
    });

    document.getElementById('runOptimization').addEventListener('click', async () => {
        if (currentOptimizer?.running) return;
        const thisRun = ++runSerial;
        offerFallback(false);
        try {
            const text = requirementsInput.value.trim();
            if (!text) {
                throw new Error('Provide requirements JSON before running the optimizer.');
            }
            const { normalized, workspace } = parseRequirements(text);
            populate({ normalized, workspace }, true);

            const generations = Number(document.getElementById('optGenerations').value);
            const populationSize = Number(document.getElementById('optPopulation').value);
            const objectiveSet = document.getElementById('optObjectiveSet').value;
            const mutationText = document.getElementById('optMutationRate').value.trim();
            if (!mutationText) throw new RangeError('mutationRate must be a finite probability in [0, 1].');
            const mutationRate = Number(mutationText);
            const ranges = readWorkspaceRanges();
            const { seed, sampling } = readSamplingSettings();
            const servoRatings = ratingControls.read();

            const OptimizerClass = Optimizer === DefaultOptimizer ? WorkerOptimizer : Optimizer;
            const options = {
                generations,
                populationSize,
                objectiveSet,
                mutationRate,
                ranges,
                topology: document.getElementById('optTopology').value || 'c3_paired',
                referenceLayout: referenceLayoutInput.value.trim() || null,
                homeHeightBounds: readHomeHeightBounds(),
                sampling,
                seed,
                servoRatings,
                ballJointLimitDeg: Number(ballJointLimitInput.value),
                ballJointClamp: ballJointClampCheckbox.checked,
                onProgress: progress => reportProgress(thisRun, progress),
            };
            currentOptimizer = OptimizerClass === WorkerOptimizer
                ? new WorkerOptimizer(normalized, options, workerFactory ? { workerFactory } : {})
                : new OptimizerClass(normalized, options);
            if (currentOptimizer.topology) document.getElementById('optTopology').value = currentOptimizer.topology;

            resultOutput.value = '';
            lastOutcome = null;
            simulatorRun = null;
            simulatorController.clear();
            clearSimulatorSelection();
            document.getElementById('simDownload').disabled = true;
            resultsView.clear();
            const work = currentOptimizer.estimateWork();
            dashboard.start(thisRun, { candidates: work.evaluations,
                generations, poseWork: work.totalPoses });
            const reference = currentOptimizer.referenceDiagnostics;
            const migrationNote = currentOptimizer.referenceLayout?.migration?.note;
            const referenceNote = reference
                ? ` Reference: ${reference.boundsConflicts.length} search-bounds conflict(s); home pose ${reference.homePoseSatisfied ? 'valid' : 'invalid'}.${migrationNote ? ` ${migrationNote}` : ''}`
                : '';
            runReferenceNote = referenceNote;
            showStatus(`Optimization starting: ${work.totalPoses.toLocaleString()} pose evaluations.${referenceNote}`);
            setRunning(true);
            const outcome = await currentOptimizer.start();
            if (thisRun !== runSerial) return;

            presentOutcome(outcome, thisRun);
        } catch (error) {
            if (thisRun !== runSerial) return;
            dashboard.finish(thisRun, { status: 'failed',
                partialResults: currentOptimizer?.fitness?.length > 0, snapshot: finalSnapshot() });
            if (error.startupFailure) {
                showStatus(`Worker startup failed: ${error.message} Select “Run on main thread” to continue.`, true);
                offerFallback(true);
            } else {
                console.error(error);
                showStatus(error.message, true);
            }
        } finally {
            if (thisRun === runSerial) setRunning(false);
        }
    });

    function exportLayout() {
        try {
            if (!currentOptimizer) {
                showStatus('Run the optimization before exporting.', true);
                return;
            }
            const data = currentOptimizer.exportBest();
            if (data !== undefined) downloadFile(data, 'optimized_layout.json', 'application/json', document);
        } catch (error) {
            console.error(error);
            showStatus(error.message, true);
        }
    }

    function simulatorJSON() {
        const state = simulatorController.getState();
        if (!state.layout) throw new Error('Load a layout before exporting simulator state.');
        return JSON.stringify({ ...layoutToJSON(state.layout), run: simulatorRun,
            simulator: { source: state.source, requested: state.requested, accepted: state.accepted,
                options: state.options, animation: state.animation, markers: state.markers,
                tracesEnabled: state.tracesEnabled, trace: state.trace,
                camera: simulatorView.getCamera(), pointerMode: document.getElementById('simPointerMode').value } }, null, 2);
    }
    document.getElementById('simUseReference').addEventListener('click', () => {
        try {
            referenceLayoutInput.value = simulatorJSON();
            setTab('optimize');
            showStatus('Simulator geometry is ready as the optimizer reference.');
        } catch (error) { showStatus(error.message, true); }
    });
    document.getElementById('simDownload').addEventListener('click', () => {
        try { downloadFile(simulatorJSON(), 'stewart_simulator.json', 'application/json', document); }
        catch (error) { showStatus(error.message, true); }
    });
    function loadSimulatorLayout(parsed, activate = true) {
        const { layout, sourceRun } = importLayout(parsed);
        layout.id = parsed.id ?? parsed.layout?.id ?? parsed.result?.layout?.id ?? null;
        simulatorRun = sourceRun;
        const saved = parsed.simulator;
        simulatorController.loadLayout(layout, { source: { kind: 'import', candidateId: layout.id ?? null },
            options: saved?.options ?? simulatorOptions(sourceRun?.effective_settings, layout) });
        simCandidateSelect.innerHTML = `<option value="">Imported reference</option>${simCandidateSelect.innerHTML}`;
        simCandidateSelect.value = '';
        if (saved?.accepted) simulatorController.requestPose(saved.accepted, { source: 'replay' });
        if (saved?.requested) simulatorController.requestPose(saved.requested, { source: 'replay' });
        if (saved?.camera) simulatorView.setCamera(saved.camera);
        if (saved?.animation) {
            const pattern = saved.animation.pattern !== 'none' && ANIMATION_PATTERNS.includes(saved.animation.pattern)
                ? saved.animation.pattern : 'wobble';
            const speed = saved.animation.speed || 1;
            document.getElementById('simPattern').value = pattern;
            document.getElementById('simSpeed').value = String(speed);
            simulatorController.setAnimation(pattern, false, { speed });
        }
        if (saved?.pointerMode) document.getElementById('simPointerMode').value = saved.pointerMode;
        if (saved?.markers !== undefined) simulatorController.setMarkers(saved.markers);
        if (saved?.tracesEnabled !== undefined) simulatorController.setTraces(saved.tracesEnabled);
        document.getElementById('simDownload').disabled = false;
        if (activate) setTab('simulate');
    }
    document.getElementById('simLoadReference').addEventListener('click', () => {
        try {
            const raw = referenceLayoutInput.value.trim();
            if (!raw) throw new Error('Provide reference layout JSON in Optimize first.');
            loadSimulatorLayout(JSON.parse(raw));
        } catch (error) { showStatus(error.message, true); setTab('optimize'); }
    });

    function restoreLocalWorkspace(raw) {
        const saved = parseLocalWorkspace(raw);
        if (saved.simulator) importLayout(saved.simulator);
        currentOptimizer = null;
        lastOutcome = null;
        simulatorRun = null;
        offerFallback(false);
        simulatorController.clear();
        clearSimulatorSelection();
        document.getElementById('simDownload').disabled = true;
        resultsView.clear();
        resultOutput.value = '';
        dashboard.reset();
        if (saved.inputs.requirementsInput.trim()) {
            try { populate(parseRequirements(saved.inputs.requirementsInput)); }
            catch { /* Preserve unfinished requirements text and saved control values. */ }
        }
        applyLocalWorkspace(document, saved);
        if (saved.simulator) loadSimulatorLayout(saved.simulator, false);
        setRunning(false);
        showStatus(saved.simulator
            ? 'Local workspace restored. The saved layout is ready in Simulate; rerun optimization for candidate results.'
            : 'Local workspace restored. Run optimization to regenerate results.');
    }

    document.getElementById('saveLocalWorkspace').addEventListener('click', () => {
        try {
            const simulator = simulatorController.getState().layout ? JSON.parse(simulatorJSON()) : null;
            window.localStorage.setItem(LOCAL_WORKSPACE_KEY,
                JSON.stringify(captureLocalWorkspace(document, simulator)));
            showStatus('Workspace saved in this browser.');
        } catch (error) { showStatus(`Could not save in this browser: ${error.message}`, true); }
    });
    document.getElementById('restoreLocalWorkspace').addEventListener('click', () => {
        try {
            const raw = window.localStorage.getItem(LOCAL_WORKSPACE_KEY);
            if (!raw) throw new Error('No browser save exists.');
            restoreLocalWorkspace(raw);
        } catch (error) { showStatus(`Could not restore browser save: ${error.message}`, true); }
    });
    document.getElementById('deleteLocalWorkspace').addEventListener('click', () => {
        try {
            window.localStorage.removeItem(LOCAL_WORKSPACE_KEY);
            showStatus('Browser save deleted.');
        } catch (error) { showStatus(`Could not delete browser save: ${error.message}`, true); }
    });

    function exportCad(format) {
        try {
            const selected = currentOptimizer?.getSelectedCandidate?.();
            if (!selected) throw new Error('Select a completed candidate before CAD export.');
            const run = JSON.parse(currentOptimizer.exportBest()).run;
            const skeleton = buildConstructionSkeleton(selected, run);
            if (format === 'fusion') {
                downloadFile(skeletonToFusionScript(skeleton), 'stewart_construction.py', 'text/x-python', document);
            } else {
                downloadFile(skeletonToCSV(skeleton), 'stewart_coordinates.csv', 'text/csv', document);
            }
            if (skeleton.diagnostic) showStatus(`CAD construction geometry exported for diagnostic candidate ${skeleton.candidateId}; failed categories: ${skeleton.failedCategories.join(', ')}.`);
        } catch (error) {
            showStatus(error.message, true);
        }
    }

    downloadButton.addEventListener('click', () => {
        if (downloadFormat.value === 'json') exportLayout();
        else if (downloadFormat.value === 'fusion' || downloadFormat.value === 'csv') exportCad(downloadFormat.value);
    });

    const ready = loadDefaultRequirements()
        .then((json) => {
            requirementsInput.value = json;
            populate(parseRequirements(json));
            showStatus('Sample requirements loaded. Adjust parameters and run the optimizer.');
        })
        .catch((error) => {
            console.error(error);
            showStatus(error.message, true);
        })
        .then(() => {
            try {
                const saved = window.localStorage?.getItem(LOCAL_WORKSPACE_KEY);
                if (saved) restoreLocalWorkspace(saved);
            } catch (error) {
                showStatus(`Could not restore browser save: ${error.message}`, true);
            }
        });
    return { ready, simulatorController, simulatorView, geometryControls, simulatorDiagnostics, setTab };
}
