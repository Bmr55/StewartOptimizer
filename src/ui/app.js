import { selectBest, displayResult } from '../io/results.js';
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
    let runSerial = 0;
    let runReferenceNote = '';
    const dashboard = createRunDashboard(document, { now });
    const fallbackButton = document.getElementById('runMainThreadFallback');
    function offerFallback(available) {
        fallbackButton.hidden = !available;
        fallbackButton.disabled = !available;
    }
    offerFallback(false);
    const { populateRequirementsDefaults, readWorkspaceRanges, readHomeHeightBounds,
        readSamplingSettings, randomizeSeed } = createControls(document);
    const ratingControls = createServoRatingControls(document);
    function populate(parsed, preserveEdits = false) {
        populateRequirementsDefaults(parsed, preserveEdits);
        ratingControls.populate(parsed.normalized, preserveEdits);
    }
    installTooltips(document, window);
    const resultsView = createResultsView(document, (candidate) => {
        if (!currentOptimizer || currentOptimizer.running) return;
        currentOptimizer.selectCandidate(candidate.layout.id);
        resultOutput.value = JSON.stringify({ run: lastOutcome, result: displayResult(candidate) }, null, 2);
        setRunning(false);
    });
    resultsView.clear();

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

    function setRunning(running) {
        ratingControls.setDisabled(running);
        for (const id of ['runOptimization', 'loadSampleRequirements', 'clearRequirements',
            'optSampling', 'optSeed', 'randomizeSeed', 'optPopulation', 'optGenerations',
            'optObjectiveSet', 'optMutationRate', 'clearReferenceLayout', 'referenceLayoutFile']) {
            document.getElementById(id).disabled = running;
        }
        document.getElementById('cancelOptimization').disabled = !running;
        if (running) fallbackButton.disabled = true;
        document.getElementById('exportBestLayout').disabled = running
            || !(currentOptimizer?.getSelectedCandidate?.() || currentOptimizer?.fitness?.length);
        const cadAvailable = !running && canExportCad(currentOptimizer?.getSelectedCandidate?.());
        document.getElementById('exportFusionScript').disabled = !cadAvailable;
        document.getElementById('exportCoordinateCsv').disabled = !cadAvailable;
    }

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
        resultOutput.value = best ? JSON.stringify({ run: lastOutcome, result: displayResult(best) }, null, 2) : '';
        dashboard.finish(thisRun, { status: outcome.status, partialResults: outcome.partialResults,
            snapshot: finalSnapshot() });
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

    document.getElementById('exportBestLayout').addEventListener('click', () => {
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

    document.getElementById('exportFusionScript').addEventListener('click', () => exportCad('fusion'));
    document.getElementById('exportCoordinateCsv').addEventListener('click', () => exportCad('csv'));

    const ready = loadDefaultRequirements()
        .then((json) => {
            requirementsInput.value = json;
            populate(parseRequirements(json));
            showStatus('Sample requirements loaded. Adjust parameters and run the optimizer.');
        })
        .catch((error) => {
            console.error(error);
            showStatus(error.message, true);
        });
    return { ready };
}
