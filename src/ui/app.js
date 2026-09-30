import { selectBest, displayResult } from '../io/results.js';
import { download } from './download.js';
import { parseRequirements } from '../model/requirements.js';
import { loadDefaultRequirements as loadSample } from '../io/sample-requirements.js';
import { Optimizer as DefaultOptimizer } from '../optimization/optimizer.js';
import { createControls } from './controls.js';
import { installTooltips } from './tooltips.js';
import { createResultsView } from './results-view.js';
import { buildConstructionSkeleton, canExportCad, skeletonToCSV, skeletonToFusionScript } from '../io/cad.js';
import { createServoRatingControls } from './servo-ratings-controls.js';

export function createApp({ document, window, Optimizer = DefaultOptimizer, loadDefaultRequirements = loadSample, downloadFile = download }) {
    const requirementsInput = document.getElementById('requirementsInput');
    const referenceLayoutInput = document.getElementById('referenceLayoutInput');
    const statusEl = document.getElementById('optStatus');
    const resultOutput = document.getElementById('resultOutput');
    const ballJointClampCheckbox = document.getElementById('ballJointClamp');
    const ballJointLimitInput = document.getElementById('ballJointLimit');
    let currentOptimizer = null;
    let lastOutcome = null;
    let runSerial = 0;
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
        resultsView.clear();
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
            'optSampling', 'optSeed', 'randomizeSeed', 'clearReferenceLayout', 'referenceLayoutFile']) {
            document.getElementById(id).disabled = running;
        }
        document.getElementById('cancelOptimization').disabled = !running;
        document.getElementById('exportBestLayout').disabled = running
            || !(currentOptimizer?.getSelectedCandidate?.() || currentOptimizer?.fitness?.length);
        const cadAvailable = !running && canExportCad(currentOptimizer?.getSelectedCandidate?.());
        document.getElementById('exportFusionScript').disabled = !cadAvailable;
        document.getElementById('exportCoordinateCsv').disabled = !cadAvailable;
    }

    document.getElementById('cancelOptimization').addEventListener('click', () => {
        currentOptimizer?.stop();
        showStatus('Cancelling optimization...');
    });

    document.getElementById('runOptimization').addEventListener('click', async () => {
        if (currentOptimizer?.running) return;
        const thisRun = ++runSerial;
        try {
            const text = requirementsInput.value.trim();
            if (!text) {
                throw new Error('Provide requirements JSON before running the optimizer.');
            }
            const { normalized, workspace } = parseRequirements(text);
            populate({ normalized, workspace }, true);

            const generations = Number(document.getElementById('optGenerations').value);
            const populationSize = Number(document.getElementById('optPopulation').value);
            const ranges = readWorkspaceRanges();
            const { seed, sampling } = readSamplingSettings();
            const servoRatings = ratingControls.read();

            currentOptimizer = new Optimizer(normalized, {
                generations,
                populationSize,
                ranges,
                topology: document.getElementById('optTopology').value || 'c3_paired',
                referenceLayout: referenceLayoutInput.value.trim() || null,
                homeHeightBounds: readHomeHeightBounds(),
                sampling,
                seed,
                servoRatings,
                ballJointLimitDeg: Number(ballJointLimitInput.value),
                ballJointClamp: ballJointClampCheckbox.checked,
                onProgress: ({ completed, total, generation }) => showStatus(
                    `Generation ${generation}: ${completed.toLocaleString()} / ${total.toLocaleString()} pose evaluations (${(100 * completed / total).toFixed(1)}%).`
                ),
            });
            if (currentOptimizer.topology) document.getElementById('optTopology').value = currentOptimizer.topology;

            resultOutput.value = '';
            lastOutcome = null;
            resultsView.clear();
            const work = currentOptimizer.estimateWork();
            const reference = currentOptimizer.referenceDiagnostics;
            const migrationNote = currentOptimizer.referenceLayout?.migration?.note;
            const referenceNote = reference
                ? ` Reference: ${reference.boundsConflicts.length} search-bounds conflict(s); home pose ${reference.homePoseSatisfied ? 'valid' : 'invalid'}.${migrationNote ? ` ${migrationNote}` : ''}`
                : '';
            showStatus(`Optimization starting: ${work.totalPoses.toLocaleString()} pose evaluations.${referenceNote}`);
            setRunning(true);
            const outcome = await currentOptimizer.start();
            if (thisRun !== runSerial) return;

            const pareto = currentOptimizer.pareto && currentOptimizer.pareto.length ? currentOptimizer.pareto : currentOptimizer.fitness;
            const best = currentOptimizer.getSelectedCandidate?.()
                ?? selectBest(currentOptimizer.pareto, currentOptimizer.fitness);
            lastOutcome = { ...outcome,
                effective_settings: currentOptimizer.effectiveSettings?.() ?? null };
            resultsView.render(currentOptimizer.fitness, best?.layout.id);
            resultOutput.value = best ? JSON.stringify({ run: lastOutcome, result: displayResult(best) }, null, 2) : '';
            if (outcome.status === 'cancelled') {
                showStatus(best ? 'Optimization cancelled. Showing partial results from the last completed population.' : 'Optimization cancelled before a population completed.');
                return;
            }
            showStatus(`Optimization complete. Feasible coverage: ${best?.coverage ?? 0}%. Pareto front contains ${currentOptimizer.pareto.length || pareto.length} layouts. Coverage applies only to sampled poses and modeled constraints.${referenceNote}`);
        } catch (error) {
            if (thisRun !== runSerial) return;
            console.error(error);
            showStatus(error.message, true);
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
