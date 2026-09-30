import { parseRequirements, loadDefaultRequirements as loadSample } from '../../requirements.js';
import { Optimizer as DefaultOptimizer } from '../../optimizer.js';
import { createControls } from './controls.js';
import { installTooltips } from './tooltips.js';

export function createApp({ document, window, Optimizer = DefaultOptimizer, loadDefaultRequirements = loadSample }) {
    const requirementsInput = document.getElementById('requirementsInput');
    const statusEl = document.getElementById('optStatus');
    const resultOutput = document.getElementById('resultOutput');
    const ballJointClampCheckbox = document.getElementById('ballJointClamp');
    const ballJointLimitInput = document.getElementById('ballJointLimit');
    let currentOptimizer = null;
    const { populateRequirementsDefaults, readWorkspaceRanges } = createControls(document);
    installTooltips(document, window);

    function showStatus(message, isError = false) {
        statusEl.textContent = message;
        statusEl.classList.toggle('error', isError);
    }

    function serializeBest(evaluation) {
        if (!evaluation) return '';
        return JSON.stringify({
            metrics: {
                coverage: evaluation.coverage,
                relaxedCoverage: evaluation.relaxedCoverage,
                dexterity: evaluation.dexterity,
                stiffness: evaluation.stiffness,
                torque: evaluation.torque,
                speedDemand: evaluation.speedDemand,
                loadBalance: evaluation.loadBalance,
                isotropy: evaluation.isotropy,
                limitMargin: evaluation.limitMargin,
                fatigue: evaluation.fatigue,
            },
            layout: {
                base_anchors: evaluation.layout.baseAnchors,
                platform_anchors: evaluation.layout.platformAnchors,
                beta_angles: evaluation.layout.betaAngles,
                horn_length: evaluation.layout.hornLength,
                rod_length: evaluation.layout.rodLength,
                servo_range: evaluation.layout.servoRangeRad.map((rad) => rad * 180 / Math.PI),
                home_height: evaluation.layout.homeHeight,
            },
            cycle: evaluation.cycle,
            feasibility: evaluation.feasibility,
            constraint_policy: evaluation.workspace?.constraintPolicy,
            workspace_stats: evaluation.workspace?.stats,
            workspace_counts: evaluation.workspace?.counts,
            workspace_samples: evaluation.workspace?.samples,
        }, null, 2);
    }

    document.getElementById('loadSampleRequirements').addEventListener('click', async () => {
        try {
            const json = await loadDefaultRequirements();
            requirementsInput.value = json;
            populateRequirementsDefaults(parseRequirements(json));
            showStatus('Sample requirements loaded.');
        } catch (error) {
            console.error(error);
            showStatus(error.message, true);
        }
    });

    document.getElementById('clearRequirements').addEventListener('click', () => {
        requirementsInput.value = '';
        resultOutput.value = '';
        showStatus('Requirements cleared.');
    });

    function setRunning(running) {
        for (const id of ['runOptimization', 'loadSampleRequirements', 'clearRequirements']) {
            document.getElementById(id).disabled = running;
        }
        document.getElementById('cancelOptimization').disabled = !running;
        document.getElementById('exportBestLayout').disabled = running || !currentOptimizer?.fitness.length;
    }

    document.getElementById('cancelOptimization').addEventListener('click', () => {
        currentOptimizer?.stop();
        showStatus('Cancelling optimization...');
    });

    document.getElementById('runOptimization').addEventListener('click', async () => {
        if (currentOptimizer?.running) return;
        try {
            const text = requirementsInput.value.trim();
            if (!text) {
                throw new Error('Provide requirements JSON before running the optimizer.');
            }
            const { normalized, workspace } = parseRequirements(text);
            populateRequirementsDefaults({ normalized, workspace }, true);

            const generations = Number(document.getElementById('optGenerations').value);
            const populationSize = Number(document.getElementById('optPopulation').value);
            const ranges = readWorkspaceRanges();

            currentOptimizer = new Optimizer(normalized, {
                generations,
                populationSize,
                ranges,
                ballJointLimitDeg: Number(ballJointLimitInput.value),
                ballJointClamp: ballJointClampCheckbox.checked,
                onProgress: ({ completed, total, generation }) => showStatus(
                    `Generation ${generation}: ${completed.toLocaleString()} / ${total.toLocaleString()} pose evaluations (${(100 * completed / total).toFixed(1)}%).`
                ),
            });

            resultOutput.value = '';
            const work = currentOptimizer.estimateWork();
            showStatus(`Optimization starting: ${work.totalPoses.toLocaleString()} pose evaluations.`);
            setRunning(true);
            const outcome = await currentOptimizer.start();

            const pareto = currentOptimizer.pareto && currentOptimizer.pareto.length ? currentOptimizer.pareto : currentOptimizer.fitness;
            const best = pareto.slice().sort((a, b) => b.coverage - a.coverage)[0];
            resultOutput.value = best ? JSON.stringify({ run: outcome, result: JSON.parse(serializeBest(best)) }, null, 2) : '';
            if (outcome.status === 'cancelled') {
                showStatus(best ? 'Optimization cancelled. Showing partial results from the last completed population.' : 'Optimization cancelled before a population completed.');
                return;
            }
            showStatus(`Optimization complete. Feasible coverage: ${best?.coverage ?? 0}%. Pareto front contains ${currentOptimizer.pareto.length || pareto.length} layouts. Coverage applies only to sampled poses and modeled constraints.`);
        } catch (error) {
            console.error(error);
            showStatus(error.message, true);
        } finally {
            setRunning(false);
        }
    });

    document.getElementById('exportBestLayout').addEventListener('click', () => {
        try {
            if (!currentOptimizer) {
                showStatus('Run the optimization before exporting.', true);
                return;
            }
            currentOptimizer.exportBest();
        } catch (error) {
            console.error(error);
            showStatus(error.message, true);
        }
    });

    const ready = loadDefaultRequirements()
        .then((json) => {
            requirementsInput.value = json;
            populateRequirementsDefaults(parseRequirements(json));
            showStatus('Sample requirements loaded. Adjust parameters and run the optimizer.');
        })
        .catch((error) => {
            console.error(error);
            showStatus(error.message, true);
        });
    return { ready };
}
