import { validatePhysicalRequirements } from '../model/requirements.js';
import { clamp, degToRad } from '../math.js';
import { DEFAULT_DESIGN_SPACE, cloneLayout, createRandomLayout, finalizeLayout, mutateLayout, crossoverLayouts } from './layout-operators.js';
import { dominates, fastNonDominatedSort, assignCrowdingDistance, tournamentSelect, selectFromFronts } from './nsga2.js';
import { evaluateLayout, evaluateCycle, computeFatigue } from './evaluate-layout.js';
import { estimateWork } from './budget.js';
import { selectBest, exportResult } from '../io/results.js';

// Owns run state and population lifecycle. Numerical work and browser I/O live elsewhere.
export class Optimizer {
  constructor(requirements = {}, {
    populationSize = 12,
    generations = 5,
    ranges = {},
    mutationRate = 0.35,
    designSpace = {},
    ballJointLimitDeg,
    lowerBallJointLimitDeg,
    upperBallJointLimitDeg,
    ballJointClamp = false,
    onProgress,
  } = {}) {
    if (!Number.isSafeInteger(populationSize) || populationSize < 4
        || !Number.isSafeInteger(generations) || generations < 1) {
      throw new RangeError('Population must be an integer >= 4 and generations an integer >= 1.');
    }
    this.onProgress = onProgress;
    this.requirements = requirements;
    this.populationSize = Math.max(4, populationSize);
    this.generations = Math.max(1, generations);
    this.ranges = ranges;
    this.mutationRate = clamp(mutationRate, 0, 1);
    this.ballJointLimitDeg = ballJointLimitDeg ?? requirements.ball_joint_max_deg ?? 52;
    this.lowerBallJointLimitDeg = lowerBallJointLimitDeg ?? this.ballJointLimitDeg;
    this.upperBallJointLimitDeg = upperBallJointLimitDeg ?? this.ballJointLimitDeg;
    this.ballJointClamp = ballJointClamp;
    this.payload = requirements.mass_kg ?? 0;
    this.stroke = requirements.cycle_mm ?? 0;
    this.frequency = requirements.frequency_hz ?? 0;
    this.cycleAxis = requirements.cycle_axis ?? 'z';

    const hornBounds = requirements.horn_length_bounds_mm || DEFAULT_DESIGN_SPACE.hornLengthBounds;
    const rodBounds = requirements.rod_length_bounds_mm || DEFAULT_DESIGN_SPACE.rodLengthBounds;
    this.servoRangeDeg = requirements.servo_travel_bounds_deg || [-120, 120];

    this.designSpace = {
      ...DEFAULT_DESIGN_SPACE,
      ...designSpace,
      hornLengthBounds: hornBounds,
      rodLengthBounds: rodBounds,
    };

    validatePhysicalRequirements({ mass_kg: this.payload, cycle_mm: this.stroke,
      frequency_hz: this.frequency, cycle_axis: this.cycleAxis,
      ball_joint_max_deg: this.ballJointLimitDeg, servo_travel_bounds_deg: this.servoRangeDeg,
      horn_length_bounds_mm: hornBounds, rod_length_bounds_mm: rodBounds });
    for (const value of [this.lowerBallJointLimitDeg, this.upperBallJointLimitDeg]) {
      if (!Number.isFinite(value) || value < 0 || value > 180) {
        throw new RangeError('Ball-joint limits must be finite angles from 0 to 180 degrees.');
      }
    }

    this.servoRangeRad = this.servoRangeDeg.map((deg) => degToRad(deg));

    this.population = [];
    this.fitness = [];
    this.pareto = [];
    this.generation = 0;
    this.running = false;
    this.runStatus = 'idle';
    this.nextLayoutId = 1;
    this.selectedCandidateId = null;
  }

  createRandomLayout() {
    return createRandomLayout({ ...this.layoutOptions(), id: this.nextLayoutId++ });
  }

  layoutOptions() { return { designSpace: this.designSpace, servoRangeRad: this.servoRangeRad }; }
  finalizeLayout(layout) { return finalizeLayout(layout, this.layoutOptions()); }
  mutateLayout(layout) { return mutateLayout(layout, this.layoutOptions()); }
  crossoverLayouts(a, b) { return crossoverLayouts(a, b, this.layoutOptions()); }

  evaluationOptions() {
    return { ranges: this.ranges, signal: this.abortController?.signal,
      payload: this.payload, stroke: this.stroke, frequency: this.frequency, cycleAxis: this.cycleAxis,
      ballJointLimitDeg: this.ballJointLimitDeg,
      lowerBallJointLimitDeg: this.lowerBallJointLimitDeg,
      upperBallJointLimitDeg: this.upperBallJointLimitDeg,
      ballJointClamp: this.ballJointClamp,
      servoRangeRad: this.servoRangeRad };
  }

  evaluateLayout(layout) {
    return evaluateLayout(layout, { ...this.evaluationOptions(),
      onProgress: ({ completed, total }) => this.onProgress?.({
        completed: (this.completedEvaluations || 0) * (this.workEstimate?.posesPerLayout ?? total) + completed,
        total: this.workEstimate?.totalPoses ?? total,
        generation: this.activeGeneration ?? this.generation,
      }),
    });
  }

  evaluateCycle(layout) { return evaluateCycle(layout, this.evaluationOptions()); }
  computeTorque(layout) { return this.evaluateCycle(layout).torqueNm; }
  computeSpeedDemand(layout) { return this.evaluateCycle(layout).speedRadPerSec; }
  computeFatigue(stats) { return computeFatigue(stats, this.evaluationOptions()); }
  dominates(a, b) { return dominates(a, b); }
  fastNonDominatedSort(evaluations) { return fastNonDominatedSort(evaluations); }
  assignCrowdingDistance(fronts, evaluations) { return assignCrowdingDistance(fronts, evaluations); }
  tournamentSelect(evaluations) { return tournamentSelect(evaluations); }

  createOffspring(evaluations) {
    const offspring = [];
    while (offspring.length < this.populationSize) {
      const parentA = this.tournamentSelect(evaluations);
      const parentB = this.tournamentSelect(evaluations);
      let child = this.crossoverLayouts(parentA.layout, parentB.layout);
      if (Math.random() < this.mutationRate) {
        child = this.mutateLayout(child);
      }
      offspring.push(child);
    }
    return offspring;
  }

  selectFromFronts(evaluations, fronts) { return selectFromFronts(evaluations, fronts, this.populationSize); }

  updateState(evaluations, fronts) {
    this.population = evaluations.map((ev) => cloneLayout(ev.layout));
    this.fitness = evaluations;
    this.pareto = fronts[0]?.map((idx) => evaluations[idx]) || [];
    if (this.running || !this.fitness.some(ev => ev.layout.id === this.selectedCandidateId)) {
      this.selectedCandidateId = selectBest(this.pareto, this.fitness)?.layout.id ?? null;
    }
  }

  getSelectedCandidate() {
    return this.fitness.find(ev => ev.layout.id === this.selectedCandidateId)
      || selectBest(this.pareto, this.fitness);
  }

  selectCandidate(id) {
    const selected = this.fitness.find(ev => String(ev.layout.id) === String(id));
    if (!selected) throw new RangeError(`Candidate ${id} is not in the retained population.`);
    this.selectedCandidateId = selected.layout.id;
    return selected;
  }

  async evaluatePopulation(layouts) {
    const results = [];
    for (const layout of layouts) {
      this.abortController?.signal.throwIfAborted();
      results.push(await this.evaluateLayout(layout));
      this.completedEvaluations += 1;
      this.onProgress?.({ completed: this.completedEvaluations * this.workEstimate.posesPerLayout,
        total: this.workEstimate.totalPoses, generation: this.activeGeneration });
    }
    return results;
  }

  estimateWork() {
    return estimateWork({ ranges: this.ranges, stroke: this.stroke, frequency: this.frequency,
      populationSize: this.populationSize, generations: this.generations });
  }

  async executeRun() {
    this.workEstimate = this.estimateWork();
    this.completedEvaluations = 0;
    this.population = Array.from({ length: this.populationSize }, () => this.createRandomLayout());
    let evaluations = await this.evaluatePopulation(this.population);
    let fronts = this.fastNonDominatedSort(evaluations);
    this.assignCrowdingDistance(fronts, evaluations);
    this.updateState(evaluations, fronts);

    for (let gen = 0; gen < this.generations; gen++) {
      this.abortController?.signal.throwIfAborted();
      this.activeGeneration = gen + 1;
      const offspringLayouts = this.createOffspring(evaluations);
      const offspringEvaluations = await this.evaluatePopulation(offspringLayouts);
      const combined = evaluations.concat(offspringEvaluations);
      fronts = this.fastNonDominatedSort(combined);
      this.assignCrowdingDistance(fronts, combined);
      evaluations = this.selectFromFronts(combined, fronts);
      fronts = this.fastNonDominatedSort(evaluations);
      this.assignCrowdingDistance(fronts, evaluations);
      this.updateState(evaluations, fronts);
      this.generation = gen + 1;
    }
  }

  async run() {
    if (this.running) throw new Error('An optimization is already running.');
    this.running = true;
    this.runStatus = 'running';
    this.abortController = new AbortController();
    this.population = [];
    this.fitness = [];
    this.pareto = [];
    this.selectedCandidateId = null;
    this.generation = 0;
    this.activeGeneration = 0;
    this.completedEvaluations = 0;
    try {
      await this.executeRun();
      this.runStatus = 'completed';
    } catch (error) {
      if (error.name !== 'AbortError') {
        this.runStatus = 'failed';
        throw error;
      }
      this.runStatus = 'cancelled';
    } finally {
      this.running = false;
      this.abortController = null;
    }
    return { status: this.runStatus, completedGenerations: this.generation,
      completedEvaluations: this.completedEvaluations,
      partialResults: this.runStatus === 'cancelled' && this.fitness.length > 0 };
  }

  start(callback) {
    return this.run().then(outcome => {
      callback?.(this, outcome);
      return outcome;
    });
  }

  stop() {
    this.abortController?.abort();
  }

  exportBest(format = 'json') {
    if (this.running) throw new Error('Wait for completion or cancellation before exporting.');
    if (!this.fitness.length) {
      console.warn('No evaluated layouts available for export.');
      return;
    }
    const best = this.getSelectedCandidate();
    if (!best) {
      console.warn('Unable to determine best layout.');
      return;
    }
    if (format !== 'json') {
      console.warn('Only JSON export is currently supported.');
      return;
    }
    return JSON.stringify(exportResult(best, {
      status: this.runStatus, completedGenerations: this.generation, partial: this.runStatus !== 'completed',
      effective_settings: this.effectiveSettings?.() ?? {
        requirements: this.requirements, ranges: this.ranges,
        populationSize: this.populationSize, generations: this.generations,
        mutationRate: this.mutationRate, designSpace: this.designSpace,
        ballJointLimitDeg: this.ballJointLimitDeg, ballJointClamp: this.ballJointClamp,
      },
    }), null, 2);
  }
}
