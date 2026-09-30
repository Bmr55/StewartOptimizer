import { validatePhysicalRequirements } from '../model/requirements.js';
import { degToRad } from '../math.js';
import { DEFAULT_DESIGN_SPACE, cloneLayout, createRandomLayout, finalizeLayout, mutateLayout, crossoverLayouts, validateDesignSpace } from './layout-operators.js';
import { dominates, fastNonDominatedSort, assignCrowdingDistance, tournamentSelect, selectFromFronts } from './nsga2.js';
import { evaluateLayout, evaluateCycle, computeFatigue } from './evaluate-layout.js';
import { estimateWork } from './budget.js';
import { selectBest, exportResult, layoutToJSON } from '../io/results.js';
import { DEFAULT_TOPOLOGY, TOPOLOGIES } from '../contracts.js';
import { normalizeSampling } from '../workspace/sampling.js';
import { createRandom, normalizeSeed, RANDOM_ALGORITHM } from './random.js';
import { validateConditionLimit } from '../model/conditioning.js';
import { importLayout } from '../io/layout-import.js';
import { evaluatePose } from '../model/pose.js';
import { initialPopulation, referenceBoundsConflicts, seedComposition } from './reference-seeding.js';
import { normalizeServoRatings } from '../model/servo-ratings.js';
import { normalizeObjectiveSet, objectiveDefinitions } from './objectives.js';

// Owns run state and population lifecycle. Numerical work and browser I/O live elsewhere.
export class Optimizer {
  static fromReplay(input, { onProgress } = {}) {
    const { sourceRun } = importLayout(input);
    const settings = sourceRun?.effective_settings;
    if (!settings || typeof settings !== 'object' || Array.isArray(settings)) {
      throw new TypeError('run.effective_settings is required to replay an exported result.');
    }
    if (settings.randomAlgorithm !== RANDOM_ALGORITHM) {
      throw new RangeError(`run.effective_settings.randomAlgorithm must be ${RANDOM_ALGORITHM}.`);
    }
    return new Optimizer(settings.requirements ?? {}, {
      ...settings,
      ranges: settings.bounds,
      referenceLayout: settings.reference_layout ?? null,
      onProgress,
    });
  }

  constructor(requirements = {}, {
    populationSize = 12,
    generations = 5,
    ranges = {},
    sampling = { strategy: 'halton' },
    seed = 1,
    mutationRate = 0.35,
    objectiveSet = 'compact',
    designSpace = {},
    topology = DEFAULT_TOPOLOGY,
    referenceLayout = null,
    homeHeightBounds,
    ballJointLimitDeg,
    lowerBallJointLimitDeg,
    upperBallJointLimitDeg,
    conditionLimit = null,
    ballJointClamp = false,
    servoRatings,
    onProgress,
    onCheckpoint,
  } = {}) {
    if (!Number.isSafeInteger(populationSize) || populationSize < 4
        || !Number.isSafeInteger(generations) || generations < 1) {
      throw new RangeError('Population must be an integer >= 4 and generations an integer >= 1.');
    }
    this.onProgress = onProgress;
    this.onCheckpoint = onCheckpoint;
    this.requirements = requirements;
    this.populationSize = Math.max(4, populationSize);
    this.generations = Math.max(1, generations);
    this.ranges = ranges;
    this.seed = normalizeSeed(seed);
    this.sampling = normalizeSampling(sampling.strategy === 'halton'
      ? { ...sampling, sequenceStart: sampling.sequenceStart ?? this.seed } : sampling);
    this.random = createRandom(this.seed);
    if (!Number.isFinite(mutationRate) || mutationRate < 0 || mutationRate > 1) {
      throw new RangeError('mutationRate must be a finite probability in [0, 1].');
    }
    this.mutationRate = mutationRate;
    this.objectiveSet = normalizeObjectiveSet(objectiveSet);
    const imported = referenceLayout == null ? null : importLayout(referenceLayout);
    this.referenceLayout = imported?.layout ?? null;
    this.referenceSourceRun = imported?.sourceRun ?? null;
    if (this.referenceLayout) topology = this.referenceLayout.topology;
    if (!TOPOLOGIES.includes(topology)) throw new Error(`topology must be one of ${TOPOLOGIES.join(', ')}.`);
    this.topology = topology;
    this.ballJointLimitDeg = ballJointLimitDeg ?? requirements.ball_joint_max_deg ?? 52;
    this.lowerBallJointLimitDeg = lowerBallJointLimitDeg ?? this.ballJointLimitDeg;
    this.upperBallJointLimitDeg = upperBallJointLimitDeg ?? this.ballJointLimitDeg;
    this.conditionLimit = validateConditionLimit(conditionLimit);
    this.ballJointClamp = ballJointClamp;
    const ratingKeys = ['servo_torque_rating_nm', 'servo_speed_rating_deg_s',
      'per_servo_ratings', 'servo_rating_policy'];
    this.servoRatingsInput = Object.fromEntries(ratingKeys
      .filter(key => Object.hasOwn(servoRatings ?? {}, key) || Object.hasOwn(requirements, key))
      .map(key => [key, Object.hasOwn(servoRatings ?? {}, key) ? servoRatings[key] : requirements[key]]));
    this.servoRatings = normalizeServoRatings(this.servoRatingsInput);
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
      homeHeightBounds: homeHeightBounds ?? requirements.home_height_bounds_mm
        ?? designSpace.homeHeightBounds ?? DEFAULT_DESIGN_SPACE.homeHeightBounds,
    };
    validateDesignSpace(this.designSpace);

    validatePhysicalRequirements({ mass_kg: this.payload, cycle_mm: this.stroke,
      frequency_hz: this.frequency, cycle_axis: this.cycleAxis,
      ball_joint_max_deg: this.ballJointLimitDeg, servo_travel_bounds_deg: this.servoRangeDeg,
      horn_length_bounds_mm: hornBounds, rod_length_bounds_mm: rodBounds,
      home_height_bounds_mm: this.designSpace.homeHeightBounds });
    for (const value of [this.lowerBallJointLimitDeg, this.upperBallJointLimitDeg]) {
      if (!Number.isFinite(value) || value < 0 || value > 180) {
        throw new RangeError('Ball-joint limits must be finite angles from 0 to 180 degrees.');
      }
    }

    this.servoRangeRad = this.servoRangeDeg.map((deg) => degToRad(deg));
    this.referenceDiagnostics = null;
    if (this.referenceLayout) {
      const home = evaluatePose(this.referenceLayout,
        { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 }, {
          ballJointLimitDeg: this.ballJointLimitDeg,
          lowerBallJointLimitDeg: this.lowerBallJointLimitDeg,
          upperBallJointLimitDeg: this.upperBallJointLimitDeg,
          conditionLimit: this.conditionLimit,
          servoRangeRad: this.referenceLayout.servoRangeRad,
          mounting: this.referenceLayout.mounting,
          recordLegData: true,
        });
      this.referenceDiagnostics = {
        boundsConflicts: referenceBoundsConflicts(this.referenceLayout, this.designSpace, this.servoRangeRad),
        homePoseSatisfied: home.reachable,
        homePoseViolations: home.violations,
      };
      this.referenceLayout.referenceDiagnostics = this.referenceDiagnostics;
    }

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

  layoutOptions() { return { designSpace: this.designSpace, servoRangeRad: this.servoRangeRad,
    topology: this.topology, random: this.random }; }
  finalizeLayout(layout) { return finalizeLayout(layout, this.layoutOptions()); }
  mutateLayout(layout) { return mutateLayout(layout, this.layoutOptions()); }
  crossoverLayouts(a, b) { return crossoverLayouts(a, b, this.layoutOptions()); }

  evaluationOptions() {
    return { ranges: this.ranges, signal: this.abortController?.signal,
      payload: this.payload, stroke: this.stroke, frequency: this.frequency, cycleAxis: this.cycleAxis,
      ballJointLimitDeg: this.ballJointLimitDeg,
      lowerBallJointLimitDeg: this.lowerBallJointLimitDeg,
      upperBallJointLimitDeg: this.upperBallJointLimitDeg,
      conditionLimit: this.conditionLimit,
      ballJointClamp: this.ballJointClamp,
      servoRangeRad: this.servoRangeRad, sampling: this.sampling,
      servoRatings: this.servoRatings, objectiveSet: this.objectiveSet };
  }

  evaluateLayout(layout) {
    const random = createRandom((this.seed ^ Math.imul(layout.id ?? 0, 0x9e3779b9)) >>> 0 || 1);
    let workspaceCompleted = 0;
    let extraCompleted = 0;
    const report = () => this.onProgress?.({
      completed: (this.completedPoseWork || 0) + workspaceCompleted + extraCompleted,
      total: this.workEstimate?.totalPoses ?? this.workEstimate?.posesPerLayout ?? workspaceCompleted + extraCompleted,
      budgeted: this.workEstimate?.totalPoses ?? null,
      generation: this.activeGeneration ?? this.generation,
    });
    return evaluateLayout(layout, { ...this.evaluationOptions(), random,
      onProgress: ({ completed, total }) => this.onProgress?.({
        completed: (this.completedPoseWork || 0) + (workspaceCompleted = completed) + extraCompleted,
        total: this.workEstimate?.totalPoses ?? total,
        budgeted: this.workEstimate?.totalPoses ?? null,
        generation: this.activeGeneration ?? this.generation,
      }),
      onPoseWork: () => { extraCompleted++; report(); },
    });
  }

  evaluateCycle(layout) { return evaluateCycle(layout, this.evaluationOptions()); }
  computeTorque(layout) { return this.evaluateCycle(layout).torqueNm; }
  computeSpeedDemand(layout) { return this.evaluateCycle(layout).speedRadPerSec; }
  computeFatigue(stats) { return computeFatigue(stats, this.evaluationOptions()); }
  dominates(a, b) { return dominates(a, b); }
  fastNonDominatedSort(evaluations) { return fastNonDominatedSort(evaluations); }
  assignCrowdingDistance(fronts, evaluations) { return assignCrowdingDistance(fronts, evaluations); }
  tournamentSelect(evaluations) { return tournamentSelect(evaluations, this.random); }

  createOffspring(evaluations) {
    const offspring = [];
    while (offspring.length < this.populationSize) {
      const parentA = this.tournamentSelect(evaluations);
      const parentB = this.tournamentSelect(evaluations);
      let child = this.crossoverLayouts(parentA.layout, parentB.layout);
      if (this.random() < this.mutationRate) {
        child = this.mutateLayout(child);
      }
      child.id = this.nextLayoutId++;
      child.seedOrigin = 'offspring';
      delete child.referenceDiagnostics;
      delete child.servoRangeDeg;
      delete child.migration;
      offspring.push(child);
    }
    return offspring;
  }

  selectFromFronts(evaluations, fronts) {
    const survivors = selectFromFronts(evaluations, fronts, this.populationSize);
    if (this.referenceEvaluation && !survivors.includes(this.referenceEvaluation)) {
      survivors[survivors.length - 1] = this.referenceEvaluation;
    }
    return survivors;
  }

  updateState(evaluations, fronts) {
    this.population = evaluations.map((ev) => cloneLayout(ev.layout));
    this.fitness = evaluations;
    this.pareto = fronts[0]?.map((idx) => evaluations[idx]) || [];
    if (this.running || !this.fitness.some(ev => ev.layout.id === this.selectedCandidateId)) {
      this.selectedCandidateId = selectBest(this.pareto, this.fitness)?.layout.id ?? null;
    }
  }

  emitCheckpoint() {
    this.onCheckpoint?.({ generation: this.generation, completedEvaluations: this.completedEvaluations,
      fitness: this.fitness, pareto: this.pareto });
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
      const result = await this.evaluateLayout(layout);
      if (layout.seedOrigin === 'reference') {
        result.referenceDiagnostics = this.referenceDiagnostics;
        if (this.referenceDiagnostics.boundsConflicts.length) {
          result.feasibility.failedCategories = [...new Set([
            ...(result.feasibility.failedCategories ?? []), 'geometry',
          ])];
          result.feasibility.passing = false;
        }
        this.referenceEvaluation = result;
      }
      results.push(result);
      this.completedPoseWork += result.workspace.total + 1
        + (result.cycle.failedSample ?? result.cycle.samples - 1) + 1;
      this.completedEvaluations += 1;
      this.onProgress?.({ completed: this.completedPoseWork,
        total: this.workEstimate.totalPoses, budgeted: this.workEstimate.totalPoses, generation: this.activeGeneration });
    }
    return results;
  }

  estimateWork() {
    return estimateWork({ ranges: this.ranges, sampling: this.sampling, stroke: this.stroke, frequency: this.frequency,
      populationSize: this.populationSize, generations: this.generations });
  }

  effectiveSettings() {
    return JSON.parse(JSON.stringify({
      requirements: this.requirements,
      bounds: this.ranges,
      sampling: this.sampling,
      seed: this.seed,
      randomAlgorithm: RANDOM_ALGORITHM,
      populationSize: this.populationSize,
      generations: this.generations,
      mutationRate: this.mutationRate,
      designSpace: this.designSpace,
      topology: this.topology,
      homeHeightBounds: this.designSpace.homeHeightBounds,
      ballJointLimitDeg: this.ballJointLimitDeg,
      lowerBallJointLimitDeg: this.lowerBallJointLimitDeg,
      upperBallJointLimitDeg: this.upperBallJointLimitDeg,
      conditionLimit: this.conditionLimit,
      ballJointClamp: this.ballJointClamp,
      servoRangeDeg: this.servoRangeDeg,
      servoRatings: this.servoRatingsInput,
      effectiveServoRatings: this.servoRatings,
      servoRatingPolicy: this.servoRatings.policy,
      objectiveSet: this.objectiveSet,
      objectiveDefinitions: objectiveDefinitions(this.objectiveSet),
      reference_layout: this.referenceLayout ? layoutToJSON(this.referenceLayout) : null,
      seed_composition: this.referenceLayout ? seedComposition(this.populationSize) : null,
    }));
  }

  async executeRun() {
    this.workEstimate = this.estimateWork();
    this.completedEvaluations = 0;
    this.completedPoseWork = 0;
    this.nextLayoutId = 1;
    this.random = createRandom(this.seed);
    this.population = initialPopulation(this);
    let evaluations = await this.evaluatePopulation(this.population);
    let fronts = this.fastNonDominatedSort(evaluations);
    this.assignCrowdingDistance(fronts, evaluations);
    this.updateState(evaluations, fronts);
    this.emitCheckpoint();

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
      this.emitCheckpoint();
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
    this.nextLayoutId = 1;
    this.referenceEvaluation = null;
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
      effective_settings: this.effectiveSettings(),
    }), null, 2);
  }
}
