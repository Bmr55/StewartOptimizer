import { validatePhysicalRequirements } from './requirements.js';
import { computeCycleDemand } from './cycle.js';
import { computeWorkspace, evaluatePose, estimateWorkspaceSize } from './workspace.js';
import {
  clamp,
  singularValues,
  randomNormal,
  degToRad,
  radToDeg,
  average,
} from './math.js';
const EPS = 1e-9;

const DEFAULT_DESIGN_SPACE = {
  baseRadius: [90, 160],
  platformRadius: [40, 120],
  platformHeight: [100, 180],
  hornLengthBounds: [30, 120],
  rodLengthBounds: [160, 420],
  betaJitterRad: degToRad(20),
  anchorJitter: 6,
  platformJitter: 6,
  baseZJitter: 2,
  mutationHorn: 4,
  mutationRod: 6,
  mutationAngle: degToRad(4),
};

function randomInRange([min, max]) {
  if (max <= min) return min;
  return min + Math.random() * (max - min);
}

function wrapAngle(angle) {
  let a = angle;
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function cloneLayout(layout) {
  return JSON.parse(JSON.stringify(layout));
}

function layoutToJSON(layout, metrics = {}) {
  return {
    base_anchors: layout.baseAnchors.map((a) => a.slice()),
    platform_anchors: layout.platformAnchors.map((a) => a.slice()),
    beta_angles: layout.betaAngles.slice(),
    horn_length: layout.hornLength,
    rod_length: layout.rodLength,
    servo_range: layout.servoRangeRad.map((rad) => radToDeg(rad)),
    home_height: layout.homeHeight,
    metadata: {
      coverage: metrics.coverage ?? null,
      relaxed_coverage: metrics.relaxedCoverage ?? null,
      dexterity: metrics.dexterity ?? null,
      stiffness: metrics.stiffness ?? null,
      torque: metrics.torque ?? null,
      speed_demand: metrics.speedDemand ?? null,
      load_balance: metrics.loadBalance ?? null,
      isotropy: metrics.isotropy ?? null,
      limit_margin: metrics.limitMargin ?? null,
      fatigue: metrics.fatigue ?? null,
    },
    cycle: metrics.cycle ?? null,
    feasibility: metrics.feasibility ?? null,
    constraint_policy: metrics.workspace?.constraintPolicy ?? null,
    workspace_counts: metrics.workspace?.counts ?? null,
    workspace_stats: metrics.workspace?.stats ?? null,
  };
}

function normalizedObjectives(objectives) {
  return objectives.map((value) => (Number.isFinite(value) ? value : -Infinity));
}

export class Optimizer {
  constructor(requirements = {}, {
    populationSize = 12,
    generations = 5,
    ranges = {},
    mutationRate = 0.35,
    designSpace = {},
    ballJointLimitDeg,
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

    this.servoRangeRad = this.servoRangeDeg.map((deg) => degToRad(deg));

    this.population = [];
    this.fitness = [];
    this.pareto = [];
    this.generation = 0;
    this.running = false;
    this.runStatus = 'idle';
    this.nextLayoutId = 1;
  }

  createRandomLayout() {
    const layout = {
      id: this.nextLayoutId++,
      baseAnchors: [],
      platformAnchors: [],
      betaAngles: [],
      hornLength: randomInRange(this.designSpace.hornLengthBounds),
      rodLength: randomInRange(this.designSpace.rodLengthBounds),
      servoRangeRad: this.servoRangeRad.slice(),
      homeHeight: 120,
    };

    const baseRadius = randomInRange(this.designSpace.baseRadius);
    const platformRadius = clamp(randomInRange(this.designSpace.platformRadius), 20, baseRadius - 10);
    const platformHeight = randomInRange(this.designSpace.platformHeight);
    const baseOffset = Math.random() * 2 * Math.PI;
    const platformOffset = baseOffset + Math.PI / 6;

    for (let i = 0; i < 6; i++) {
      const angle = baseOffset + i * (Math.PI / 3) + randomNormal() * degToRad(3);
      const platformAngle = platformOffset + i * (Math.PI / 3) + randomNormal() * degToRad(3);
      const baseAnchor = [
        baseRadius * Math.cos(angle) + randomNormal() * this.designSpace.anchorJitter,
        baseRadius * Math.sin(angle) + randomNormal() * this.designSpace.anchorJitter,
        randomNormal() * this.designSpace.baseZJitter,
      ];
      const platformAnchor = [
        platformRadius * Math.cos(platformAngle) + randomNormal() * this.designSpace.platformJitter,
        platformRadius * Math.sin(platformAngle) + randomNormal() * this.designSpace.platformJitter,
        0,
      ];
      const beta = angle + Math.PI / 2 + randomNormal() * this.designSpace.betaJitterRad;

      layout.baseAnchors.push(baseAnchor);
      layout.platformAnchors.push(platformAnchor);
      layout.betaAngles.push(wrapAngle(beta));
    }

    layout.homeHeight = platformHeight;
    this.finalizeLayout(layout);
    return layout;
  }

  finalizeLayout(layout) {
    const hornMin = this.designSpace.hornLengthBounds[0];
    const hornMax = this.designSpace.hornLengthBounds[1];
    const rodMin = this.designSpace.rodLengthBounds[0];
    const rodMax = this.designSpace.rodLengthBounds[1];
    layout.hornLength = clamp(layout.hornLength, hornMin, hornMax);
    layout.rodLength = clamp(layout.rodLength, rodMin, rodMax);

    const heightContributions = [];
    let requiredRodSq = rodMin * rodMin;

    for (let i = 0; i < 6; i++) {
      const base = layout.baseAnchors[i];
      const platform = layout.platformAnchors[i];
      const dx = platform[0] - base[0];
      const dy = platform[1] - base[1];
      const horizontalSq = dx * dx + dy * dy;
      const required = horizontalSq - layout.hornLength * layout.hornLength + 1;
      if (required > requiredRodSq) {
        requiredRodSq = required;
      }
    }

    layout.rodLength = clamp(Math.max(layout.rodLength, Math.sqrt(Math.max(requiredRodSq, 0))), rodMin, rodMax);

    for (let i = 0; i < 6; i++) {
      const base = layout.baseAnchors[i];
      const platform = layout.platformAnchors[i];
      const dx = platform[0] - base[0];
      const dy = platform[1] - base[1];
      const horizontalSq = dx * dx + dy * dy;
      const radicand = layout.rodLength * layout.rodLength
        + layout.hornLength * layout.hornLength
        - horizontalSq;
      heightContributions.push(Math.sqrt(Math.max(radicand, 0)));
    }

    layout.homeHeight = average(heightContributions);
    layout.servoRangeRad = this.servoRangeRad.slice();
    return layout;
  }

  mutateLayout(source) {
    const layout = cloneLayout(source);
    for (let i = 0; i < 6; i++) {
      layout.baseAnchors[i][0] += randomNormal() * this.designSpace.anchorJitter;
      layout.baseAnchors[i][1] += randomNormal() * this.designSpace.anchorJitter;
      layout.baseAnchors[i][2] += randomNormal() * this.designSpace.baseZJitter;
      layout.platformAnchors[i][0] += randomNormal() * this.designSpace.platformJitter;
      layout.platformAnchors[i][1] += randomNormal() * this.designSpace.platformJitter;
      layout.betaAngles[i] = wrapAngle(layout.betaAngles[i] + randomNormal() * this.designSpace.mutationAngle);
    }

    layout.hornLength += randomNormal() * this.designSpace.mutationHorn;
    layout.rodLength += randomNormal() * this.designSpace.mutationRod;

    return this.finalizeLayout(layout);
  }

  crossoverLayouts(a, b) {
    const layout = cloneLayout(a);
    const split = Math.floor(Math.random() * 6);
    for (let i = split; i < 6; i++) {
      layout.baseAnchors[i] = b.baseAnchors[i].slice();
      layout.platformAnchors[i] = b.platformAnchors[i].slice();
      layout.betaAngles[i] = b.betaAngles[i];
    }
    layout.hornLength = (a.hornLength + b.hornLength) / 2;
    layout.rodLength = (a.rodLength + b.rodLength) / 2;
    layout.servoRangeRad = this.servoRangeRad.slice();
    return this.finalizeLayout(layout);
  }

  async evaluateLayout(layout) {
    const workspaceResult = await computeWorkspace(layout, this.ranges, {
      signal: this.abortController?.signal,
      onProgress: ({ completed, total }) => this.onProgress?.({
        completed: (this.completedEvaluations || 0) * (this.workEstimate?.posesPerLayout ?? total) + completed,
        total: this.workEstimate?.totalPoses ?? total,
        generation: this.activeGeneration ?? this.generation,
      }),
      payload: this.payload,
      stroke: this.stroke,
      frequency: this.frequency,
      ballJointLimitDeg: this.ballJointLimitDeg,
      ballJointClamp: this.ballJointClamp,
    });

    const coverage = Number.isFinite(workspaceResult.coverage) ? workspaceResult.coverage : 0;
    const relaxedCoverage = workspaceResult.relaxedCoverage ?? coverage;
    const stats = workspaceResult.stats || {};

    const homeResult = evaluatePose(layout, {
      x: 0,
      y: 0,
      z: 0,
      rx: 0,
      ry: 0,
      rz: 0,
    }, {
      ballJointLimitDeg: this.ballJointLimitDeg,
      ballJointClamp: this.ballJointClamp,
      servoRangeRad: layout.servoRangeRad,
      recordLegData: true,
    });

    let dexterity = 0;
    let stiffness = 0;
    let condition = Infinity;
    if (homeResult.reachable && homeResult.jacobianRows.length === 6) {
      const sv = singularValues(homeResult.jacobianRows);
      const sigmaMax = Math.max(...sv, EPS);
      const sigmaMin = sv.filter((v) => v > EPS).reduce((min, val) => Math.min(min, val), Infinity);
      if (sigmaMax > EPS && sigmaMin < Infinity) {
        dexterity = sigmaMin / sigmaMax;
        stiffness = sigmaMin;
        condition = sigmaMax / sigmaMin;
      }
    }

    const cycle = this.evaluateCycle(layout);
    const torque = cycle.torqueNm;
    const speedDemand = cycle.speedRadPerSec;
    const loadBalance = stats.loadBalanceScore ?? 0;
    const isotropy = stats.averageIsotropy ?? 0;
    const stiffnessScore = stats.averageStiffness > 0 ? stats.averageStiffness : stiffness;
    const ballLimit = degToRad(this.ballJointLimitDeg || 0);
    const ballMarginRaw = ballLimit > 0 && Number.isFinite(stats.ballJointOverallMax)
      ? 1 - stats.ballJointOverallMax / ballLimit
      : 1;
    const violationMargin = 1 - (stats.violationRate ?? 0);
    const limitMargin = clamp(Math.max(ballMarginRaw, 0) * Math.max(violationMargin, 0), 0, 1);
    const fatigue = this.computeFatigue(stats);
    const objectives = [
      coverage,
      this.ballJointClamp ? relaxedCoverage : coverage,
      dexterity,
      stiffnessScore,
      loadBalance,
      isotropy,
      limitMargin,
      cycle.valid ? -torque : -Infinity,
      cycle.valid ? -speedDemand : -Infinity,
      -fatigue,
    ];

    return {
      layout,
      workspace: workspaceResult,
      coverage,
      relaxedCoverage,
      cycle,
      feasibility: {
        cycleSatisfied: cycle.valid,
        sampledWorkspaceSatisfied: coverage === 100,
        homePoseSatisfied: homeResult.reachable,
        scope: 'Sampled poses under the modeled geometry, servo, rod and ball-joint constraints only',
      },
      dexterity,
      stiffness: stiffnessScore,
      torque,
      speedDemand,
      loadBalance,
      isotropy,
      limitMargin,
      fatigue,
      condition,
      objectives,
      homePose: homeResult,
      rank: Infinity,
      crowding: 0,
    };
  }

  evaluateCycle(layout) {
    return computeCycleDemand(layout, { mass: this.payload, stroke: this.stroke,
      frequency: this.frequency, axis: this.cycleAxis, ballJointLimitDeg: this.ballJointLimitDeg,
      signal: this.abortController?.signal });
  }

  computeTorque(layout) { return this.evaluateCycle(layout).torqueNm; }
  computeSpeedDemand(layout) { return this.evaluateCycle(layout).speedRadPerSec; }

  computeFatigue(stats) {
    if (!stats) return 0;
    const ballJointAvg = Number.isFinite(stats.ballJointAverage) ? stats.ballJointAverage : 0;
    const servoAvg = Number.isFinite(stats.servoUsageAvg) ? stats.servoUsageAvg : 0;
    const ballLimit = degToRad(this.ballJointLimitDeg || 0);
    const ballRatio = ballLimit > 0 ? ballJointAvg / ballLimit : 0;
    const servoSpan = Math.abs(this.servoRangeRad[1] - this.servoRangeRad[0]) || Math.PI;
    const servoDuty = servoSpan > 0 ? servoAvg / servoSpan : 0;
    const strokeMeters = this.stroke / 1000;
    return (Math.max(ballRatio, 0) + Math.max(servoDuty, 0)) * this.frequency * strokeMeters;
  }

  dominates(a, b) {
    const objA = normalizedObjectives(a);
    const objB = normalizedObjectives(b);
    let betterInAny = false;
    for (let i = 0; i < objA.length; i++) {
      if (objA[i] < objB[i]) {
        return false;
      }
      if (objA[i] > objB[i]) {
        betterInAny = true;
      }
    }
    return betterInAny;
  }

  fastNonDominatedSort(evaluations) {
    const n = evaluations.length;
    const dominationCounts = new Array(n).fill(0);
    const dominatedSets = Array.from({ length: n }, () => []);
    const fronts = [];

    for (let i = 0; i < n; i++) {
      evaluations[i].rank = Infinity;
      for (let j = 0; j < n; j++) {
        if (i === j) continue;
        if (this.dominates(evaluations[i].objectives, evaluations[j].objectives)) {
          dominatedSets[i].push(j);
        } else if (this.dominates(evaluations[j].objectives, evaluations[i].objectives)) {
          dominationCounts[i] += 1;
        }
      }
      if (dominationCounts[i] === 0) {
        evaluations[i].rank = 0;
        if (!fronts[0]) fronts[0] = [];
        fronts[0].push(i);
      }
    }

    let frontIndex = 0;
    while (fronts[frontIndex] && fronts[frontIndex].length) {
      const nextFront = [];
      for (const idx of fronts[frontIndex]) {
        for (const dominatedIdx of dominatedSets[idx]) {
          dominationCounts[dominatedIdx] -= 1;
          if (dominationCounts[dominatedIdx] === 0) {
            evaluations[dominatedIdx].rank = frontIndex + 1;
            nextFront.push(dominatedIdx);
          }
        }
      }
      if (nextFront.length) {
        fronts.push(nextFront);
      }
      frontIndex += 1;
    }

    return fronts;
  }

  assignCrowdingDistance(fronts, evaluations) {
    for (const front of fronts) {
      if (!front || front.length === 0) continue;
      for (const idx of front) {
        evaluations[idx].crowding = 0;
      }
      const objectiveCount = evaluations[front[0]].objectives.length;
      for (let m = 0; m < objectiveCount; m++) {
        const sorted = front.slice().sort((a, b) => {
          const va = evaluations[a].objectives[m];
          const vb = evaluations[b].objectives[m];
          return va - vb;
        });
        const minVal = evaluations[sorted[0]].objectives[m];
        const maxVal = evaluations[sorted[sorted.length - 1]].objectives[m];
        evaluations[sorted[0]].crowding = Infinity;
        evaluations[sorted[sorted.length - 1]].crowding = Infinity;
        if (!Number.isFinite(minVal) || !Number.isFinite(maxVal) || maxVal === minVal) continue;
        for (let i = 1; i < sorted.length - 1; i++) {
          if (!Number.isFinite(evaluations[sorted[i]].crowding)) continue;
          const prev = evaluations[sorted[i - 1]].objectives[m];
          const next = evaluations[sorted[i + 1]].objectives[m];
          evaluations[sorted[i]].crowding += (next - prev) / (maxVal - minVal);
        }
      }
    }
  }

  tournamentSelect(evaluations) {
    const pick = () => evaluations[Math.floor(Math.random() * evaluations.length)];
    const a = pick();
    const b = pick();
    if (a.rank < b.rank) return a;
    if (b.rank < a.rank) return b;
    if (a.crowding > b.crowding) return a;
    if (b.crowding > a.crowding) return b;
    return Math.random() < 0.5 ? a : b;
  }

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

  selectFromFronts(evaluations, fronts) {
    const selected = [];
    for (const front of fronts) {
      const frontEvaluations = front.map((idx) => evaluations[idx]);
      if (selected.length + frontEvaluations.length <= this.populationSize) {
        selected.push(...frontEvaluations);
      } else {
        const remaining = this.populationSize - selected.length;
        if (remaining > 0) {
          const sorted = frontEvaluations
            .slice()
            .sort((a, b) => (b.crowding ?? -Infinity) - (a.crowding ?? -Infinity));
          selected.push(...sorted.slice(0, remaining));
        }
        break;
      }
    }
    return selected;
  }

  updateState(evaluations, fronts) {
    this.population = evaluations.map((ev) => cloneLayout(ev.layout));
    this.fitness = evaluations;
    this.pareto = fronts[0]?.map((idx) => evaluations[idx]) || [];
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
    const workspacePosesPerLayout = estimateWorkspaceSize(this.ranges);
    const cyclePosesPerLayout = this.stroke > 0 && this.frequency > 0 ? 64 : 1;
    const posesPerLayout = workspacePosesPerLayout + cyclePosesPerLayout + 1;
    const evaluations = this.populationSize * (this.generations + 1);
    const totalPoses = posesPerLayout * evaluations;
    if (!Number.isSafeInteger(totalPoses) || totalPoses > 1000000) {
      throw new RangeError('Run exceeds 1,000,000 pose evaluations. Increase sweep steps or reduce population/generations.');
    }
    return { posesPerLayout, workspacePosesPerLayout, cyclePosesPerLayout, evaluations, totalPoses };
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
    const bestFront = this.pareto.length ? this.pareto : this.fitness;
    const best = bestFront.slice().sort((a, b) => b.coverage - a.coverage)[0];
    if (!best) {
      console.warn('Unable to determine best layout.');
      return;
    }
    if (format !== 'json') {
      console.warn('Only JSON export is currently supported.');
      return;
    }
    const json = layoutToJSON(best.layout, best);
    json.run = { status: this.runStatus, completedGenerations: this.generation, partial: this.runStatus !== 'completed' };
    const data = JSON.stringify(json, null, 2);
    this.download(data, 'optimized_layout.json', 'application/json');
  }

  download(data, filename, mime) {
    const blob = new Blob([data], { type: mime });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }
}
