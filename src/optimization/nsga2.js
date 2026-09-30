function normalizedObjectives(objectives) {
  return objectives.map((value) => (Number.isFinite(value) ? value : -Infinity));
}

export function dominates(a, b) {
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

export function fastNonDominatedSort(evaluations) {
  const n = evaluations.length;
  const dominationCounts = new Array(n).fill(0);
  const dominatedSets = Array.from({ length: n }, () => []);
  const fronts = [];

  for (let i = 0; i < n; i++) {
    evaluations[i].rank = Infinity;
    for (let j = 0; j < n; j++) {
      if (i === j) continue;
      if (dominates(evaluations[i].objectives, evaluations[j].objectives)) {
        dominatedSets[i].push(j);
      } else if (dominates(evaluations[j].objectives, evaluations[i].objectives)) {
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

export function assignCrowdingDistance(fronts, evaluations) {
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

export function tournamentSelect(evaluations) {
  const pick = () => evaluations[Math.floor(Math.random() * evaluations.length)];
  const a = pick();
  const b = pick();
  if (a.rank < b.rank) return a;
  if (b.rank < a.rank) return b;
  if (a.crowding > b.crowding) return a;
  if (b.crowding > a.crowding) return b;
  return Math.random() < 0.5 ? a : b;
}

export function selectFromFronts(evaluations, fronts, populationSize) {
  const selected = [];
  for (const front of fronts) {
    const frontEvaluations = front.map((idx) => evaluations[idx]);
    if (selected.length + frontEvaluations.length <= populationSize) {
      selected.push(...frontEvaluations);
    } else {
      const remaining = populationSize - selected.length;
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
