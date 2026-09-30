import test from 'node:test';
import assert from 'node:assert/strict';
import { dominates, fastNonDominatedSort, assignCrowdingDistance, selectFromFronts, tournamentSelect } from '../src/optimization/nsga2.js';

test('non-dominated sorting keeps tradeoffs and ranks invalid objectives last', () => {
  const evaluations = [[3,1], [2,2], [1,3], [1,1], [NaN,1]].map(objectives => ({ objectives }));
  const fronts = fastNonDominatedSort(evaluations);
  assert.deepEqual(fronts, [[0,1,2], [3], [4]]);
  assert.deepEqual(evaluations.map(e => e.rank), [0,0,0,1,2]);
  assert.equal(dominates([2,2], [2,2]), false);
  assignCrowdingDistance(fronts, evaluations);
  assert.deepEqual(evaluations.slice(0,3).map(e => e.crowding), [Infinity,2,Infinity]);
  assert.deepEqual(selectFromFronts(evaluations, fronts, 2), [evaluations[0], evaluations[2]]);
});

test('tournament prefers rank, then crowding, then breaks a tie', t => {
  let draws = [];
  t.mock.method(Math, 'random', () => draws.shift());
  const a = {rank:0, crowding:1}, b = {rank:1, crowding:Infinity};
  draws = [0,0.9];
  assert.equal(tournamentSelect([a,b]), a);
  b.rank = 0;
  draws = [0,0.9];
  assert.equal(tournamentSelect([a,b]), b);
  a.crowding = Infinity;
  draws = [0,0.9,0.9];
  assert.equal(tournamentSelect([a,b]), b);
});
