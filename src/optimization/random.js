// Mulberry32 is a small, specified 32-bit generator for replay, not cryptography.
export const RANDOM_ALGORITHM = 'mulberry32-v1';

export function normalizeSeed(seed = 1) {
  if (!Number.isSafeInteger(seed) || seed < 1 || seed > 0xffffffff) {
    throw new RangeError('Seed must be an integer from 1 to 4,294,967,295.');
  }
  return seed;
}

export function createRandom(seed = 1) {
  let state = normalizeSeed(seed) >>> 0;
  return () => {
    state = (state + 0x6D2B79F5) >>> 0;
    let value = state;
    value = Math.imul(value ^ value >>> 15, value | 1);
    value ^= value + Math.imul(value ^ value >>> 7, value | 61);
    return ((value ^ value >>> 14) >>> 0) / 0x100000000;
  };
}
