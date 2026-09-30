import fs from 'node:fs';
import { createApp } from '../../src/ui/app.js';
export const sampleText = fs.readFileSync(new URL('../../examples/sample-requirements.json', import.meta.url), 'utf8');
class FakeChart {
  constructor(_canvas, config) { this.data = config.data; this.options = config.options; }
  update() {}
}
export async function loadUI(Optimizer, options = {}) {
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      _value: '', get value() { return this._value; }, set value(value) { this._value = String(value); }, checked: false, disabled: false, handlers: {}, textContent: '',
      classList: { toggle() {}, contains() { return false; }, remove() {}, add() {} },
      addEventListener(type, fn) { this.handlers[type] = fn; },
      setAttribute() {}, style: {}
    });
    return elements.get(id);
  };
  const document = { getElementById: element, createElement: () => element('popup'),
    body: { appendChild() {} }, addEventListener() {} };
  element('downloadFormat').value = 'json';
  const app = createApp({ document, window: { addEventListener() {} },
    Optimizer, ChartClass: FakeChart, loadDefaultRequirements: async () => sampleText, ...options });
  await app.ready;
  element('optPopulation').value = '4';
  element('optGenerations').value = '1';
  element('optObjectiveSet').value = 'compact';
  element('optMutationRate').value = '0.35';
  return element;
}
