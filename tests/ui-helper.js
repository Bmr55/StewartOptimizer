import fs from 'node:fs';
import { createApp } from '../src/ui/app.js';
export const sampleText = fs.readFileSync(new URL('../Additional_Repo_Stuff/examples/Sample_Requirements.json', import.meta.url), 'utf8');
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
  const app = createApp({ document, window: { addEventListener() {} },
    Optimizer, loadDefaultRequirements: async () => sampleText, ...options });
  await app.ready;
  element('optPopulation').value = '4';
  element('optGenerations').value = '1';
  return element;
}
