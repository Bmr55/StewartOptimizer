import fs from 'node:fs';
import vm from 'node:vm';
import { parseRequirements } from '../requirements.js';
export const sampleText = fs.readFileSync(new URL('../Additional_Repo_Stuff/examples/Sample_Requirements.json', import.meta.url), 'utf8');
export async function loadUI(Optimizer) {
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      value: '', checked: false, disabled: false, handlers: {}, textContent: '',
      classList: { toggle() {}, contains() { return false; }, remove() {}, add() {} },
      addEventListener(type, fn) { this.handlers[type] = fn; },
      setAttribute() {}, style: {}
    });
    return elements.get(id);
  };
  const document = { getElementById: element, createElement: () => element('popup'),
    body: { appendChild() {} }, addEventListener() {} };
  const context = vm.createContext({ document, window: { addEventListener() {} }, console,
    parseRequirements, Optimizer, loadDefaultRequirements: async () => sampleText });
  const html = fs.readFileSync(new URL('../index.html', import.meta.url), 'utf8');
  const script = html.match(/<script type="module">([\s\S]*?)<\/script>/)[1].replace(/^\s*import .*;$/gm, '');
  vm.runInContext(script, context);
  await new Promise(setImmediate);
  element('optPopulation').value = '4';
  element('optGenerations').value = '1';
  return element;
}
