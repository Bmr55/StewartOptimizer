import { mkdir, writeFile } from 'node:fs/promises';
import { Info } from 'lucide';

const directory = new URL('../assets/icons/', import.meta.url);
const shapes = Info.map(([element, attributes]) => {
  const properties = Object.entries(attributes).map(([name, value]) => `${name}="${value}"`).join(' ');
  return `  <${element} ${properties} />`;
}).join('\n');
const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">\n${shapes}\n</svg>\n`;

await mkdir(directory, { recursive: true });
await writeFile(new URL('info.svg', directory), svg);
