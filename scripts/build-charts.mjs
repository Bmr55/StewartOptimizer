import { copyFile, mkdir } from 'node:fs/promises';

const source = new URL('../node_modules/chart.js/', import.meta.url);
const target = new URL('../assets/vendor/', import.meta.url);
await mkdir(target, { recursive: true });
await copyFile(new URL('dist/chart.umd.js', source), new URL('chart.umd.js', target));
await copyFile(new URL('LICENSE.md', source), new URL('chart.js-LICENSE.md', target));
