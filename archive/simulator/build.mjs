import { build } from 'esbuild';
import { fileURLToPath } from 'node:url';
await build({
  absWorkingDir: fileURLToPath(new URL('.', import.meta.url)),
  entryPoints: ['browser-entry.js'], outfile: 'stewart.min.js',
  bundle: true, minify: true, format: 'iife', platform: 'browser',
  target: 'es2020', legalComments: 'inline'
});
