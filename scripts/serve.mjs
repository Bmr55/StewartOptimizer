import http from 'node:http';
import { readFile, realpath, stat } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const repositoryRoot = fileURLToPath(new URL('../', import.meta.url));
const mimeTypes = {
  '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript',
  '.css': 'text/css', '.json': 'application/json', '.md': 'text/plain',
  '.svg': 'image/svg+xml', '.png': 'image/png', '.ico': 'image/x-icon',
};

// Local development only: no directory listings, hidden files, or paths outside root.
export async function startServer({ port = 8000, root = repositoryRoot } = {}) {
  if (!Number.isInteger(port) || port < 0 || port > 65535) throw new RangeError('Port must be an integer from 0 to 65535.');
  root = await realpath(root);
  const inside = file => file === root || file.startsWith(root + path.sep);
  const server = http.createServer(async (request, response) => {
    const send = (status, text) => { response.writeHead(status, { 'Content-Type': 'text/plain' }); response.end(text); };
    if (!['GET', 'HEAD'].includes(request.method)) { send(405, 'Method not allowed'); return; }
    let pathname;
    try { pathname = decodeURIComponent(request.url.split('?')[0]); }
    catch { send(400, 'Invalid URL'); return; }
    if (!pathname.startsWith('/') || /[\\:\0]/.test(pathname)
      || pathname.split('/').some(part => part.startsWith('.') || part === 'node_modules')) {
      send(403, 'Forbidden'); return;
    }
    try {
      let file = await realpath(path.resolve(root, '.' + pathname));
      if (!inside(file)) { send(403, 'Forbidden'); return; }
      if ((await stat(file)).isDirectory()) file = await realpath(path.join(file, 'index.html'));
      if (!inside(file)) { send(403, 'Forbidden'); return; }
      const body = await readFile(file);
      response.writeHead(200, {
        'Content-Type': mimeTypes[path.extname(file)] || 'application/octet-stream',
        'Content-Length': body.length, 'Cache-Control': 'no-store',
      });
      response.end(request.method === 'HEAD' ? undefined : body);
    } catch (error) {
      send(['ENOENT', 'ENOTDIR', 'EISDIR'].includes(error.code) ? 404 : 500, 'File unavailable');
    }
  });
  await new Promise((resolve, reject) => {
    server.once('error', reject);
    server.listen(port, '127.0.0.1', () => { server.off('error', reject); resolve(); });
  });
  return server;
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    const args = process.argv.slice(2);
    if (args.length && (args.length !== 2 || args[0] !== '--port' || !/^\d+$/.test(args[1]))) {
      throw new Error('Usage: npm run dev -- --port 8000');
    }
    const server = await startServer({ port: args.length ? Number(args[1]) : 8000 });
    console.log(`Stewart Optimizer: http://127.0.0.1:${server.address().port}`);
    for (const signal of ['SIGINT', 'SIGTERM']) process.once(signal, () => server.close());
  } catch (error) {
    console.error(error.code === 'EADDRINUSE' ? 'Port is occupied. Use npm run dev -- --port 8001.' : error.message);
    process.exitCode = 1;
  }
}
