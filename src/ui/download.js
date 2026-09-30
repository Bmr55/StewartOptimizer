export function download(data, filename = 'optimized_layout.json', mime = 'application/json', document = globalThis.document) {
  const blob = new Blob([data], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  try { a.click(); }
  finally { a.remove(); URL.revokeObjectURL(url); }
}
