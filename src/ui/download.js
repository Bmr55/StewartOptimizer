// Firefox, and historically Safari, start the navigation for an `a[download]`
// click asynchronously. Revoking the Blob URL before that navigation is
// dispatched can cancel the download, so the revoke waits for a later task.
export const REVOKE_DELAY_MS = 1000;

export function download(data, filename = 'optimized_layout.json', mime = 'application/json',
  document = globalThis.document, { revokeDelayMs = REVOKE_DELAY_MS, setTimeout = globalThis.setTimeout } = {}) {
  const blob = new Blob([data], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  try { a.click(); }
  finally {
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), revokeDelayMs);
  }
}
