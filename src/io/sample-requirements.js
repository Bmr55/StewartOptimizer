export async function loadDefaultRequirements() {
  const response = await fetch(new URL('../../examples/sample-requirements.json', import.meta.url));
  if (!response.ok) throw new Error(`Failed to load sample requirements: HTTP ${response.status}`);
  return JSON.stringify(await response.json(), null, 2);
}
