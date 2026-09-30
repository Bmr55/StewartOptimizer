export async function loadDefaultRequirements() {
  const response = await fetch('Additional_Repo_Stuff/examples/Sample_Requirements.json');
  if (!response.ok) throw new Error(`Failed to load sample requirements: HTTP ${response.status}`);
  return JSON.stringify(await response.json(), null, 2);
}
