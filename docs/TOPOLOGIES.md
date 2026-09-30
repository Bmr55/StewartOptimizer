# Layout topologies

New searches default to **C3 paired**. Select Circular, C3 paired, Rectangular paired, or Free in the optimization controls, or pass `topology` to the headless `Optimizer` constructor. Internal anchor coordinates and lengths use millimeters; angles use radians.

| Topology | Anchor invariant | Search parameters |
| --- | --- | --- |
| `circular` | Six equally spaced anchors on each centered, coplanar ring; alternating horn orientations. | `base_radius`, `platform_radius`, `base_orientation`, `platform_orientation`, `beta_offset`, `beta_pair_offset` |
| `c3_paired` | Three identical pairs on each centered, coplanar ring, repeated every 120°. Each pair's gap is a chord length. | Base/platform radii and orientations, `beta_offset`, `base_pair_gap`, `platform_pair_gap` |
| `rectangular_paired` | Three rows of left/right paired anchors on each centered, coplanar plate, rotated together; alternating horn orientations. Rows are at negative, zero, and positive half-depth. | Circular fields plus `base_aspect`, `platform_aspect` (half-width / half-depth) |
| `free` | Six independently positioned anchors and beta angles. Generated platform anchors stay in the platform plane (finalization sets their Z to 0, so an out-of-plane reference anchor is not inherited); generated base anchor Z varies within its configured jitter. | No invariant parameters (`{}`) |

For rectangular layouts, `base_radius` and `platform_radius` are the distance from plate center to an **outer corner**. Their bounds apply to this envelope. The middle-row anchors lie inside it. Circular and C3 radii apply to every anchor of their respective rings. A parametric horn's beta angle starts tangent to its base anchor's center-to-anchor ray plus the shared `beta_offset`. Circular and Rectangular then subtract `beta_pair_offset` on legs 1/3/5 and add it on legs 2/4/6. New layouts start this alternating offset at 30° (stored in radians); mutation and crossover can evolve it while preserving the plate geometry and repeated horn pattern. This removes the forced home singularity of the previous common-direction families; the evaluator still rejects any individual singular or otherwise invalid candidate. C3 uses only the shared offset, and Free beta angles evolve independently.

`beta_pair_offset` is optional for compatibility: an omitted value means zero and reproduces the old horn directions exactly. Imported references retain their coordinates and directions, including singular legacy designs. Generated variations may evolve the offset; importing does not silently repair it. Crossover bounds coupled radii and pair gaps before constructing offspring, so an out-of-bounds diagnostic reference cannot create an invalid intermediate gap/radius combination.

The default base radius remains 90–160 mm and platform radius 40–120 mm. The headless `designSpace` option can change these, `pairGapBounds`, `rectangularAspectBounds`, and the horn/rod length bounds; the browser UI exposes none of them. Effective parameters and generated Free anchor radii are kept within configured bounds during generation, mutation, crossover, and finalization. The default horn bounds are 30–120 mm in the direct API and 30–110 mm when omitted from parsed requirements; rod bounds are 160–420 mm in both. Horn, rod and home-height bounds resolve in the same order: explicit requirement bounds (`horn_length_bounds_mm`, `rod_length_bounds_mm`, `home_height_bounds_mm`) win, then the headless `designSpace` option, then `DEFAULT_DESIGN_SPACE`; the direct `homeHeightBounds` option sits above the requirement value for height only.

## Design-space constants and operators

`DEFAULT_DESIGN_SPACE` in `src/optimization/layout-operators.js` fixes the remaining search constants:

| Constant | Default | Role |
| --- | --- | --- |
| `pairGapBounds` | [12, 45] mm | C3 pair gap. Each plate's ceiling is also capped at 1.2 × that plate's radius, and C3 radii are floored at the minimum gap / 1.2 so a gap always fits. |
| `rectangularAspectBounds` | [0.6, 1.4] | Rectangular width/depth ratio |
| `betaJitterRad` | 20° | Range of the random shared `beta_offset` for new parametric layouts; Gaussian scale for new Free beta angles |
| `anchorJitter`, `platformJitter` | 6 mm | Gaussian mutation scale for base/platform radii and pair gaps, and for Free anchor X/Y |
| `baseZJitter` | 2 mm | Free base-anchor Z range at generation, mutation scale and finalization clamp |
| `mutationHorn`, `mutationRod`, `mutationHeight` | 4, 6, 15 mm | Gaussian mutation scales for the shared lengths and home height |
| `mutationAngle` | 4° | Gaussian mutation scale for plate orientations, `beta_offset`, `beta_pair_offset` and Free beta angles; rectangular aspects mutate with a fixed 0.05 scale |

New Free layouts place base anchors on a hexagon at a random start angle with 3° Gaussian jitter, platform anchors offset by 30° with the same jitter, base Z within `baseZJitter`, and beta angles tangent to each base ray plus `betaJitterRad` noise. New parametric layouts draw radii, orientations (full circle), `beta_offset`, gaps and aspects uniformly within their bounds.

Mutation perturbs every parameter (or every Free anchor coordinate and beta angle) with Gaussian noise at the scales above, then the shared lengths and home height, and re-finalizes. Crossover copies parent A, then for parametric layouts takes each topology parameter from either parent with equal probability and re-bounds the coupled radius/gap choices before regenerating anchors; for Free layouts it takes legs from a random split index onward from parent B. Horn and rod lengths are averaged and home height comes from either parent. Each offspring is mutated with probability `mutationRate`. Parents with different topologies cannot be crossed.

Home height is a separate variable. It is sampled, inherited, and mutated independently of rods and horns. The default bounds are **50–450 mm**, configurable as `home_height_bounds_mm` in requirements, by the two UI controls, or as `homeHeightBounds` in the headless constructor. An explicit constructor/UI value wins over requirements; requirements win over `designSpace.homeHeightBounds`. Finalization only clamps height to these bounds. Feasibility evaluation reports whether the chosen height permits a home pose; it does not replace the chosen value with a derived height.

Layouts carry `topology` and `topologyParameters` internally. JSON uses `topology` and `topology_parameters` with the snake-case keys in the table above. A declared parametric topology must recreate the actual six base anchors, six platform anchors, and six beta angles. `validateTopology` returns a field-specific error when metadata and geometry disagree. An absent topology is treated as Free without changing coordinates. The [reference importer](./IMPORT.md) validates dimensions and preserves a diagnostic reference when its search bounds or home pose fail.
