# Layout topologies

New searches default to **C3 paired**. Select Circular, C3 paired, Rectangular paired, or Free in the optimization controls, or pass `topology` to the headless `Optimizer` constructor. Internal anchor coordinates and lengths use millimeters; angles use radians.

| Topology | Anchor invariant | Search parameters |
| --- | --- | --- |
| `circular` | Six equally spaced anchors on each centered, coplanar ring. | `base_radius`, `platform_radius`, `base_orientation`, `platform_orientation`, `beta_offset` |
| `c3_paired` | Three identical pairs on each centered, coplanar ring, repeated every 120°. Each pair's gap is a chord length. | Circular fields plus `base_pair_gap`, `platform_pair_gap` |
| `rectangular_paired` | Three rows of left/right paired anchors on each centered, coplanar plate, rotated together. Rows are at negative, zero, and positive half-depth. | Circular fields plus `base_aspect`, `platform_aspect` (half-width / half-depth) |
| `free` | Six independently positioned anchors and beta angles. Generated platform anchors stay in the platform plane; generated base anchor Z varies within its configured jitter. | No invariant parameters (`{}`) |

For rectangular layouts, `base_radius` and `platform_radius` are the distance from plate center to an **outer corner**. Their bounds apply to this envelope. The middle-row anchors lie inside it. Circular and C3 radii apply to every anchor of their respective rings. Every parametric family's beta angle is tangent to its base anchor's center-to-anchor ray plus one shared `beta_offset`. This rule rotates and mutates with the geometry. Free beta angles evolve independently.

The default base radius remains 90–160 mm and platform radius 40–120 mm. The headless `designSpace` option can change these, `pairGapBounds`, `rectangularAspectBounds`, and the horn/rod length bounds. Effective parameters and generated Free anchor radii are kept within configured bounds during generation, mutation, crossover, and finalization. The default horn bounds are 30–120 mm in the direct API and 30–110 mm when omitted from parsed requirements; rod bounds are 160–420 mm in both. Explicit requirement horn/rod bounds retain their existing precedence.

Home height is a separate variable. It is sampled, inherited, and mutated independently of rods and horns. The default bounds are **50–450 mm**, configurable as `home_height_bounds_mm` in requirements, by the two UI controls, or as `homeHeightBounds` in the headless constructor. An explicit constructor/UI value wins over requirements; requirements win over `designSpace.homeHeightBounds`. Finalization only clamps height to these bounds. Feasibility evaluation reports whether the chosen height permits a home pose; it does not replace the chosen value with a derived height.

Layouts carry `topology` and `topologyParameters` internally. JSON uses `topology` and `topology_parameters` with the snake-case keys in the table above. A declared parametric topology must recreate the actual six base anchors, six platform anchors, and six beta angles. `validateTopology` returns a field-specific error when metadata and geometry disagree. An absent topology is treated as Free without changing coordinates. The [reference importer](./IMPORT.md) validates dimensions and preserves a diagnostic reference when its search bounds or home pose fail.
