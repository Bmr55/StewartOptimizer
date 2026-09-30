# Physical stiffness and compliance

Model identity: `cartesian-compliance-v1`. The existing `stiffness` metric is unchanged: it is the dimensionless minimum singular value of the conditioning Jacobian, a geometric proxy (see [CONDITIONING_MODEL.md](./CONDITIONING_MODEL.md)). The physical model below is separate and optional.

## Inputs

`constraints.stiffness_model` (or a flat `stiffness_model`):

| Field | Meaning |
| --- | --- |
| `servo_torsional_stiffness_nm_per_rad` | Output-shaft servo torsional stiffness, N m/rad; one positive number or six in servo order. Required |
| `rod_axial_stiffness_n_per_m` | Axial rod stiffness, N/m |
| `rod_material` | `{ youngs_modulus_gpa, area_mm2 }` or `{ youngs_modulus_gpa, diameter_mm }` (solid round); stiffness is E A / L with L the candidate's rod length |
| `rods` | `"rigid"` for the servo-compliance-only limit |
| `characteristic_length_mm` | Length that scales rotations for the scalar score; defaults to the platform RMS anchor radius (recorded) |
| `test_wrenches` | `[{ name, force_n: [x, y, z], moment_nm: [x, y, z] }]` applied to the platform at the moving origin, base-frame components |
| `use_as_objective` | `true` replaces the proxy with `physicalStiffness` in the Full objective set |

Exactly one rod description is required, and `rod_material` needs exactly one of `area_mm2` or `diameter_mm`; only omission selects the other, so an explicit `null` is rejected like every other stiffness-model sub-field. Without `stiffness_model`, physical stiffness is `unavailable` (null metric), never a default value.

## Model

At the home pose (the stated operating pose), with actual rod directions `u_i`, moving-origin anchor arms `r_i` (m) and `A_i = [u_i^T, (r_i x u_i)^T]`, each leg is a servo spring referred through its transmission `D_ii = u_i . h'_i` in series with its rod spring:

`1 / k_leg,i = 1 / k_rod + D_ii^2 / k_servo,i` (N/m)

The Cartesian stiffness about the moving origin in base-frame axes is `K = A^T diag(k_leg) A`, mapping a small twist `[dx, dy, dz (m), rx, ry, rz (rad)]` to the wrench `[F (N), M (N m)]`. With rigid rods it reduces to `K = G^T K_servo G`, where `G = D^-1 A` is the physical rotary mapping (`alphaDot = G V`) from [CYCLE_MODEL.md](./CYCLE_MODEL.md). The dimensionless conditioning matrix is never substituted for `G`.

`K` is symmetric positive semidefinite. Blocks carry N/m (translation), N m/rad (rotation) and N/rad or N (coupling). The result (`physical_stiffness` in exports) reports `K`, the compliance `C = K^-1`, per-leg series stiffness, each leg's servo share of its compliance, and for each test wrench the predicted translation (mm), rotation (deg) and stored energy (J).

The scalar metric `physical_stiffness` (N/m) is the minimum eigenvalue of `S K S` with `S = diag(1, 1, 1, 1/L, 1/L, 1/L)`, so every entry is N/m: a rotation `theta` counts like a translation `L theta`. Raw mixed-unit eigenvalues are not compared. `L` and its source are exported. If the smallest scaled eigenvalue is at most 1e-10 of the largest, the status is `singular` with a null compliance, and no displacement predictions are made.

## Omitted effects

The model is an unloaded, small-deflection material stiffness with ideal frictionless joints. It omits preload and geometric (load-dependent) stiffness, and so says nothing about buckling or loaded stability. It also omits compliance of the ball joints, horns, base and platform structures, backlash, and servo control-loop stiffness beyond the supplied torsional value. Only the home pose is analysed.
