> Historical simulator requirements. This is not a description of the current root optimizer. See the [current README](../README.md) and [archived simulator instructions](Stuff%20from%20Old%20Project/README.md).

# Stewart Platform Simulator — Consolidated Specification for CODEX

## 1. Goal
Create a simulator that:
- Visualizes and simulates Stewart platforms with customizable geometry.
- Generalizes inverse kinematics math to arbitrary base/platform anchors and servo orientations.
- Provides sliders and input boxes for *all* mechanical parameters (no artificial limits).
- Supports optimization functions (workspace, torque, stiffness, dexterity).

---

## 2. Generalization of Math
The old simulator hard-coded circular/hexagonal anchor placements. We need to allow:

- **Base Anchors (B)**: array of 6 [x,y,z] coordinates in mm.
- **Platform Anchors (P)**: array of 6 [x,y,z] coordinates in mm.
- **Servo Axes (β)**: 6 orientation angles (radians).
- **Horn Length (h)** and **Rod Length (d)** as parameters.

### Inverse Kinematics Formula
For each leg k:
- l_k = (translation + rotated platform anchor) - base anchor
- e_k = 2 h l_kz
- f_k = 2 h (cos(β_k) l_kx + sin(β_k) l_ky)
- g_k = ||l_k||² - (d² - h²)
- Servo angle:
  α_k = arcsin(g_k / sqrt(e_k² + f_k²)) - atan2(f_k, e_k)

This is general for any layout.

---

## 3. Simulation Functions
- Workspace sweeps: iterate over X, Y, Z, Rx, Ry, Rz ranges.
- Workspace coverage metrics: % of reachable poses vs. target.
- Performance simulation: payload mass, stroke, frequency → torque & RPM.
 - Ball-joint angle limits (toggleable), servo range limits, rod collision checks.

---

## 4. UI Requirements
- **Mechanical Section**: horn length, rod length, anchor coordinates, servo axes, servo range.
- **Simulation Section**: payload, stroke, frequency, workspace sweeps.
- **Export Section**: export/import JSON layouts and results.

No limits: sliders should not enforce min/max.

---

## 5. Rendering
- Draw anchors, horns, rods, outlines.
- Update live on parameter change.
- Highlight rods in red when servo limits exceeded.

---

## 6. JSON Schema
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "StewartPlatformLayout",
  "type": "object",
  "properties": {
    "name": {
      "type": "string",
      "description": "Optional name/label for the layout"
    },
    "base_anchors": {
      "type": "array",
      "description": "Array of 6 base anchor points [x,y,z] in mm",
      "items": {
        "type": "array",
        "items": {
          "type": "number"
        },
        "minItems": 3,
        "maxItems": 3
      },
      "minItems": 6,
      "maxItems": 6
    },
    "platform_anchors": {
      "type": "array",
      "description": "Array of 6 platform anchor points [x,y,z] in mm",
      "items": {
        "type": "array",
        "items": {
          "type": "number"
        },
        "minItems": 3,
        "maxItems": 3
      },
      "minItems": 6,
      "maxItems": 6
    },
    "beta_angles": {
      "type": "array",
      "description": "Array of 6 servo axis orientation angles in radians",
      "items": {
        "type": "number"
      },
      "minItems": 6,
      "maxItems": 6
    },
    "horn_length": {
      "type": "number",
      "description": "Servo horn length in mm"
    },
    "rod_length": {
      "type": "number",
      "description": "Rod length in mm"
    },
    "servo_range": {
      "type": "array",
      "description": "Min and max servo angles in degrees [min, max]",
      "items": {
        "type": "number"
      },
      "minItems": 2,
      "maxItems": 2
    },
    "payload": {
      "type": "object",
      "description": "Payload simulation parameters",
      "properties": {
        "mass_kg": {
          "type": "number"
        },
        "stroke_mm": {
          "type": "number"
        },
        "frequency_hz": {
          "type": "number"
        }
      }
    }
  },
  "required": [
    "base_anchors",
    "platform_anchors",
    "beta_angles",
    "horn_length",
    "rod_length"
  ]
}
```

---

## 7. Example Layouts

### Linear Layout
```json
{
  "name": "Optimized Linear Layout (X-axis travel)",
  "base_anchors": [
    [
      -60,
      -40,
      0
    ],
    [
      60,
      -40,
      0
    ],
    [
      -60,
      0,
      0
    ],
    [
      60,
      0,
      0
    ],
    [
      -60,
      40,
      0
    ],
    [
      60,
      40,
      0
    ]
  ],
  "platform_anchors": [
    [
      -30,
      -40,
      0
    ],
    [
      30,
      -40,
      0
    ],
    [
      -30,
      0,
      0
    ],
    [
      30,
      0,
      0
    ],
    [
      -30,
      40,
      0
    ],
    [
      30,
      40,
      0
    ]
  ],
  "beta_angles": [
    0,
    0,
    0,
    0,
    0,
    0
  ],
  "horn_length": 46.7,
  "rod_length": 155.6,
  "servo_range": [
    -90,
    90
  ],
  "payload": {
    "mass_kg": 6.8,
    "stroke_mm": 203.0,
    "frequency_hz": 4.0
  }
}
```

### Circular Layout
```json
{
  "name": "Circular Layout (Symmetrical Default)",
  "base_anchors": [
    [
      70,
      0,
      0
    ],
    [
      35,
      60,
      0
    ],
    [
      -35,
      60,
      0
    ],
    [
      -70,
      0,
      0
    ],
    [
      -35,
      -60,
      0
    ],
    [
      35,
      -60,
      0
    ]
  ],
  "platform_anchors": [
    [
      50,
      0,
      0
    ],
    [
      25,
      43,
      0
    ],
    [
      -25,
      43,
      0
    ],
    [
      -50,
      0,
      0
    ],
    [
      -25,
      -43,
      0
    ],
    [
      25,
      -43,
      0
    ]
  ],
  "beta_angles": [
    0,
    1.047,
    2.094,
    3.142,
    -2.094,
    -1.047
  ],
  "horn_length": 50.0,
  "rod_length": 120.0,
  "servo_range": [
    -90,
    90
  ],
  "payload": {
    "mass_kg": 0.0,
    "stroke_mm": 0.0,
    "frequency_hz": 0.0
  }
}
```

### Hexagonal Layout
```json
{
  "name": "Hexagonal Layout (RAW-style, platform rotated 180\u00b0)",
  "base_anchors": [
    [
      70,
      0,
      0
    ],
    [
      35,
      60,
      0
    ],
    [
      -35,
      60,
      0
    ],
    [
      -70,
      0,
      0
    ],
    [
      -35,
      -60,
      0
    ],
    [
      35,
      -60,
      0
    ]
  ],
  "platform_anchors": [
    [
      -50,
      0,
      0
    ],
    [
      -25,
      -43,
      0
    ],
    [
      25,
      -43,
      0
    ],
    [
      50,
      0,
      0
    ],
    [
      25,
      43,
      0
    ],
    [
      -25,
      43,
      0
    ]
  ],
  "beta_angles": [
    0,
    1.047,
    2.094,
    3.142,
    -2.094,
    -1.047
  ],
  "horn_length": 50.0,
  "rod_length": 120.0,
  "servo_range": [
    -90,
    90
  ],
  "payload": {
    "mass_kg": 0.0,
    "stroke_mm": 0.0,
    "frequency_hz": 0.0
  }
}
```

---

## 8. References for Optimization

1. Taherifar, A. et al. (2013). *Kinematic Optimization of Stewart Platform for Simulators Using Genetic Algorithm.* IEEE.  
2. Chaudhury, A. (2017). *Kinematic Design and Optimal Synthesis of Parallel Manipulators Using Monte Carlo Method.*  
3. Albitar, C. et al. (2018). *Design Optimization of 6-RUS Parallel Manipulator Using Hybrid Algorithm (GA + PSO).*  
4. Petrašinović, M. D. et al. (2022). *Real-coded Mixed Integer Genetic Algorithm for Geometry Optimization of Rotary Stewart Platforms.* Applied Sciences.  
5. Kelaiaia, R., & Zaatri, A. (2012). *Multiobjective Optimization of Parallel Kinematic Mechanisms by Genetic Algorithms.* Robotica.  
6. Jamwal, P. K. et al. (2009). *Kinematic Design Optimization of a Parallel Ankle Rehabilitation Robot Using Modified GA.*  
7. Hou, Y. et al. (2009). *Optimal Design of a Hyperstatic Stewart Platform-Based Force/Torque Sensor with GA.*  
8. Inner, B., & Kucuk, S. (2013). *A Novel Kinematic Design, Analysis, and Simulation Tool for General Stewart Platforms.*  
9. Monsarrat, B., & Gosselin, C. M. (2004). *Workspace Analysis and Optimal Design of a 3-leg 6-DOF Parallel Platform Mechanism.*  
10. Chapin, S., Hildebrand, R. et al. (2023). *Force-controlled Pose Optimization and Trajectory Planning for Chained Stewart Platforms.*  

---

## 9. Optimization Math & Metrics

- **Dexterity indices**: condition number of Jacobian, isotropy index.
- **Stiffness optimization**: compliance matrix, Castigliano’s theorem.
- **Transmission index**: maps servo torque to platform force.
- **Singularity avoidance**: Jacobian determinant near zero → workspace boundary.
- **Load sharing**: distribute payload across legs with pseudo-inverse Jacobian.

---

## 10. Example Pseudocode (Genetic Algorithm)

```python
# Pseudocode for optimizing rod/horn lengths vs workspace coverage & torque

population = initialize_random_layouts(N)
for generation in range(max_gen):
    fitness_scores = []
    for layout in population:
        workspace = simulate_workspace(layout)
        torque = compute_servo_torque(layout, payload, stroke, freq)
        coverage = workspace.coverage(target)
        # fitness = weighted sum (maximize coverage, minimize torque)
        fitness = coverage - 0.1 * torque
        fitness_scores.append(fitness)
    population = select_and_mutate(population, fitness_scores)

best_layout = select_best(population, fitness_scores)
```

---

