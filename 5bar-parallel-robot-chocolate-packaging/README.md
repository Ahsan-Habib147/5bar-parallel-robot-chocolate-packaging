# 5-Bar Planar Parallel Robot — Automated Chocolate Tray Packaging

MATLAB kinematic simulation of a 2-DOF, 5-bar planar parallel robot performing pick-and-place
packaging of 100 chocolates into a 10×10 tray grid. Developed for **MTE 4205 — Robotics**,
Department of Mechatronics Engineering, Rajshahi University of Engineering & Technology (RUET).

## Overview

The robot has two independently driven revolute joints (`q1`, `q2`) mounted on a fixed base, each
powering a two-link open chain. The chains meet at a common end-effector point `P`, forming a
closed kinematic loop. A vacuum suction-cup end-effector with a pneumatic Z-axis actuator picks
each chocolate from a fixed conveyor point and places it into a dimple on the tray.

- **Type:** 5-bar planar parallel robot (symmetric, 2-DOF)
- **Active joints:** q1 (left motor at O1), q2 (right motor at O2)
- **Passive joints:** A (L1–L2 junction), B (L3–L4 junction)
- **End-effector:** P (shared tip of L2 and L4)
- **Link lengths:** L1 = L2 = L3 = L4 = 20 cm
- **Base joints:** O1 = [−8, 10] cm, O2 = [−2, 10] cm
- **Tray:** 10×10 dimple grid, x ∈ [8, 28] cm, y ∈ [0, 20] cm
- **Pick-up point:** [4, 10] cm (conveyor centre)

Full derivations, CAD model, end-effector design, and simulation results are in
[`docs/report.pdf`](docs/report.pdf).

## Repository Structure

```
.
├── docs/
│   └── report.pdf              # Full assignment report (design, kinematics, results)
├── src/
│   ├── master_5bar.m           # Main script: runs the full 100-chocolate pick-and-place cycle
│   ├── ForwKin_5bar.m          # Forward kinematics: q -> [P, A, B]
│   ├── InvKin_5bar.m           # Inverse kinematics: P -> q
│   └── draw_5bar.m             # Visualisation of a single robot configuration
├── simulation/
│   └── simulation.mp4          # Recorded simulation video (full pick-and-place cycle)
└── README.md
```

## Requirements

- MATLAB (tested with base MATLAB, no additional toolboxes required)

## Usage

```matlab
cd src
master_5bar
```

This runs all 100 pick-and-place cycles (conveyor → tray dimple → conveyor), animating the robot
configuration at each step, then plots:

1. Active joint angles `q1`, `q2` (degrees) over the full trajectory
2. End-effector Cartesian trajectory `X`, `Y` (cm) over the full trajectory

To compute kinematics directly:

```matlab
% Forward kinematics
[P, A, B] = ForwKin_5bar(L1, L2, L3, L4, O1, O2, q);

% Inverse kinematics
q = InvKin_5bar(L1, L2, L3, L4, O1, O2, P);
```

## Simulation Video

A recorded video of the full 100-chocolate pick-and-place cycle is available at
[`simulation/simulation.mp4`](simulation/simulation.mp4). It was generated with
[`simulation/master_5bar_record.m`](simulation/master_5bar_record.m), which runs the same
kinematic loop as `src/master_5bar.m` while capturing each animation frame to video.

## Results Summary

| Metric | Value |
|---|---|
| Chocolates placed | 100 (10×10 grid) |
| Simulation steps | 1800 |
| q1 range (left chain) | ≈ −98° to +5° |
| q2 range (right chain) | ≈ +38° to +100° |
| Out-of-workspace events | None |
| Singularity events | None |
| FK–IK round-trip error | < 0.001 cm |

All 100 tray positions and the conveyor pick-up point were verified to lie within the simultaneous
reachable workspace of both chains, with no singularities encountered throughout the cycle.

## Author

**Md. Ahsan Habib** — ID 2008032
Department of Mechatronics Engineering, RUET


## Contact

For any inquiries or issues, please contact us at [ahsan.2008032.ruet.mte@gmail.com](mailto:ahsan.2008032.ruet.mte@gmail.com).

Thank you for using the 5-Bar Parallel Robot simulation!

## References

1. J.-P. Merlet, *Parallel Robots*, 2nd ed. Springer, 2006.
2. J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed. Pearson Prentice Hall, 2004.
