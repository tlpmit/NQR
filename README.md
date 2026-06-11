# NQR

New implementation of the QR robot planning/simulation/control stack.

## Setup

NQR depends on **sibling checkouts** of HPN and QAA (editable pixi path
dependencies, used by the planning/policy stack). Clone all three side by
side first — `pixi install` fails with
`error while canonicalization ../HPN` otherwise:

```text
GitHub/
├── NQR/    (this repo)
├── HPN/
└── QAA/
```

Then:

```bash
pixi install   # solves the env and builds the qr_mapping C++ module
pixi run test
```

Notes:
- Supported platforms: macOS arm64 and Linux x86-64 (Ubuntu jammy/noble).
- The first install downloads the Drake C++ SDK (~600 MB, cached under
  `.pixi/fetchcontent`) and compiles `qr_mapping`; expect several minutes.
- `src/qr_belief/`, `src/qr_perception/`, `src/qr_policy/`, `src/qr_main/`,
  `src/qr_run/` host the belief/perception/policy stack; planning itself
  (RetroPlan, TAMP domains, Roboverse) is imported from the HPN checkout.

## What's here

- `src/qr_api/` — interfaces and typing for the pipeline components.
- `src/qr_robots/` — virtual robots (RBY1, Spot) backed by MuJoCo and Drake
  simulation servers over a shared ZMQ protocol.
- `src/qr_kinsim/` — kinematics-only simulator (Pinocchio + Coal + Meshcat)
  with object attachment for grasp/push modeling.
- `assets/` — robot models and test scenes.

## Demos

```bash
# RBY1 pick-and-carry in the kinematic sim (open the printed Meshcat URL)
pixi run kinsim-demo

# A MuJoCo sim server with an interactive viewer
pixi run python -m qr_robots.mujoco.rby1.sim --mode display \
    --model-dir assets/test_scene \
    --objects '{"table": {"file": "table.xml", "fixed": true}, "block": "block.xml"}'
```
