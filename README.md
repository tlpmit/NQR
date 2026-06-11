# NQR

New implementation of the QR robot planning/simulation/control stack.

## Setup

```bash
pixi install
pixi run test
```

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
