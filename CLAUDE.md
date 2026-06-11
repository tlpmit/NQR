# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Context
- New implementation of robot planning/simulation/control stack. 
- Reference the OLD repo at `../QR` for existing APIs and some implementations to preserve compatibility with.
- There will also be relevant material in `../HPN/HPN/Roboverse` and `../lisdf`.
- In these old repos, there are some unused files and possibly conflicting versions of code, so don't take everything at face value.
- **HPN strategy (decided 2026-06-10):** HPN is NOT ported/vendored. It is an external editable dependency (sibling checkouts `../HPN` and `../QAA`, wired in pixi pypi-dependencies). NQR provides plug-compatible interfaces (belief view, virtual robots, observations); HPN provides the planner (RetroPlan), TAMP domains, and Roboverse world model. Import HPN modules (`Domains.*`, `RetroPlan.*`, `Roboverse.*`, `CogMan.*`, `qaa.*`) rather than porting their files. Later: upgrade HPN as necessary, then decide on a mono-repo.

## Current state of this repo
Implemented so far (all tested, `pixi run test` runs the suite in ~10 s):
- `src/qr_api/` — API/typing layer (copied from old repo), with its support deps ported: `src/qr_core/` (QRModule/events) and a minimal `src/qr_utils/` (asset paths, colors, pcd_utilities, traceFile, timer).
- `src/qr_robots/` — Drake and MuJoCo virtual robots + sim servers for RBY1 and Spot (see "Sim architecture" below).
- `src/qr_kinsim/` — the kinematics-only simulator (Pinocchio + Coal + Meshcat) replacing Roboverse, with object attachment for grasp/push modeling.
- `src/qr_perception/` — perception pipeline ported from the old repo (PercPipelineUncertainSegComplete: label-image segmentation via SimSegmenter, ProjectionCompletion, SurfaceFinder plane detection, RGB featurizers). `pipelines.py` has the standard constructions (`make_sim_perception_pipeline`, `make_sim_obm`) mirroring old `qr_run.run.construct_belief_module`.
- `src/qr_belief/` — full belief port: OBM (`obm/`), goal/robot/dummy memory modules, and `spatial/simple_spatial_module.py` (ternary voxel occupancy from depth images via qr_mapping; HPN's VV_Regions and the Roboverse costmap are deferred to the policy port). `builder.make_sim_belief_module()` wires a complete `BeliefModule`; drive it with `belief.update(Action(), None, {"images": [...], "conf": {...}})`.
- `src/qr_mapping/` — the C++ occupancy-grid (pybind11 module `qr_mapping.cpp`), built by scikit-build-core during `pixi install`: the root `CMakeLists.txt` fetches the Drake C++ SDK (pinned 1.45.0, cached under `.pixi/fetchcontent`) and builds with the vendored common_robotics_utilities / voxelized_geometry_tools. Two hard version pins, do not bump independently: pip `drake==1.45.0` must match the SDK (Drake objects cross the pybind boundary), and conda `pybind11 <2.14` must match pydrake's pybind11 internals ABI (3.x cannot share types like Rgba/Meshcat).
- `assets/` — robot models copied from the old repo: `rby1a/` (MJCF + URDF), `rby1_description_drake/`, `spot/` (Drake URDF + scenario YAMLs), `spot_description_drake/` (meshes), `test_scene/` (small table+block scenes per backend), `package.xml` (ROS package name `qr_assets` used by `package://qr_assets/...` URIs).
- `src/qr_policy/` — the HPN policy adapter (`hpn_policy_module.py`); `src/qr_main/` (perceive-plan-act loop), `src/qr_run/` (assembly: `run.run_QR_main(problem, config)`, `virtual_robots.py` QDDL→scene bridge), `src/qr_problem_defs/` + `src/qr_domains/` (problems and QDDL files). Run a problem: build a script like `run_QR_main(ruby_1_move_base, QRSystemConfig(segmentation_method="sim", hpn_params=HPNConfig(debug_level=1, interactive=False)))` — `interactive=False` is required headless (HPN's debug tags pause on input() otherwise). HPN planning logs land in `~/QR_logs`/`~/HPN_logs`; the old repo's pkl recordings there can be replayed via `QRSystemConfig(run_from_pkl_path=...)`.
- KNOWN ISSUE (2026-06-10): on `ruby_1_move_base`, the first plan from an empty belief fails (`plan_fail: monotonic`, no look_region generated). This is NOT an NQR bug: the unmodified old repo running live with its own Roboverse sim against today's HPN fails identically at the same point, while recorded logs from 2026-05-31 (in `~/QR_logs`) show it working then — the regression is in the HPN/QR repos' state since. Belongs to the "upgrade HPN" phase.
- Not yet started: `qr_planning` (OMPL2 motion planning).

## Commands
- `pixi install` — set up the whole environment (conda-forge: python, pinocchio; pypi: drake, mujoco, manipulation, meshcat, …). REQUIRES sibling checkouts `../HPN` and `../QAA` (editable path deps); without them it fails with "error while canonicalization ../HPN".
- `pixi run test` — run the pytest suite (`tests/`). A single test: `pixi run pytest tests/test_kinsim.py::test_grasp_attach_detach -q`.
- `QR_TEST_MODE=display` runs the test sims with their viewers (Meshcat for Drake/kinsim — run with `-s` to see the URL; a MuJoCo window otherwise). In-process MuJoCo tests on macOS must then be run via `mjpython -m pytest`; the ZMQ virtual-robot tests handle this automatically.
- `QR_TEST_MODE=interactive` is display mode plus prompts: it waits for Enter once the viewer is up before executing, and again before closing. Requires `-s`, e.g. `QR_TEST_MODE=interactive pixi run mjpython -m pytest tests/test_mujoco_rby1.py -s`.
- `pixi run kinsim-demo` — RBY1 pick-and-carry demo in Meshcat (kinematic sim, display mode).
- Sim servers can run standalone, e.g. `pixi run python -m qr_robots.mujoco.rby1.sim --objects '{...}' --mode display` (MuJoCo display mode on macOS needs `mjpython`; the launcher handles this automatically).

## Environment gotchas
- The user's shell profile sets `PYTHONPATH` to the OLD repos (QR, HPN, …). The pixi env clears it via `[tool.pixi.activation.env]` in `pyproject.toml` — always run code through `pixi run`, never bare `python`, or you'll silently import old-repo packages.
- `~/Library/Caches/rattler/.../pypi/mujoco` is root-owned (from a past sudo run), which breaks `pixi install`'s pypi solve. Workaround: `PIXI_CACHE_DIR=$HOME/.cache/pixi-cache pixi install`. Permanent fix needs `sudo chown -R $(whoami) ~/Library/Caches/rattler`.

## Sim architecture
All simulation backends speak one pickle-over-ZMQ REQ/REP protocol (`qr_robots/common/zmq_sim.py`: `serve()` + `SimClient`), so one set of sensor/control modules (`zmq_robot_modules.py`) and one generic virtual robot (`zmq_virtual_moman.py`) serve every robot × backend combination. Per-robot conf-vector layouts live in `qr_robots/common/rby1.py` (canonical 31-vector) and `spot.py` (10-vector); they are identical across backends.
- MuJoCo: generic `qr_robots/mujoco/sim_base.py` (`MujocoRobotSim` + `MujocoRobotSpec`); RBY1 uses the MJCF model with a mocap-welded holonomic base, Spot is built by rewriting the Drake URDF at load time (MjSpec) and injecting position actuators. The injected compiler block must keep `fusestatic="false"`: fusing the static `base_z` body silently drops its 0.52 m elevation (robot sunk into the floor) and fused URDF specs crash `MjSpec.attach` with SIGBUS. MuJoCo's OBJ loader also keeps only the first material group of a file (body.obj loses ~6.5k faces → holes) and ignores .mtl colors, so the rewrite splits each visual OBJ by `usemtl` into per-material sub-meshes (cached in `assets/spot_description_drake/mujoco_mesh_cache/`, regenerated when stale) and expands the URDF `<visual>` elements with the .mtl diffuse colors.
- Drake: generic `qr_robots/drake/sim_base.py` (`DrakeRobotSim` + spec) built on `manipulation.MakeHardwareStation` and the scenario YAMLs in `assets/`; a parameterized `DesiredStateSource` feeds the per-group `desired_state` ports.
- Kinematic: `qr_kinsim/world.py` (`KinematicWorld`: pin model + coal collision + meshcat display, object attach/detach), `robots.py` (tree-of-chains specs; URDF preprocessing — the rby1a URDF needs capsule→cylinder and wheel-limit fixes, the Spot URDF needs package:// resolution and drake-tag stripping), `sim.py` (protocol adapter; gripper close/open = grasp attach/detach). Cameras come from `render.py`: a headless render-only MuJoCo scene (same robot model/cameras as the MuJoCo backend, MJCF bodies generated from the shape specs) synced to the kinematic state before each image and never stepped — so kinsim image/intrinsics/extrinsics/label formats match the dynamics backends exactly.
- Conventions: camera extrinsics are 4×4 world→camera in OpenCV convention (+Z forward) and come from the sim server (`get_camera_extrinsics`); quaternions are (w, x, y, z); `get_world_state()` returns `{obj: [x,y,z,qw,qx,qy,qz], "robot": conf}`. Label images: positive ids only for FREE scene objects; robot, fixed objects, and background are -1 (fixtures/surfaces are detected geometrically by SurfaceFinder, not via labels). This holds across MuJoCo, Drake (which natively labels every body incl. the robot — remapped in `get_camera_image`), and the kinsim render bridge.
- Perception/OBM port notes: `qr_utils/pcd_utilities.verts_from_planes` uses scipy HalfspaceIntersection instead of pycddlib (no wheels; cdd is used if installed), and the GJK quick-reject from Roboverse became a conservative AABB test. `Sensor.trace_image_raw` (OBM visibility reasoning) needs `ncollpyde`. The VLM goal-driven perception has a `NullDesignationPerceptionFunction` stand-in for sim tests.
- Scene objects dict: MuJoCo takes MJCF files, Drake takes URDF/SDF files, kinsim takes primitive `shape` specs (see each `sim.py` docstring).

## Packaging

- We want to be able to install the whole pipeline by simply doing "pixi install"
- We want to support Mac OS and Linux.
- On Mac OS use MPS for GPU interface.  Functionality that might require Linux/Cuda specific environments, will be accessed via ZMQ services.

## Key APIs to preserve from old repo
- `../QR/src/qr_api/` has api and typing files for many of the key components of the pipeline.  Try to preserve compatibility but it is possible to make changes to improve the design.  A copy of `qr_api` has been copied to this repo `NQR/src/qr_api`.
- `../QR/src/qr_robots/drake/rby1/rby1_drake_virtual_robot.py` and `../QR/src/qr_robots/mujoco/rby1/rby1_mujoco_virtual_robot.py` implement the API that we want to have for all robots (Spot, Franka, etc).  The implementation of `rby1_drake_sim.py` and `rby1_mujoco_sim` should be pretty close.

## The qr_api layer
`src/qr_api/` is organized as paired `*_interfaces.py` (abstract classes) and `*_typing.py` (data types) files per module. Key abstractions:
- `virtual_robot_interfaces.VirtualRobotModule`: base class for all robot interfaces. A robot is a named collection of `SensorModuleBase`s (each with a `sense()` method, aggregated into an `Observation`) and `ControlModuleBase`s. Control is either synchronous (`control_blocking`) or asynchronous via a set_control/step_control protocol where commands for multiple modules are staged into a `VirtualRobotControlContext` and then stepped together (e.g., one simulator step, or one whole-body command to the real robot).
- `virtual_moman_interfaces.VirtualMomanModule`: the mobile-manipulator API on top of `VirtualRobotModule` — cameras, robot conf/base pose, and sync/async trajectory following per chain (joint, cartesian, end-effector, gripper, base, head, torso).
- `belief_interfaces.BeliefModule` plus memory modules (`SpatialMemoryModule`, `ObjectMemoryModule`, `RobotMemoryModule`, `AgentMemoryModule`, `GoalMemoryModule`) with a hook mechanism for updates.
- `planning_interfaces.BeliefViewForPlanner`: the planner's read-only view of the belief — object pose/attribute/holding probabilities and path legality checks (`phys_legal_path`, `get_phys_path_viols`).
- Pipeline data flow: sensors produce `Observation`s → perception (`perc_*`) segments/completes/featurizes → OBM (`obm_*`) maintains object set → belief is updated → policy (`policy_*`) picks actions → control commands go back to the virtual robot.

## Architecture
The top-level architecture is made up of the following key modules (as present in the old repo):
- Perception: Pipeline that takes RGBD images, does segmentation, completion, and featurization and returns Observations.
- Object-Based Memory (OBM): Takes Observations and maintains a set of objects
- Belief: `../QR/src/qr_belief/` Updates belief over the state of the world
- Policy: `../QR/src/qr_policy/` Chooses actions based on the belief
- Mapping: `../QR/src/qr_mapping/` a C++ implementation of Occupancy Grid that is crucial for Belief
Additional modules:
- Robots: `../QR/src/qr_robots` Implementation of the virtual robot interface for several robots and simulators (much of the code may not be currently relevant)
- Main and Run: `../QR/src/qr_main` and `../QR/src/qr_run`. There are examples of "problem" definitions in `../QR/src/qr_problem_defs/misc_problems.py`.
- QDDL file: `../QR/src/qr_examples/` Some QDDL file defining problems
- Odometry: `../QR/src/qr_localization/` A ZMQ-based service for real robot Livox output

## Architecture Issues
- We need a kinematics-only simulator, currently implemented via `../QR/src/qr_robots/roboverse/` built on the HPN repo.  But, we want to re-implement that using Pinocchio + Coal (for kinematics and collision), Meshcat (for display) and OMPL2 (for motion planning).  Note that this simulator will need model attachment of objects to gripper (to model the effect of grasping and also to model the effect of pushing).
- We need to create `qr_planning` to implement collision-free motion planning.
- The robot should be modeled as a tree of chains (base, torso, left arm, right arm, right_gripper, left_gripper, head) as in `../QR/src/qr_robots/drake/rby1/rby1_drake_virtual_robot.py`.  We should plan for the motions of the chains to be in sequence (not in parallel).  Note that the base may be stationary, holonomic or non-holonomic.

## What NOT to carry forward from old repo
- `qr_fault/`, `qr_hardware`, `qr_simulation`
