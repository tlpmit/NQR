"""
Run the combined NQR + HPN system on a QR problem, with displays.

    pixi run combined                         # ruby_pick_place_move_base
    pixi run python scripts/run_combined.py --problem ruby_pick_place_move_base --headless
    pixi run python scripts/run_combined.py --replay <QR_logs dir>

Uses the standard HPN overrides and settings from the old repo's
main_tlp.py (see qr_run.standard_configs) — behavior differs wildly
without them.

Displays:
  - Kinematic sim (default): robot + scene in Meshcat (URL printed at startup,
    usually http://localhost:7000); trajectories animate at 50 Hz.
    With --virtual-robot Ruby_Mujoco_Sim a MuJoCo viewer window opens instead.
  - Belief Meshcat viewers (open in a browser):
      http://localhost:7777  voxel grid (occupied/unknown)
      http://localhost:7778  aux occupancy
      http://localhost:7779  aux max-range occupancy

With --interactive (terminal runs only), HPN's input() pauses stay enabled
(plan_fail at debug pauses, execution_fail); never use it unattended.
"""

# ruff: noqa: E402
import os

# Threading / MPS environment, as in main_tlp.py — set before heavy imports.
os.environ["PYTORCH_ENABLE_MPS_FALLBACK"] = "1"
n_threads = "1"
os.environ["OMP_NUM_THREADS"] = n_threads
os.environ["MKL_NUM_THREADS"] = n_threads
os.environ["OPENBLAS_NUM_THREADS"] = n_threads
os.environ["VECLIB_MAXIMUM_THREADS"] = n_threads
os.environ["NUMEXPR_NUM_THREADS"] = n_threads

import argparse


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problem", default="ruby_pick_place_move_base",
                    help="Problem name from qr_problem_defs.misc_problems.")
    ap.add_argument("--virtual-robot", default="Ruby_Kinsim",
                    help="Virtual robot backend: Ruby_Kinsim (kinematics-only, "
                         "default) or Ruby_Mujoco_Sim (dynamics).")
    ap.add_argument("--headless", action="store_true",
                    help="No simulator display.")
    ap.add_argument("--interactive", action="store_true",
                    help="Keep HPN's input() pauses (plan/execution failures).")
    ap.add_argument("--debug-level", type=int, default=1)
    ap.add_argument("--replay", default=None, metavar="PKL_DIR",
                    help="Replay a recorded run (QR_logs dir) instead of live sim.")
    ap.add_argument("--record", action="store_true",
                    help="Record observations/actions to a new QR_logs dir.")
    args = ap.parse_args()

    import qr_problem_defs.misc_problems as problems
    from qr_run.run import run_QR_main
    from qr_run.standard_configs import adjust_config_for_robot, make_sim_config

    problem = getattr(problems, args.problem)
    problem.virtual_robot = args.virtual_robot

    cfg = make_sim_config(
        interactive=args.interactive,
        debug_level=args.debug_level,
        write_to_pkl_path="QR" if args.record else None,
        run_from_pkl_path=args.replay,
    )
    cfg.virtual_robot_mode = (
        "headless" if (args.headless or args.replay) else "display"
    )
    cfg = adjust_config_for_robot(problem, cfg)

    run_QR_main(problem, cfg)
    print("RUN COMPLETED")


if __name__ == "__main__":
    # Avoid loading expensive packages more than once under multiprocessing
    # (as in main_tlp.py).
    try:
        from multiprocessing import set_start_method

        import open3d  # noqa: F401, I001
        import qr_mapping.cpp  # noqa: F401

        # FORK can be dangerous in the presence of threads;
        # SPAWN is safer but slow since it re-imports everything.
        set_start_method("spawn")
    except Exception:
        # set_start_method fails if already set.
        pass

    main()
