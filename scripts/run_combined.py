"""
Run the combined NQR + HPN system on a QR problem, with displays.

    pixi run combined                         # ruby_1_move_base, with displays
    pixi run python scripts/run_combined.py --problem ruby_1_move_base --headless
    pixi run python scripts/run_combined.py --replay ~/QR_logs/log_2026-05-31_14_42_59_288586

Displays:
  - Kinematic sim (default): robot + scene in Meshcat (URL printed at startup,
    usually http://localhost:7000); trajectories animate at 50 Hz.
    With --virtual-robot Ruby_Mujoco_Sim a MuJoCo viewer window opens instead.
  - Belief Meshcat viewers (open in a browser):
      http://localhost:7777  voxel grid (occupied/unknown)
      http://localhost:7778  aux occupancy
      http://localhost:7779  aux max-range occupancy

With --interactive, HPN's debug pauses are left enabled: the run stops at
plan failures with an input() prompt so you can inspect state (requires a
terminal; never use it for unattended runs).
"""

import argparse

from qr_run.qr_config import HPNConfig, QRSystemConfig
from qr_run.run import run_QR_main


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problem", default="ruby_1_move_base",
                    help="Problem name from qr_problem_defs.misc_problems.")
    ap.add_argument("--virtual-robot", default="Ruby_Kinsim",
                    help="Virtual robot backend: Ruby_Kinsim (kinematics-only, "
                         "default) or Ruby_Mujoco_Sim (dynamics).")
    ap.add_argument("--headless", action="store_true",
                    help="No MuJoCo viewer window.")
    ap.add_argument("--interactive", action="store_true",
                    help="Keep HPN's input() pauses (e.g. on plan_fail).")
    ap.add_argument("--debug-level", type=int, default=1)
    ap.add_argument("--replay", default=None, metavar="PKL_DIR",
                    help="Replay a recorded run (QR_logs dir) instead of live sim.")
    ap.add_argument("--record", action="store_true",
                    help="Record observations/actions to a new QR_logs dir.")
    args = ap.parse_args()

    import qr_problem_defs.misc_problems as problems

    problem = getattr(problems, args.problem)
    problem.virtual_robot = args.virtual_robot

    cfg = QRSystemConfig(
        segmentation_method="sim",
        virtual_robot_mode="headless" if (args.headless or args.replay) else "display",
        hpn_params=HPNConfig(
            debug_level=args.debug_level,
            interactive=args.interactive,
        ),
        run_from_pkl_path=args.replay,
        write_to_pkl_path="QR" if args.record else None,
    )
    run_QR_main(problem, cfg)
    print("RUN COMPLETED")


if __name__ == "__main__":
    main()
