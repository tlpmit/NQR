"""
Kinematic simulator demo: RBY1 in a table-and-block scene, displayed in
Meshcat.  Run with:

    pixi run kinsim-demo

then open the printed Meshcat URL in a browser.
"""

from __future__ import annotations

import time

import numpy as np

from qr_kinsim.sim import KinematicSim


def main():
    sim = KinematicSim(
        "rby1",
        objects={
            "table": {"shape": {"box": [1.2, 0.8, 0.04]}, "pos": [0.9, 0, 0.73],
                      "fixed": True, "color": [0.6, 0.45, 0.3, 1.0]},
            "block": {"shape": {"box": [0.08, 0.08, 0.08]}, "pos": [0.65, 0, 0.79],
                      "color": [0.9, 0.2, 0.1, 1.0]},
        },
        mode="display",
    )
    print(f"Meshcat at {sim.world.meshcat_url}")
    print("Running demo loop (Ctrl-C to stop) …")

    while True:
        # Reach toward the block.
        sim.execute_chain_trajectory("torso", [[0, 0.6, -1.2, 0.6, 0, 0]],
                                     waypoint_dt=1.0)
        sim.execute_chain_trajectory(
            "right_arm", [[-0.3, -0.6, 0, -1.2, 0, 0.4, 0]], waypoint_dt=1.0
        )
        print("collisions:", sim.check_collisions())

        # Grasp (attach), drive away, release.
        sim.set_object_pose("block", sim.get_frame_pose("ee_right")[:3, 3])
        sim.execute_gripper_command("right", "close")
        print("attached:", sim.world.attached_objects())
        sim.execute_base_trajectory([[0.4, 0.5, 0.8]], waypoint_dt=2.0)
        sim.execute_gripper_command("right", "open")

        # Reset.
        sim.execute_gripper_command("right", "close")
        sim.execute_base_trajectory([[0, 0, 0]], waypoint_dt=2.0)
        sim.execute_chain_trajectory("right_arm", [np.zeros(7)], waypoint_dt=1.0)
        sim.execute_chain_trajectory("torso", [np.zeros(6)], waypoint_dt=1.0)
        sim.set_object_pose("block", [0.65, 0, 0.79])
        sim.execute_gripper_command("right", "open")
        time.sleep(1.0)


if __name__ == "__main__":
    main()
