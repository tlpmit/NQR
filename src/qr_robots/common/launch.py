"""Helpers to launch a simulation server as a subprocess and connect to it."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time

from qr_robots.common.zmq_sim import SimClient


def start_sim_server(
    module: str,
    objects: dict | None = None,
    model_dir: str = ".",
    mode: str = "headless",
    port: int = 5555,
    use_mjpython: bool = False,
    extra_args: list[str] | None = None,
) -> subprocess.Popen:
    """
    Launch ``python -m <module>`` as a sim server subprocess and wait until it
    prints "serving on port".  Returns the Popen object.

    use_mjpython: launch via mjpython instead of python — required for MuJoCo
    display mode on macOS.
    """
    if use_mjpython and sys.platform == "darwin":
        interpreter = os.path.join(os.path.dirname(sys.executable), "mjpython")
    else:
        interpreter = sys.executable

    cmd = [
        interpreter,
        "-m", module,
        "--model-dir", str(model_dir),
        "--objects", json.dumps(objects if objects is not None else {}),
        "--mode", mode,
        "--port", str(port),
    ] + (extra_args or [])

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    for line in proc.stdout:
        print(f"  [sim_server] {line}", end="")
        if "serving on port" in line:
            break
    else:
        raise RuntimeError(
            f"Sim server '{module}' exited before serving (see log above)."
        )
    return proc


def connect_client(port: int, timeout_ms: int = 120_000) -> SimClient:
    time.sleep(0.2)
    return SimClient(port=port, timeout_ms=timeout_ms)


def shutdown_server(client: SimClient | None, server: subprocess.Popen | None) -> None:
    """Stop the sim server and drain its remaining output."""
    if client is not None:
        try:
            client.stop()
        except Exception:
            pass
        client.close()
    if server is not None:
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server.terminate()
        if server.stdout:
            for line in server.stdout:
                print(f"  [sim_server] {line}", end="")
