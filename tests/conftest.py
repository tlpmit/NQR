import os
import pathlib

import pytest

REPO_ROOT = pathlib.Path(__file__).parent.parent
TEST_SCENE = REPO_ROOT / "assets" / "test_scene"

# Set QR_TEST_MODE=display to run the sims with their viewers:
#   - kinsim / Drake tests: a Meshcat URL is printed (run pytest with -s);
#   - in-process MuJoCo tests on macOS need the mjpython interpreter:
#         QR_TEST_MODE=display pixi run mjpython -m pytest tests/test_mujoco_rby1.py -s
#   - the ZMQ virtual-robot tests spawn their servers with mjpython
#     automatically, so plain pytest works for them.
#
# QR_TEST_MODE=interactive is display mode plus prompts: once the viewer is up
# it waits for Enter before executing the tests, and again before closing, so
# you can actually watch.  Requires -s (and mjpython for in-process MuJoCo).
SIM_MODE = os.environ.get("QR_TEST_MODE", "headless")
INTERACTIVE = SIM_MODE == "interactive"
if INTERACTIVE:
    SIM_MODE = "display"


def pytest_configure(config):
    if INTERACTIVE and config.getoption("capture") != "no":
        raise pytest.UsageError(
            "QR_TEST_MODE=interactive needs pytest -s so the start/end "
            "prompts can read from the terminal."
        )


@pytest.hookimpl(wrapper=True)
def pytest_runtest_call(item):
    # In interactive mode, ask before each test executes.  This hook runs
    # after all fixture setup, so the viewer is already up.
    if INTERACTIVE:
        interactive_pause(f"\n▶ {item.nodeid} — press Enter to run … ")
    return (yield)


def interactive_pause(message: str) -> None:
    if not INTERACTIVE:
        return
    try:
        input(message)
    except EOFError:   # stdin closed (e.g. piped run) — just continue
        print()


def sim_fixture(sim, name: str, url: str | None = None):
    """Wrap a sim (or virtual robot) fixture body: in interactive mode,
    announce the viewer when it comes up and wait for Enter before tearing it
    down.  The per-test start prompts come from pytest_runtest_call."""
    if INTERACTIVE:
        where = f" at {url}" if url else ""
        print(f"\n[{name}] viewer ready{where}")
    yield sim
    interactive_pause(f"\n[{name}] done — press Enter to close … ")

# MJCF scene (MuJoCo backends)
MJCF_OBJECTS = {
    "table": {"file": "table.xml", "fixed": True},
    "block": {"file": "block.xml"},
}

# URDF scene (Drake backends); poses match the MJCF scene
URDF_OBJECTS = {
    "table": {"file": "table.urdf", "fixed": True, "pos": [0.9, 0, 0]},
    "block": {"file": "block.urdf", "pos": [0.65, 0, 0.81]},
}

# Primitive-shape scene (kinematic backend); colors match the MJCF/URDF scenes
KINSIM_OBJECTS = {
    "table": {"shape": {"box": [1.2, 0.8, 0.04]}, "pos": [0.9, 0, 0.73],
              "fixed": True, "color": [0.6, 0.45, 0.3, 1.0]},
    "block": {"shape": {"box": [0.08, 0.08, 0.08]}, "pos": [0.65, 0, 0.79],
              "color": [0.9, 0.2, 0.1, 1.0]},
}


@pytest.fixture(scope="session")
def test_scene_dir() -> str:
    return str(TEST_SCENE)
