#!/usr/bin/env python3
"""Interactive viewer for the Overboard onewheel checkpoint model.

No controller is wired in yet -- this is the intended checkpoint: you should
see the onewheel topple over under gravity within a couple of seconds.

macOS note: MuJoCo's interactive viewer needs its own run loop on the main
thread on macOS, so it must be launched with `mjpython`, not plain `python`:

    mjpython scripts/view_sim.py

(From the project venv: `source .venv/bin/activate && mjpython scripts/view_sim.py`,
or just call `.venv/bin/mjpython scripts/view_sim.py` directly.)
"""

from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(__file__).resolve().parent.parent / "sim" / "models" / "overboard_onewheel.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    print(f"Loaded {MODEL_PATH}")
    print("No controller is active -- watch it fall over. Close the viewer window to exit.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
