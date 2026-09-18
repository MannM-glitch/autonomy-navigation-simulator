"""Export the browser's MJCF and planner map from the Python reference arena."""
import json
from pathlib import Path

import mujoco

from .navigation import ARENA, ROBOT_CLEARANCE, OBSTACLES, make_arena


def main():
    output = Path(__file__).resolve().parents[1] / "public" / "sentry"
    output.mkdir(parents=True, exist_ok=True)
    model, _ = make_arena()
    mujoco.mj_saveLastXML(str(output / "arena.xml"), model)
    (output / "arena.json").write_text(json.dumps({
        "arena": ARENA, "clearance": ROBOT_CLEARANCE, "obstacles": OBSTACLES,
    }, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
