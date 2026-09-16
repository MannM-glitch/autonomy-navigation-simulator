"""Generate the browser's replay and report directly from MuJoCo experiments."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image
from .model import make_model, LEGS
from .run import experiment, PUSHES

ROOT = Path(__file__).resolve().parents[1]


def arrow(scene, start, end, color, width=.008):
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array(color, dtype=np.float32))
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, width, start, end)
    scene.ngeom += 1


def render(result, output):
    config = result["config"]
    model, data = make_model(config["payloadKg"], config["friction"])
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .25]
    camera.distance, camera.azimuth, camera.elevation = 1.7, 135, -22
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        with imageio.get_writer(str(output), fps=25, codec="libx264", quality=8, macro_block_size=16, ffmpeg_log_level="error") as writer:
            for index, frame in enumerate(result["frames"][::2]):
                data.qpos[:] = frame["qpos"]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera)
                for leg, load in zip(LEGS, frame["loads"]):
                    start = data.site_xpos[model.site(f"{leg}_toe").id].copy()
                    arrow(renderer.scene, start, start + [0, 0, max(.008, load*.002)], [.32,.94,.74,1])
                force = np.array(frame["push"])
                if np.linalg.norm(force) > 0:
                    end = data.xpos[model.body("trunk").id].copy() + [0,0,.10]
                    arrow(renderer.scene, end-force*.004, end, [1,.35,.22,1], .02)
                pixels = renderer.render()
                writer.append_data(pixels)
                if index == 115 and result["controller"] == "whole_body":
                    Image.fromarray(pixels).save(output.with_name("poster.jpg"), quality=92)
    print(f"Rendered {output}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-video", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "public" / "sentry")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    runs = {}
    for mode in ("whole_body", "joint_pd"):
        print(f"Running {mode}...", flush=True)
        result = experiment(mode)
        print(json.dumps(result["metrics"], indent=2), flush=True)
        if not args.skip_video:
            render(result, args.output / f"{mode}.mp4")
        # Keep generalized coordinates: native rendering can be reproduced from this file.
        runs[mode] = result
    digest = hashlib.sha256()
    for name in ("model.py", "control.py", "run.py", "export.py"):
        digest.update((ROOT/"simulation"/name).read_bytes())
    report = {"schemaVersion": 1, "engine": f"MuJoCo {mujoco.__version__}",
              "python": platform.python_version(), "sourceSha256": digest.hexdigest(),
              "pushes": [{"start": a,"end": b,"force": f} for a,b,f in PUSHES], "runs": runs}
    (args.output / "experiment.json").write_text(json.dumps(report, separators=(",", ":")), encoding="utf-8")
    print("Experiment and telemetry exported.")


if __name__ == "__main__":
    main()
