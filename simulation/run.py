"""Reproduce a stance experiment, export telemetry, or open the native viewer."""
import argparse
import json
import time
from pathlib import Path
import mujoco
import numpy as np
from .model import make_model, LEGS
from .control import StanceController, reference

PUSHES = ((5.8, 6.05, (0., 65., 0.)), (10.5, 10.75, (-55., 0., 0.)))


def push_at(t, scale=1.):
    for start, end, force in PUSHES:
        if start <= t < end:
            return np.array(force)*scale
    return np.zeros(3)


def measure(model, data, controller, push):
    R = data.xmat[controller.trunk].reshape(3, 3)
    roll = np.arctan2(R[2,1], R[2,2])
    pitch = np.arcsin(np.clip(-R[2,0], -1, 1))
    loads = np.zeros(4)
    contact_force = np.zeros(6)
    foot_ids = [model.geom(f"{leg}_foot").id for leg in LEGS]
    for i in range(data.ncon):
        contact = data.contact[i]
        for foot, geom in enumerate(foot_ids):
            if geom in (contact.geom1, contact.geom2):
                mujoco.mj_contactForce(model, data, i, contact_force)
                loads[foot] += max(0., contact_force[0])
    height, _, phase = reference(data.time)
    return {
        "time": round(float(data.time), 4), "phase": phase,
        "height": float(data.qpos[2]), "targetHeight": height,
        "roll": float(np.degrees(roll)), "pitch": float(np.degrees(pitch)),
        "drift": float(np.linalg.norm(data.qpos[:2])),
        "loads": loads.tolist(), "push": push.tolist(),
        "torque": float(np.max(np.abs(data.ctrl))),
        "qpos": data.qpos.tolist(),
    }


def summarize(frames, controller):
    # Exclude the first second of contact settling, include both push transients.
    valid = [f for f in frames if f["time"] >= 1]
    tilt = np.array([np.hypot(f["roll"], f["pitch"]) for f in valid])
    error = np.array([f["height"]-f["targetHeight"] for f in valid])
    recovery = []
    for push_index, (_, end, _) in enumerate(PUSHES):
        deadline = PUSHES[push_index+1][0] if push_index+1 < len(PUSHES) else frames[-1]["time"]
        recovered = None
        for i, f in enumerate(frames):
            if f["time"] < end:
                continue
            if f["time"] + .5 > deadline:
                break
            window = [x for x in frames[i:] if x["time"] <= f["time"] + .5]
            if len(window) < 20:
                continue
            if all(np.hypot(x["roll"], x["pitch"]) < 2 and abs(x["height"]-x["targetHeight"]) < .015 and x["drift"] < .03 for x in window):
                recovered = round(f["time"]-end, 3)
                break
        recovery.append(recovered)
    return {
        "rmsTiltDeg": float(np.sqrt(np.mean(tilt**2))),
        "peakTiltDeg": float(tilt.max()),
        "heightRmseMm": float(1000*np.sqrt(np.mean(error**2))),
        "maxDriftMm": float(1000*max(f["drift"] for f in valid)),
        "minHeightM": float(min(f["height"] for f in frames)),
        "recoverySeconds": recovery,
        "solverFailures": controller.failures,
        "saturationFraction": controller.saturated/max(1,controller.calls),
        "fell": bool(any(f["height"] < .20 or np.hypot(f["roll"],f["pitch"]) > 45 for f in frames)),
    }


def experiment(mode="whole_body", payload=1.5, friction=.8, push_scale=1., duration=14., viewer=False):
    model, data = make_model(payload, friction)
    controller = StanceController(model, data, mode)
    frames = []
    handle = None
    if viewer:
        from mujoco import viewer as mjviewer
        handle = mjviewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
        with handle.lock():
            handle.cam.lookat[:] = [0, 0, .25]
            handle.cam.distance = 1.9
            handle.cam.azimuth = 135
            handle.cam.elevation = -22
    start = time.perf_counter()
    try:
        for step in range(round(duration/model.opt.timestep)):
            if handle and not handle.is_running():
                break
            push = push_at(data.time, push_scale)
            data.xfrc_applied[controller.trunk, :3] = push
            if step % 5 == 0:  # 100 Hz control, 500 Hz physics.
                controller.step(data)
            # Log the evaluated state before advancing it: poses, contacts and time align.
            if step % 10 == 0:
                mujoco.mj_forward(model, data)
                frames.append(measure(model, data, controller, push))
            mujoco.mj_step(model, data)
            if handle and step % 10 == 0:
                handle.sync()
                delay = data.time - (time.perf_counter()-start)
                if delay > 0:
                    time.sleep(delay)
    finally:
        if handle:
            handle.close()
    return {"controller": mode, "config": {"payloadKg": payload, "friction": friction, "pushScale": push_scale, "duration": duration, "physicsHz": 500, "controlHz": 100},
            "metrics": summarize(frames, controller), "frames": frames}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=["whole_body", "joint_pd"], default="whole_body")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--payload", type=float, default=1.5)
    parser.add_argument("--friction", type=float, default=.8)
    parser.add_argument("--push-scale", type=float, default=1.)
    parser.add_argument("--output", type=Path, default=Path("simulation/output/run.json"))
    args = parser.parse_args()
    if not (args.payload > 0 and 0 < args.friction <= 2 and args.push_scale >= 0):
        parser.error("payload must be positive, friction in (0, 2], push-scale nonnegative")
    result = experiment(args.controller, args.payload, args.friction, args.push_scale, viewer=args.viewer)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result["metrics"], indent=2))


if __name__ == "__main__":
    main()
