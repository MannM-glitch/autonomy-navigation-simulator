# SENTRY: whole-body stance control

A four-legged inspection platform has to lower its sensor package while keeping it level, withstand a lateral disturbance, and return to its observation height. This project uses **real MuJoCo rigid-body dynamics**, a free-floating base, 12 torque-controlled joints, gravity, collisions, and frictional ground contacts. The original robot is built from primitives; there are no downloaded meshes or pretrained policies.

The robot solves a **stationary sensor-platform stabilization task**. It does not walk, navigate, detect objects, or perform a real inspection. The cylinder represents a rigid sensor payload. This narrow task makes controller behavior measurable and explainable.

## Run it

From the repository root, with Python 3.13:

```powershell
python -m venv .venv
.venv\Scripts\python -m pip install -r simulation/requirements.txt
.venv\Scripts\python -m simulation.run --viewer
```

The viewer opens a 14-second experiment. Rotate/zoom with the mouse; rerun to repeat. On Linux use `.venv/bin/python`. On macOS the native viewer requires the `mjpython` launcher. Headless runs and tests do not require an OpenGL window:

```powershell
.venv\Scripts\python -m simulation.run
.venv\Scripts\python -m simulation.run --controller joint_pd --viewer
.venv\Scripts\python -m simulation.run --payload 4 --friction 0.35 --push-scale 1.5
.venv\Scripts\python -m unittest simulation.test_control -v
.venv\Scripts\python -m simulation.benchmark
```

`simulation/output/run.json` contains telemetry and generalized coordinates. Arguments change the actual dynamics, not a playback animation. To export the model for the standalone MuJoCo viewer, run `python -m simulation.model`. Loading the XML alone does **not** run the controller; use `simulation.run --viewer` for the controlled demo.

## Experiment

| Time | Action |
| --- | --- |
| 0–3 s | Stabilize at a 0.435 m body height |
| 3–5 s | Smoothly lower to 0.340 m |
| 5–8 s | Hold inspection height; apply +65 N along world Y at 5.80–6.05 s |
| 8–10 s | Return to 0.435 m |
| 10–14 s | Hold; apply −55 N along world X at 10.50–10.75 s |

Both pushes act at the trunk center of mass and last 0.25 s. MuJoCo integrates at 500 Hz; the controller updates at 100 Hz. The logs are sampled at 50 Hz. There is no base weld, kinematic body teleporting, artificial stabilization force, or hidden support. External forces are only the documented pushes. Joint torques are capped at ±32 N·m.

## Controller, from error to torque

1. Read the simulated body pose, velocity, center of mass, and foot positions.
2. Convert position and orientation errors into a desired six-dimensional body wrench (force + moment), including gravity compensation.
3. Solve a regularized quadratic contact-force allocation problem. The wrench matrix contains an identity block for forces and `skew(foot − COM)` for moments. The objective is `0.5 ||W(Af − wrench)||² + 0.0005 ||f||²`.
4. Enforce positive normal forces, a 180 N per-foot normal limit, and `|fx| + |fy| ≤ 0.55 fz`. This diamond fits inside the circular friction cone. The unconstrained optimum is used only when it satisfies every bound; otherwise SLSQP solves the constrained problem. A failed solve holds the previous feasible force command and increments a logged counter.
5. Map ground reaction forces to joint torques with `tau = bias − Jᵀf`, then add low-gain posture feedback and joint damping. The minus sign follows `M qdd + bias = tau + Jᵀf`. Clip the final motor commands.

This is a **reduced-order whole-body stance controller**. It coordinates all four legs using a centroidal wrench target. It is not a full inverse-dynamics QP: joint accelerations, actuator limits, and contact complementarity are not jointly optimized. Force allocation assumes four supporting feet; actual contact loss is measured but does not change that assumption.

The baseline uses joint PD alone (`Kp=90`, `Kd=5`) to track the same height-derived posture with the same robot, torque limits, and pushes. The whole-body controller uses posture gains 8 and 1.2 in addition to body feedback and model-based feedforward. This is a comparison with a simple, explicitly defined baseline, not proof of superiority over a tuned controller with gravity compensation.

## Measurements and limits

Metrics exclude the first second of contact settling. Tilt is `sqrt(roll² + pitch²)` in degrees (a small-angle metric, not a geometric tilt angle near a fall). Height RMSE compares trunk Z with the moving height target. Drift is the trunk's planar displacement from its initial origin. Recovery requires tilt <2°, height error <15 mm, and drift <30 mm continuously for 0.5 s, before the next push or the experiment end. Null means no qualifying interval was observed. Times have 20 ms sample resolution.

The five-case sweep checks payload mass, stronger pushes, and lower friction. The controller keeps its assumed coefficient at 0.55 while the ground coefficient changes. The last case deliberately exceeds the controller's useful range: a 4 kg payload, friction 0.25, and 2.5× pushes. **The whole-body controller falls in that case**, while the joint-PD robot slides far away. The controller keeps trying to return to its original position without taking a step; inaccurate friction assumptions and the four-contact assumption become invalid. This is a useful failure analysis, not a supported operating condition.

Results are deterministic simulated trials with exact simulator state and known payload mass. There is no sensor noise, estimator, random terrain, hardware validation, or statistical success-rate claim. A meaningful next step is contact-aware control and stepping recovery; a later step is state estimation and randomized evaluation.

## Rebuild the visual demo

```powershell
.venv\Scripts\python -m simulation.export
.venv\Scripts\python -m simulation.benchmark
npm install
npm run dev
```

Open `/sentry.html`. The browser replays rendered MuJoCo frames with synchronized telemetry; it does not run live physics. The native viewer is the interactive physics entry point. Export requires a working OpenGL context; on a Linux server use Xvfb/software OpenGL. `--skip-video` exports telemetry only. The report records the MuJoCo/Python versions and a SHA-256 of the simulation source. Videos and telemetry are checked in so recruiters need only a browser.

## Explain it in an interview

Start with the problem: “Changing body height and rejecting disturbances requires coordinating ground forces across four legs.” Show joint PD versus whole-body control at the same timestamp. Explain the wrench matrix and why feet cannot pull on the floor. Point out gravity feedforward, friction limits, and torque saturation. Then show the slippery-floor failure and describe what would need to change to recover by stepping.

Before claiming this as your work, run it, change a gain or push magnitude, regenerate the results, and explain what changed. A defensible portfolio claim is: “Implemented and evaluated a MuJoCo quadruped stance-control experiment, comparing centroidal force allocation with joint PD under scripted disturbances.” Do not claim locomotion, RL training, hardware deployment, or authorship of a pretrained policy.

## References

- [MuJoCo Python API and native viewer](https://mujoco.readthedocs.io/en/stable/python.html)
- [MuJoCo Jacobian and contact-force functions](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)
- [MuJoCo equations of motion and contacts](https://mujoco.readthedocs.io/en/stable/computation/index.html)
