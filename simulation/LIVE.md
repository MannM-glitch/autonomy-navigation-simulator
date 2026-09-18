# SENTRY live navigation lab

SENTRY walks a floating-base quadruped through a known obstacle arena using 12 motor torques. The live app runs the official MuJoCo WebAssembly engine in the visitor's browser. Vercel serves static assets; there is no Python service, account, or remote control connection to keep online.

## Try it

```sh
npm ci
npm run dev
```

Open http://127.0.0.1:5173/sentry.html#live. Hold **W/S** to walk forward/back, **A/D** to turn, and **Q/E** to strafe. Arrow keys and the on-screen buttons also work. **Space** cancels the command. Releasing a command finishes the current footstep before holding. Leaving the page releases commands; hidden tabs pause physics.

Click clear floor on the map to select a goal, or try **Around the crates**. The green route is the plan; the blue trail is measured trunk motion. The crawl is deliberately slow: manual translation is capped at 0.06 m/s, navigation at 0.05 m/s, and a footstep takes 0.95 s. Allow roughly a minute for the crates route. Pause, reset, and switch to the arena camera with the toolbar.

The **Recorded experiment** tab keeps the original stationary balance benchmark separate from locomotion. Its disturbance-rejection numbers do not measure walking performance.

## From a destination to physical motion

1. **Planning:** A* searches a 0.1 m, eight-connected grid with a 0.65 m clearance around known obstacles. Diagonal corner cutting is disallowed. Line-of-sight shortcuts are checked at 0.025 m spacing. Blocked goals are rejected. The planner uses exact simulator state and a fixed map.
2. **Navigation:** Track waypoints using the average foot position, with a 0.1 m arrival radius. Convert manual body-frame velocity commands into world coordinates. A predictive clearance check stops translation before the next stride reaches a wall or crate.
3. **Gait:** Crawl in FL → RR → FR → RL order. Shift the center-of-mass target over the other three feet, swing one foot with a smooth 7.5 cm lift, then land. One step has 0.4 s of weight transfer, 0.4 s of swing, and 0.15 s of touchdown.
4. **Force allocation:** Convert body pose error and velocity into a desired six-dimensional wrench. Solve `min 0.5 ||W(Af − wrench)||² + 0.001 ||f||²`, with `W = diag(1,1,1,4,4,2)`. Only scheduled stance legs participate. Constrain each normal force to 0–180 N and `|fx| + |fy| ≤ 0.55 fz`. A failed solve falls back to bounded vertical support and increments a diagnostic counter.
5. **Actuation:** Map support forces and Cartesian swing-foot feedback through foot Jacobians, add model bias compensation and joint damping, and cap motor torques at ±32 N·m. MuJoCo integrates at 500 Hz; control updates at 100 Hz. The renderer reads the resulting geometry transforms. The free base is never translated by the renderer or navigation code; resetting is the only pose reset.

This is reduced-order whole-body control. The QP allocates contact forces; it does not jointly optimize joint accelerations, contact complementarity, or actuator saturation. Contact forces shown in the UI come from MuJoCo, rather than the QP's requested forces.

## Code and reproduction

| Component | Source |
| --- | --- |
| Robot and reference arena | `model.py`, `navigation.py` |
| Python reference gait / engine | `locomotion.py`, `live.py` |
| Browser dynamics and control | `../src/live/physics.ts` |
| Browser path planner | `../src/live/navigation.ts` |
| Geometry rendering and simulation clock | `../src/live/renderer.ts`, `../src/live/useBrowserRobot.ts` |
| Controls, map, telemetry | `../src/LiveLab.tsx` |

After changing the Python model or arena, regenerate the checked-in browser assets:

```sh
python -m simulation.export_arena
npm test
npm run build
python -m unittest simulation.test_control -v
```

The browser tests load the **actual MuJoCo WASM binary** in Node. They check physical translation, turning/strafe, stopping, pause/reset, obstacle-route arrival without sampled crate contacts, and infeasible-goal rejection. Tests are deterministic checks on this model and map, not a general reliability study. Python and TypeScript controllers are separate implementations and should be validated together when gains or gait logic change.

## Deployment

`vercel.json` builds the Vite app and serves SENTRY at `/`. The original differential-drive demo remains at `/index.html`. The MuJoCo binary and model are bundled in the deployment. To publish from an authenticated Vercel CLI:

```sh
vercel link --project sentry-robotics-lab
vercel deploy --prod
```

The engine is about 10 MB before compression. A WebAssembly/WebGL-capable browser is required. Rendering targets 30 frames/s; the displayed real-time factor and frame rate are measured, and may be lower on slow hardware or throttled tabs. Physics uses fixed 2 ms steps with bounded catch-up, not variable timesteps.

## Scope and an interview walkthrough

Demonstrate the crates route, take manual control, then stop and inspect contact loads. Explain why the body shifts before a foot lifts, how the friction constraints limit usable forces, and why a geometric path planner needs extra clearance for swinging legs. Compare the known-map navigation assumption with what localization and perception would need to provide on hardware.

Supported scope is a slow crawl on this flat arena with ideal state information. There is no SLAM, obstacle perception, RL policy, hardware validation, dynamic-obstacle avoidance, or robust fall recovery. The gait uses scheduled support rather than contact-triggered transitions. The clearance check is a conservative geometric heuristic, not a full swept-volume collision guarantee. A fall requires reset.

References: [official MuJoCo browser bindings](https://github.com/google-deepmind/mujoco/blob/main/wasm/README.md), [MuJoCo dynamics](https://mujoco.readthedocs.io/en/stable/computation/index.html), and [Jacobian/contact-force API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html).
