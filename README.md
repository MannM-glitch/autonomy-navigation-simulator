# Autonomy Navigation Simulator

Interactive robotics software project for autonomous route planning, differential-drive control, noisy range sensing, and telemetry inspection.

## Project Summary

The simulator models a mobile robot that plans a route through obstacles, tracks the route with a PID controller, scans nearby geometry with noisy range sensors, and exposes runtime diagnostics in a browser UI.

## Capabilities

- A* global path planning on an occupancy grid with adjustable obstacle clearance.
- Differential-drive kinematics for two-wheel robot motion.
- PID heading control for route tracking.
- Simulated range sensors with configurable measurement noise.
- Route, blocked-cell, explored-cell, sensor, and robot-trace visualization.
- Unit tests for robotics math, sensing, motion, and planning behavior.
- Automated engineering-log workflow for steady experiment records.

## Skill Alignment

Scan date: August 24, 2026.

Robotics internship descriptions repeatedly emphasized:

- Programming: Python, C++, TypeScript/JavaScript, Git, Linux, and testing.
- Robotics stack: ROS or ROS2, simulation, sensor pipelines, robot state, autonomy loops.
- Math and controls: kinematics, PID control, coordinate frames, dynamics basics.
- Perception and planning: cameras, lidar/range sensors, computer vision, SLAM or mapping, path planning.
- Engineering habits: debugging, data logs, clear documentation, experiments, collaboration.

Sources checked while creating the project scope:

- [Amazon Robotics Software Development Engineer Intern/Co-op 2026](https://www.amazon.jobs/en/jobs/3136266/robotics-software-development-engineer-intern-co-op-2026)
- [MERL internship openings](https://www.merl.com/employment/internship-openings)
- [Apptronik Robotics Software Intern - Real-Time Controls](https://job-boards.greenhouse.io/apptronik/jobs/5985132004)
- [Lunar Outpost Robotics Engineering Intern - Summer 2026](https://jobs.type1ventures.com/companies/lunar-outpost/jobs/58926030-robotics-engineering-intern-summer-2026)
- [ROS Jobs wiki](https://wiki.ros.org/Jobs)

## Quick Start

```bash
npm install
npm run dev
```

Then open the URL printed by Vite.

## Scripts

```bash
npm run dev      # Start the app locally
npm run build    # Type-check and build
npm test         # Run unit tests
npm run daily    # Add today's engineering-log entry
```

## Technical Roadmap

Phase 1: Differential-drive simulator, PID controller, route telemetry.

Phase 2: A* global planning with obstacle clearance and explored-cell diagnostics.

Phase 3: Sensor-processing experiments with raw versus filtered range readings.

Phase 4: ROS2-style architecture notes for planner, perception, controller, and telemetry nodes.

Phase 5: Perception extension using a camera mock or target-tracking module.

Phase 6: Demo report with screenshots, metrics, tests, and failure analysis.

## Engineering Log

The workflow in `.github/workflows/daily-progress.yml` runs once a day. It creates a dated engineering entry from `data/missions.json`, runs tests, commits the update, and pushes it with `GITHUB_TOKEN`.

GitHub scheduled workflows run after this repo is pushed to GitHub and Actions are enabled for the repository.

