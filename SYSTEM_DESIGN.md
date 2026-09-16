# System Design Notes

## Robot Model

The simulator uses a differential-drive robot with two independently controlled wheels. Equal wheel speeds move the robot forward. Different wheel speeds rotate the robot.

Robot state:

- `x`: horizontal position
- `y`: vertical position
- `heading`: direction the robot is facing
- `leftVelocity`: left wheel speed
- `rightVelocity`: right wheel speed

## Global Planner

The global planner converts the field into an occupancy grid. Each cell is marked free or blocked. Obstacles are expanded by a clearance margin so the route keeps space around the robot body.

The planner uses A* search:

1. Start at the robot's initial grid cell.
2. Prefer cells that look cheap to reach and close to the goal.
3. Reject blocked cells and diagonal moves that cut through blocked corners.
4. Reconstruct the best route when the goal cell is reached.

## Local Controller

The controller tracks the route one waypoint at a time. It measures the angle between the robot's heading and the active waypoint, then feeds that error into a PID controller.

PID terms:

- Proportional: reacts to current heading error.
- Integral: reacts to accumulated bias.
- Derivative: reacts to fast changes in error.

## Perception

The range-sensor model casts rays from the robot. Each ray returns the distance to the nearest wall or obstacle within range. Deterministic noise is added so sensor readings behave like measurements rather than perfect truth.

## ROS2-Style Decomposition

| Node | Topic | Message |
| --- | --- | --- |
| `planner_node` | `/global_route` | `PlannedPath` |
| `range_sensor_node` | `/range_scan` | `RangeReading[]` |
| `controller_node` | `/wheel_cmd` | `WheelCommand` |
| `telemetry_node` | `/robot_state` | `RobotState` |

