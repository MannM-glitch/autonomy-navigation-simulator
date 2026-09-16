import React, { useEffect, useMemo, useRef, useState } from 'react';
import { createRoot } from 'react-dom/client';
import {
  Activity,
  BrainCircuit,
  Gauge,
  GitBranch,
  Map,
  Network,
  Pause,
  Play,
  Radar,
  RotateCcw,
  Route,
  ScanLine,
  SlidersHorizontal
} from 'lucide-react';
import {
  clamp,
  createPid,
  distance,
  getHeadingError,
  makeSensorReadings,
  planAStarPath,
  stepDifferentialDrive,
  type Obstacle,
  type PidState,
  type PlannedPath,
  type RobotState,
  type SensorReading,
  type Waypoint
} from './robotics';
import './styles.css';

type Mode = 'control' | 'perception' | 'planner' | 'architecture';

const FIELD_WIDTH = 900;
const FIELD_HEIGHT = 560;
const WHEEL_BASE = 54;
const GRID_SIZE = 30;

const missionGoal: Waypoint = { x: 802, y: 438 };

const obstacles: Obstacle[] = [
  { x: 438, y: 164, radius: 54 },
  { x: 566, y: 362, radius: 70 },
  { x: 302, y: 302, radius: 42 },
  { x: 698, y: 218, radius: 44 }
];

const startingRobot: RobotState = {
  x: 118,
  y: 104,
  heading: 0.24,
  leftVelocity: 0,
  rightVelocity: 0
};

const systemNotes = {
  control: {
    eyebrow: 'Controller',
    title: 'Closed-loop PID steering converts route error into wheel commands.',
    points: ['Angle error is measured every frame', 'P/I/D terms shape the turn rate', 'Wheel speeds execute the correction']
  },
  perception: {
    eyebrow: 'Perception',
    title: 'Range scans estimate nearby structure with imperfect measurements.',
    points: ['Rays return distance to walls or obstacles', 'Noise changes confidence, not ground truth', 'Closest range is a safety signal']
  },
  planner: {
    eyebrow: 'Global planner',
    title: 'A* searches an occupancy grid before the controller starts tracking.',
    points: ['Obstacle clearance expands blocked cells', 'Explored cells show search effort', 'The yellow route is the selected path']
  },
  architecture: {
    eyebrow: 'System architecture',
    title: 'Planning, perception, control, and telemetry are separated like robotics nodes.',
    points: ['planner_node publishes route waypoints', 'controller_node publishes wheel commands', 'dashboard_node records state']
  }
} satisfies Record<Mode, { eyebrow: string; title: string; points: string[] }>;

function useAnimationFrame(enabled: boolean, callback: (dt: number) => void) {
  const frameRef = useRef<number | undefined>(undefined);
  const lastRef = useRef<number | undefined>(undefined);
  const callbackRef = useRef(callback);

  callbackRef.current = callback;

  useEffect(() => {
    if (!enabled) {
      return;
    }

    const tick = (time: number) => {
      if (lastRef.current === undefined) {
        lastRef.current = time;
      }
      const dt = Math.min((time - lastRef.current) / 1000, 0.05);
      lastRef.current = time;
      callbackRef.current(dt);
      frameRef.current = requestAnimationFrame(tick);
    };

    frameRef.current = requestAnimationFrame(tick);

    return () => {
      if (frameRef.current !== undefined) {
        cancelAnimationFrame(frameRef.current);
      }
      lastRef.current = undefined;
    };
  }, [enabled]);
}

function drawField(
  canvas: HTMLCanvasElement,
  robot: RobotState,
  currentWaypoint: number,
  sensors: SensorReading[],
  tracePath: Waypoint[],
  plannedRoute: PlannedPath,
  target: Waypoint
) {
  const ctx = canvas.getContext('2d');
  if (!ctx) {
    return;
  }

  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  ctx.setTransform((rect.width * dpr) / FIELD_WIDTH, 0, 0, (rect.height * dpr) / FIELD_HEIGHT, 0, 0);

  ctx.fillStyle = '#f7f4ed';
  ctx.fillRect(0, 0, FIELD_WIDTH, FIELD_HEIGHT);

  ctx.strokeStyle = '#d8d2c4';
  ctx.lineWidth = 1;
  for (let x = 0; x <= FIELD_WIDTH; x += GRID_SIZE) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, FIELD_HEIGHT);
    ctx.stroke();
  }
  for (let y = 0; y <= FIELD_HEIGHT; y += GRID_SIZE) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(FIELD_WIDTH, y);
    ctx.stroke();
  }

  ctx.fillStyle = 'rgba(160, 71, 58, 0.16)';
  plannedRoute.blockedCells.forEach((cell) => {
    ctx.fillRect(cell.col * GRID_SIZE, cell.row * GRID_SIZE, GRID_SIZE, GRID_SIZE);
  });

  ctx.fillStyle = 'rgba(89, 122, 150, 0.12)';
  plannedRoute.exploredCells.forEach((cell) => {
    ctx.fillRect(cell.col * GRID_SIZE + 6, cell.row * GRID_SIZE + 6, GRID_SIZE - 12, GRID_SIZE - 12);
  });

  ctx.strokeStyle = '#263238';
  ctx.lineWidth = 3;
  ctx.strokeRect(10, 10, FIELD_WIDTH - 20, FIELD_HEIGHT - 20);

  if (plannedRoute.path.length > 1) {
    ctx.strokeStyle = '#d69d23';
    ctx.lineWidth = 5;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.beginPath();
    plannedRoute.path.forEach((point, index) => {
      if (index === 0) {
        ctx.moveTo(point.x, point.y);
      } else {
        ctx.lineTo(point.x, point.y);
      }
    });
    ctx.stroke();

    plannedRoute.path.forEach((point, index) => {
      if (index === 0 || index === plannedRoute.path.length - 1 || index % 3 !== 0) {
        return;
      }
      ctx.fillStyle = '#f3c969';
      ctx.beginPath();
      ctx.arc(point.x, point.y, 4, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  if (tracePath.length > 1) {
    ctx.strokeStyle = 'rgba(15, 139, 141, 0.82)';
    ctx.lineWidth = 4;
    ctx.beginPath();
    tracePath.forEach((point, index) => {
      if (index === 0) {
        ctx.moveTo(point.x, point.y);
      } else {
        ctx.lineTo(point.x, point.y);
      }
    });
    ctx.stroke();
  }

  obstacles.forEach((obstacle) => {
    ctx.fillStyle = 'rgba(160, 71, 58, 0.16)';
    ctx.beginPath();
    ctx.arc(obstacle.x, obstacle.y, obstacle.radius + 18, 0, Math.PI * 2);
    ctx.fill();

    const gradient = ctx.createRadialGradient(
      obstacle.x - 14,
      obstacle.y - 18,
      8,
      obstacle.x,
      obstacle.y,
      obstacle.radius
    );
    gradient.addColorStop(0, '#f6a35b');
    gradient.addColorStop(1, '#a0473a');
    ctx.fillStyle = gradient;
    ctx.beginPath();
    ctx.arc(obstacle.x, obstacle.y, obstacle.radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#78362f';
    ctx.lineWidth = 2;
    ctx.stroke();
  });

  drawRouteMarker(ctx, plannedRoute.path[0], 'S', '#263238');
  drawRouteMarker(ctx, missionGoal, 'G', '#0f8b8d');
  drawRouteMarker(ctx, target, String(currentWaypoint + 1), '#d69d23');

  sensors.forEach((reading) => {
    const absolute = robot.heading + reading.angle;
    const endX = robot.x + Math.cos(absolute) * reading.distance;
    const endY = robot.y + Math.sin(absolute) * reading.distance;
    ctx.strokeStyle = reading.hit ? 'rgba(196, 87, 70, 0.6)' : 'rgba(15, 139, 141, 0.32)';
    ctx.lineWidth = reading.hit ? 3 : 2;
    ctx.beginPath();
    ctx.moveTo(robot.x, robot.y);
    ctx.lineTo(endX, endY);
    ctx.stroke();
  });

  ctx.save();
  ctx.translate(robot.x, robot.y);
  ctx.rotate(robot.heading);

  ctx.fillStyle = '#263238';
  roundRect(ctx, -34, -22, 68, 44, 8);
  ctx.fill();
  ctx.fillStyle = '#21a0a0';
  roundRect(ctx, -14, -17, 38, 34, 5);
  ctx.fill();
  ctx.fillStyle = '#f3c969';
  ctx.beginPath();
  ctx.moveTo(38, 0);
  ctx.lineTo(20, -12);
  ctx.lineTo(20, 12);
  ctx.closePath();
  ctx.fill();
  ctx.fillStyle = '#11191c';
  roundRect(ctx, -30, -31, 52, 10, 4);
  ctx.fill();
  roundRect(ctx, -30, 21, 52, 10, 4);
  ctx.fill();
  ctx.restore();
}

function drawRouteMarker(ctx: CanvasRenderingContext2D, point: Waypoint | undefined, label: string, color: string) {
  if (!point) {
    return;
  }

  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(point.x, point.y, 12, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#ffffff';
  ctx.font = '700 11px Inter, system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, point.x, point.y + 1);
}

function roundRect(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  width: number,
  height: number,
  radius: number
) {
  ctx.beginPath();
  ctx.moveTo(x + radius, y);
  ctx.arcTo(x + width, y, x + width, y + height, radius);
  ctx.arcTo(x + width, y + height, x, y + height, radius);
  ctx.arcTo(x, y + height, x, y, radius);
  ctx.arcTo(x, y, x + width, y, radius);
  ctx.closePath();
}

function App() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [running, setRunning] = useState(true);
  const [mode, setMode] = useState<Mode>('planner');
  const [robot, setRobot] = useState<RobotState>(startingRobot);
  const [pid, setPid] = useState<PidState>(() => createPid(2.4, 0.02, 0.45));
  const [speed, setSpeed] = useState(74);
  const [noise, setNoise] = useState(9);
  const [clearance, setClearance] = useState(18);
  const [currentWaypoint, setCurrentWaypoint] = useState(0);
  const [tracePath, setTracePath] = useState<Waypoint[]>([{ x: startingRobot.x, y: startingRobot.y }]);

  const plannedRoute = useMemo(
    () =>
      planAStarPath(startingRobot, missionGoal, obstacles, {
        fieldWidth: FIELD_WIDTH,
        fieldHeight: FIELD_HEIGHT,
        cellSize: GRID_SIZE,
        clearance
      }),
    [clearance]
  );
  const routeTargets = plannedRoute.path.length > 1 ? plannedRoute.path : [missionGoal];
  const activeWaypoint = Math.min(currentWaypoint, routeTargets.length - 1);
  const target = routeTargets[activeWaypoint] ?? missionGoal;
  const headingError = getHeadingError(robot, target);
  const targetDistance = distance(robot, target);
  const sensors = useMemo(
    () => makeSensorReadings(robot, obstacles, FIELD_WIDTH, FIELD_HEIGHT, noise),
    [robot, noise]
  );
  const closestSensor = sensors.reduce((best, item) => Math.min(best, item.distance), Number.POSITIVE_INFINITY);
  const systemNote = systemNotes[mode];

  useAnimationFrame(running, (dt) => {
    setRobot((currentRobot) => {
      const activeTarget = routeTargets[activeWaypoint] ?? missionGoal;
      const error = getHeadingError(currentRobot, activeTarget);
      const nextPid = pid.step(error, dt);
      const turn = clamp(nextPid.output, -95, 95);
      const slowDown = clamp(distance(currentRobot, activeTarget) / 100, 0.28, 1);
      const leftVelocity = speed * slowDown - turn;
      const rightVelocity = speed * slowDown + turn;
      const nextRobot = stepDifferentialDrive(
        { ...currentRobot, leftVelocity, rightVelocity },
        dt,
        WHEEL_BASE,
        FIELD_WIDTH,
        FIELD_HEIGHT
      );

      setPid(nextPid.controller);
      setTracePath((currentPath) => {
        const last = currentPath[currentPath.length - 1] ?? nextRobot;
        if (distance(last, nextRobot) < 8) {
          return currentPath;
        }
        return [...currentPath.slice(-220), { x: nextRobot.x, y: nextRobot.y }];
      });

      if (distance(nextRobot, activeTarget) < 26) {
        if (activeWaypoint >= routeTargets.length - 1) {
          setRunning(false);
        } else {
          setCurrentWaypoint((index) => Math.min(index + 1, routeTargets.length - 1));
        }
      }

      return nextRobot;
    });
  });

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }
    drawField(canvas, robot, activeWaypoint, sensors, tracePath, plannedRoute, target);
  }, [robot, activeWaypoint, sensors, tracePath, plannedRoute, target]);

  function reset() {
    setRobot(startingRobot);
    setPid(createPid(pid.kp, pid.ki, pid.kd));
    setCurrentWaypoint(0);
    setTracePath([{ x: startingRobot.x, y: startingRobot.y }]);
    setRunning(true);
  }

  function updateClearance(value: number) {
    setClearance(value);
    setRobot(startingRobot);
    setPid(createPid(pid.kp, pid.ki, pid.kd));
    setCurrentWaypoint(0);
    setTracePath([{ x: startingRobot.x, y: startingRobot.y }]);
    setRunning(true);
  }

  const architectureRows = [
    ['planner_node', '/global_route', 'PlannedPath'],
    ['range_sensor_node', '/range_scan', 'RangeReading[]'],
    ['controller_node', '/wheel_cmd', 'WheelCommand'],
    ['telemetry_node', '/robot_state', 'RobotState']
  ];

  return (
    <main className="app-shell">
      <section className="top-bar" aria-label="Project overview">
        <div>
          <span className="label">Autonomy Navigation Simulator</span>
          <h1>Plan, track, and inspect autonomous routes.</h1>
        </div>
        <div className="status-strip" aria-label="Robot metrics">
          <Metric icon={<Route size={18} />} label="Route length" value={`${plannedRoute.routeLength.toFixed(0)} px`} />
          <Metric icon={<Network size={18} />} label="Expanded cells" value={`${plannedRoute.exploredCells.length}`} />
          <Metric icon={<Gauge size={18} />} label="Clearance" value={`${clearance} px`} />
          <Metric icon={<Radar size={18} />} label="Closest range" value={`${closestSensor.toFixed(0)} px`} />
        </div>
      </section>

      <section className="workbench">
        <aside className="panel controls-panel" aria-label="Controls">
          <div className="mode-tabs" role="tablist" aria-label="Diagnostics mode">
            {([
              ['control', SlidersHorizontal],
              ['perception', ScanLine],
              ['planner', Map],
              ['architecture', GitBranch]
            ] as const).map(([key, Icon]) => (
              <button
                key={key}
                type="button"
                className={mode === key ? 'tab active' : 'tab'}
                onClick={() => setMode(key)}
                aria-selected={mode === key}
                role="tab"
                title={key}
              >
                <Icon size={18} />
                <span>{key}</span>
              </button>
            ))}
          </div>

          <div className="control-row">
            <button
              type="button"
              className="icon-button primary"
              onClick={() => setRunning((value) => !value)}
              title={running ? 'Pause simulation' : 'Run simulation'}
              aria-label={running ? 'Pause simulation' : 'Run simulation'}
            >
              {running ? <Pause size={20} /> : <Play size={20} />}
            </button>
            <button type="button" className="icon-button" onClick={reset} title="Reset robot" aria-label="Reset robot">
              <RotateCcw size={20} />
            </button>
          </div>

          <Slider label="Base speed" min={20} max={140} value={speed} unit="px/s" onChange={setSpeed} />
          <Slider label="Planner clearance" min={0} max={38} value={clearance} unit="px" onChange={updateClearance} />
          <Slider label="Sensor noise" min={0} max={36} value={noise} unit="px" onChange={setNoise} />
          <Slider label="P gain" min={0} max={5} step={0.1} value={pid.kp} unit="" onChange={(value) => setPid(createPid(value, pid.ki, pid.kd))} />
          <Slider label="I gain" min={0} max={0.2} step={0.01} value={pid.ki} unit="" onChange={(value) => setPid(createPid(pid.kp, value, pid.kd))} />
          <Slider label="D gain" min={0} max={1.4} step={0.05} value={pid.kd} unit="" onChange={(value) => setPid(createPid(pid.kp, pid.ki, value))} />
        </aside>

        <section className="field-panel" aria-label="Robot field">
          <canvas ref={canvasRef} className="field-canvas" aria-label="Differential drive simulator" />
          <div className="path-readout" aria-label="Route statistics">
            <span>Target {activeWaypoint + 1}/{routeTargets.length}</span>
            <span>{plannedRoute.blockedCells.length} blocked cells</span>
            <span>A* {plannedRoute.success ? 'planned' : 'fallback'}</span>
            <span>{running ? 'running' : 'paused'}</span>
          </div>
        </section>

        <aside className="panel note-panel" aria-label="System diagnostics">
          <div className="note-heading">
            <BrainCircuit size={20} />
            <div>
              <span className="label">{systemNote.eyebrow}</span>
              <h2>{systemNote.title}</h2>
            </div>
          </div>
          <ul className="note-list">
            {systemNote.points.map((point) => (
              <li key={point}>{point}</li>
            ))}
          </ul>

          {mode === 'architecture' ? (
            <table className="ros-table">
              <thead>
                <tr>
                  <th>Node</th>
                  <th>Topic</th>
                  <th>Message</th>
                </tr>
              </thead>
              <tbody>
                {architectureRows.map((row) => (
                  <tr key={row[0]}>
                    <td>{row[0]}</td>
                    <td>{row[1]}</td>
                    <td>{row[2]}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          ) : (
            <div className="experiment">
              <Activity size={20} />
              <div>
                <span className="label">Telemetry note</span>
                <p>
                  Heading error is {headingError.toFixed(2)} rad while the robot is {targetDistance.toFixed(0)} px
                  from its active route target.
                </p>
              </div>
            </div>
          )}
        </aside>
      </section>
    </main>
  );
}

function Slider({
  label,
  min,
  max,
  step = 1,
  value,
  unit,
  onChange
}: {
  label: string;
  min: number;
  max: number;
  step?: number;
  value: number;
  unit: string;
  onChange: (value: number) => void;
}) {
  return (
    <label className="slider-row">
      <span>
        {label}
        <strong>
          {value.toFixed(step < 1 ? 2 : 0)} {unit}
        </strong>
      </span>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(event) => onChange(Number(event.currentTarget.value))}
      />
    </label>
  );
}

function Metric({ icon, label, value }: { icon: React.ReactNode; label: string; value: string }) {
  return (
    <div className="metric">
      {icon}
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

createRoot(document.getElementById('root')!).render(<App />);
