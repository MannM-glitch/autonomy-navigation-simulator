export type RobotState = {
  x: number;
  y: number;
  heading: number;
  leftVelocity: number;
  rightVelocity: number;
};

export type Waypoint = {
  x: number;
  y: number;
};

export type Obstacle = {
  x: number;
  y: number;
  radius: number;
};

export type SensorReading = {
  angle: number;
  distance: number;
  hit: boolean;
};

export type GridCell = {
  col: number;
  row: number;
};

export type PlannerConfig = {
  fieldWidth: number;
  fieldHeight: number;
  cellSize: number;
  clearance: number;
};

export type PlannedPath = {
  success: boolean;
  path: Waypoint[];
  exploredCells: GridCell[];
  blockedCells: GridCell[];
  startCell: GridCell;
  goalCell: GridCell;
  columns: number;
  rows: number;
  routeLength: number;
};

export type PidState = {
  kp: number;
  ki: number;
  kd: number;
  integral: number;
  previousError: number;
  output: number;
  step: (error: number, dt: number) => { output: number; controller: PidState };
};

export function clamp(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, value));
}

export function normalizeAngle(angle: number) {
  let next = angle;
  while (next > Math.PI) {
    next -= Math.PI * 2;
  }
  while (next < -Math.PI) {
    next += Math.PI * 2;
  }
  return next;
}

export function distance(a: Waypoint, b: Waypoint) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export function getHeadingError(robot: RobotState, target: Waypoint) {
  const targetHeading = Math.atan2(target.y - robot.y, target.x - robot.x);
  return normalizeAngle(targetHeading - robot.heading);
}

export function createPid(
  kp: number,
  ki: number,
  kd: number,
  integral = 0,
  previousError = 0,
  output = 0
): PidState {
  return {
    kp,
    ki,
    kd,
    integral,
    previousError,
    output,
    step(error: number, dt: number) {
      const safeDt = Math.max(dt, 0.001);
      const nextIntegral = clamp(integral + error * safeDt, -8, 8);
      const derivative = (error - previousError) / safeDt;
      const nextOutput = kp * error + ki * nextIntegral + kd * derivative;

      return {
        output: nextOutput,
        controller: createPid(kp, ki, kd, nextIntegral, error, nextOutput)
      };
    }
  };
}

export function stepDifferentialDrive(
  robot: RobotState,
  dt: number,
  wheelBase: number,
  fieldWidth: number,
  fieldHeight: number
): RobotState {
  const linearVelocity = (robot.rightVelocity + robot.leftVelocity) / 2;
  const angularVelocity = (robot.rightVelocity - robot.leftVelocity) / wheelBase;
  const nextHeading = normalizeAngle(robot.heading + angularVelocity * dt);
  const nextX = clamp(robot.x + linearVelocity * Math.cos(nextHeading) * dt, 24, fieldWidth - 24);
  const nextY = clamp(robot.y + linearVelocity * Math.sin(nextHeading) * dt, 24, fieldHeight - 24);

  return {
    ...robot,
    x: nextX,
    y: nextY,
    heading: nextHeading
  };
}

export function makeSensorReadings(
  robot: RobotState,
  obstacles: Obstacle[],
  fieldWidth: number,
  fieldHeight: number,
  noise = 0
): SensorReading[] {
  const sensorAngles = [-0.82, -0.42, 0, 0.42, 0.82];
  return sensorAngles.map((angle, index) => {
    const maxRange = 210;
    const absoluteAngle = robot.heading + angle;
    const cast = castRange(robot, absoluteAngle, obstacles, fieldWidth, fieldHeight, maxRange);
    const deterministicNoise = noise * Math.sin(robot.x * 0.09 + robot.y * 0.05 + index * 1.7);
    return {
      angle,
      distance: clamp(cast.distance + deterministicNoise, 0, maxRange),
      hit: cast.hit
    };
  });
}

function castRange(
  origin: Waypoint,
  angle: number,
  obstacles: Obstacle[],
  fieldWidth: number,
  fieldHeight: number,
  maxRange: number
) {
  const step = 6;
  for (let distanceAlongRay = step; distanceAlongRay <= maxRange; distanceAlongRay += step) {
    const x = origin.x + Math.cos(angle) * distanceAlongRay;
    const y = origin.y + Math.sin(angle) * distanceAlongRay;
    const outsideField = x < 12 || x > fieldWidth - 12 || y < 12 || y > fieldHeight - 12;
    const hitsObstacle = obstacles.some((obstacle) => distance({ x, y }, obstacle) <= obstacle.radius);
    if (outsideField || hitsObstacle) {
      return { distance: distanceAlongRay, hit: true };
    }
  }

  return { distance: maxRange, hit: false };
}

export function planAStarPath(
  start: Waypoint,
  goal: Waypoint,
  obstacles: Obstacle[],
  config: PlannerConfig
): PlannedPath {
  const columns = Math.ceil(config.fieldWidth / config.cellSize);
  const rows = Math.ceil(config.fieldHeight / config.cellSize);
  const startCell = pointToGridCell(start, config, columns, rows);
  const goalCell = pointToGridCell(goal, config, columns, rows);
  const blocked = buildBlockedCellSet(obstacles, config, columns, rows);

  blocked.delete(cellKey(startCell));
  blocked.delete(cellKey(goalCell));

  const open = new Map<string, SearchNode>();
  const closed = new Set<string>();
  const exploredCells: GridCell[] = [];
  const startKey = cellKey(startCell);

  open.set(startKey, {
    cell: startCell,
    key: startKey,
    g: 0,
    f: heuristic(startCell, goalCell)
  });

  while (open.size > 0) {
    const current = getLowestCostNode(open);
    open.delete(current.key);

    if (closed.has(current.key)) {
      continue;
    }

    closed.add(current.key);
    exploredCells.push(current.cell);

    if (current.cell.col === goalCell.col && current.cell.row === goalCell.row) {
      const path = normalizePlannedPath(reconstructPath(current), start, goal, config);
      return {
        success: true,
        path,
        exploredCells,
        blockedCells: [...blocked].map(cellFromKey),
        startCell,
        goalCell,
        columns,
        rows,
        routeLength: measureRoute(path)
      };
    }

    for (const neighbor of getNeighbors(current.cell, columns, rows)) {
      const neighborKey = cellKey(neighbor.cell);
      if (closed.has(neighborKey) || blocked.has(neighborKey)) {
        continue;
      }

      if (neighbor.diagonal && cutsBlockedCorner(current.cell, neighbor.cell, blocked)) {
        continue;
      }

      const nextG = current.g + neighbor.cost;
      const existing = open.get(neighborKey);
      if (!existing || nextG < existing.g) {
        open.set(neighborKey, {
          cell: neighbor.cell,
          key: neighborKey,
          g: nextG,
          f: nextG + heuristic(neighbor.cell, goalCell),
          parent: current
        });
      }
    }
  }

  return {
    success: false,
    path: [start, goal],
    exploredCells,
    blockedCells: [...blocked].map(cellFromKey),
    startCell,
    goalCell,
    columns,
    rows,
    routeLength: distance(start, goal)
  };
}

type SearchNode = {
  cell: GridCell;
  key: string;
  g: number;
  f: number;
  parent?: SearchNode;
};

function pointToGridCell(point: Waypoint, config: PlannerConfig, columns: number, rows: number): GridCell {
  return {
    col: clamp(Math.floor(point.x / config.cellSize), 0, columns - 1),
    row: clamp(Math.floor(point.y / config.cellSize), 0, rows - 1)
  };
}

function gridCellToPoint(cell: GridCell, config: PlannerConfig): Waypoint {
  return {
    x: clamp(cell.col * config.cellSize + config.cellSize / 2, 0, config.fieldWidth),
    y: clamp(cell.row * config.cellSize + config.cellSize / 2, 0, config.fieldHeight)
  };
}

function buildBlockedCellSet(
  obstacles: Obstacle[],
  config: PlannerConfig,
  columns: number,
  rows: number
) {
  const blocked = new Set<string>();
  const borderMargin = 14;
  const cellRadius = Math.SQRT2 * config.cellSize * 0.5;

  for (let row = 0; row < rows; row += 1) {
    for (let col = 0; col < columns; col += 1) {
      const cell = { col, row };
      const center = gridCellToPoint(cell, config);
      const outsideField =
        center.x < borderMargin ||
        center.x > config.fieldWidth - borderMargin ||
        center.y < borderMargin ||
        center.y > config.fieldHeight - borderMargin;
      const insideObstacle = obstacles.some(
        (obstacle) => distance(center, obstacle) <= obstacle.radius + config.clearance + cellRadius
      );

      if (outsideField || insideObstacle) {
        blocked.add(cellKey(cell));
      }
    }
  }

  return blocked;
}

function getLowestCostNode(open: Map<string, SearchNode>) {
  let lowest: SearchNode | undefined;
  for (const node of open.values()) {
    if (!lowest || node.f < lowest.f || (node.f === lowest.f && node.g > lowest.g)) {
      lowest = node;
    }
  }

  return lowest!;
}

function getNeighbors(cell: GridCell, columns: number, rows: number) {
  const directions = [
    { dc: 1, dr: 0, cost: 1, diagonal: false },
    { dc: -1, dr: 0, cost: 1, diagonal: false },
    { dc: 0, dr: 1, cost: 1, diagonal: false },
    { dc: 0, dr: -1, cost: 1, diagonal: false },
    { dc: 1, dr: 1, cost: Math.SQRT2, diagonal: true },
    { dc: 1, dr: -1, cost: Math.SQRT2, diagonal: true },
    { dc: -1, dr: 1, cost: Math.SQRT2, diagonal: true },
    { dc: -1, dr: -1, cost: Math.SQRT2, diagonal: true }
  ];

  return directions
    .map((direction) => ({
      cell: { col: cell.col + direction.dc, row: cell.row + direction.dr },
      cost: direction.cost,
      diagonal: direction.diagonal
    }))
    .filter(({ cell: candidate }) => candidate.col >= 0 && candidate.col < columns && candidate.row >= 0 && candidate.row < rows);
}

function cutsBlockedCorner(from: GridCell, to: GridCell, blocked: Set<string>) {
  const horizontal = { col: to.col, row: from.row };
  const vertical = { col: from.col, row: to.row };
  return blocked.has(cellKey(horizontal)) || blocked.has(cellKey(vertical));
}

function heuristic(a: GridCell, b: GridCell) {
  return Math.hypot(a.col - b.col, a.row - b.row);
}

function reconstructPath(node: SearchNode) {
  const cells: GridCell[] = [];
  let current: SearchNode | undefined = node;
  while (current) {
    cells.unshift(current.cell);
    current = current.parent;
  }
  return cells;
}

function normalizePlannedPath(cells: GridCell[], start: Waypoint, goal: Waypoint, config: PlannerConfig) {
  const middle = cells.slice(1, -1).map((cell) => gridCellToPoint(cell, config));
  return [start, ...middle, goal];
}

function measureRoute(path: Waypoint[]) {
  return path.slice(1).reduce((total, point, index) => total + distance(path[index], point), 0);
}

function cellKey(cell: GridCell) {
  return `${cell.col},${cell.row}`;
}

function cellFromKey(key: string): GridCell {
  const [col, row] = key.split(',').map(Number);
  return { col, row };
}
