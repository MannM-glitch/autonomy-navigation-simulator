import { describe, expect, it } from 'vitest';
import {
  createPid,
  getHeadingError,
  makeSensorReadings,
  normalizeAngle,
  planAStarPath,
  stepDifferentialDrive,
  type RobotState
} from './robotics';

describe('robotics math helpers', () => {
  it('normalizes angles into the -pi to pi range', () => {
    expect(normalizeAngle(Math.PI * 3)).toBeCloseTo(Math.PI);
    expect(normalizeAngle(Math.PI * -3)).toBeCloseTo(-Math.PI);
  });

  it('computes heading error toward a waypoint', () => {
    const robot: RobotState = {
      x: 0,
      y: 0,
      heading: 0,
      leftVelocity: 0,
      rightVelocity: 0
    };

    expect(getHeadingError(robot, { x: 0, y: 10 })).toBeCloseTo(Math.PI / 2);
  });

  it('moves straight when wheel speeds match', () => {
    const next = stepDifferentialDrive(
      {
        x: 50,
        y: 50,
        heading: 0,
        leftVelocity: 20,
        rightVelocity: 20
      },
      1,
      50,
      500,
      500
    );

    expect(next.x).toBeCloseTo(70);
    expect(next.y).toBeCloseTo(50);
  });

  it('turns when wheel speeds differ', () => {
    const next = stepDifferentialDrive(
      {
        x: 50,
        y: 50,
        heading: 0,
        leftVelocity: 10,
        rightVelocity: 30
      },
      1,
      50,
      500,
      500
    );

    expect(next.heading).toBeGreaterThan(0);
  });

  it('updates PID output using proportional, integral, and derivative terms', () => {
    const pid = createPid(2, 0.5, 0.25);
    const first = pid.step(1, 0.1);
    const second = first.controller.step(0.4, 0.1);

    expect(first.output).toBeGreaterThan(2);
    expect(second.output).toBeLessThan(first.output);
  });

  it('detects an obstacle in front of a range sensor', () => {
    const readings = makeSensorReadings(
      {
        x: 50,
        y: 50,
        heading: 0,
        leftVelocity: 0,
        rightVelocity: 0
      },
      [{ x: 110, y: 50, radius: 12 }],
      500,
      500
    );

    const center = readings[2];
    expect(center.hit).toBe(true);
    expect(center.distance).toBeLessThan(70);
  });

  it('plans a grid route around a blocked region', () => {
    const plan = planAStarPath(
      { x: 30, y: 90 },
      { x: 270, y: 90 },
      [{ x: 150, y: 90, radius: 35 }],
      {
        fieldWidth: 300,
        fieldHeight: 210,
        cellSize: 30,
        clearance: 0
      }
    );

    expect(plan.success).toBe(true);
    expect(plan.path.length).toBeGreaterThan(2);
    expect(plan.routeLength).toBeGreaterThan(240);
    expect(plan.exploredCells.length).toBeGreaterThan(0);
    expect(plan.path.some((point) => Math.hypot(point.x - 150, point.y - 90) < 35)).toBe(false);
  });
});
