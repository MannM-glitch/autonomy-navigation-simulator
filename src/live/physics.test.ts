import { beforeAll, describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import loadMujoco, { type MainModule } from '@mujoco/mujoco';
import { BrowserRobot, allocate } from './physics';
import { planPath, segmentClear, type Arena } from './navigation';

let mj: MainModule;
const xml=readFileSync('public/sentry/arena.xml','utf8');
const arena=JSON.parse(readFileSync('public/sentry/arena.json','utf8')) as Arena;
beforeAll(async()=>{mj=await loadMujoco();});

describe('browser MuJoCo physics',()=>{
  it('supports the weight with feasible contact forces',()=>{
    const {forces,ok}=allocate([[.25,.25,0],[.25,-.25,0],[-.25,.25,0],[-.25,-.25,0]],[0,0,.4],[0,0,160,0,0,0],[true,true,true,true]);
    expect(ok).toBe(true);
    expect(forces.reduce((a,f)=>a+f[2],0)).toBeCloseTo(160,0);
  });
  it('walks under motor torque and holds after a stop',()=>{
    const robot=new BrowserRobot(mj,xml,arena);
    try {
      robot.command({action:'drive',forward:-1});
      robot.advance(5000);
      expect(robot.data.qpos[0]).toBeLessThan(-.35);
      expect(robot.fallen).toBe(false);
      robot.command({action:'stop'});robot.advance(1500);
      const x=robot.data.qpos[0];robot.advance(1000);
      expect(Math.abs(robot.data.qpos[0]-x)).toBeLessThan(.01);
      expect(robot.snapshot().loads.reduce((a,b)=>a+b,0)).toBeGreaterThan(100);
    } finally {robot.dispose();}
  },30000);
  it('routes around crates and reaches the destination without contact',()=>{
    const robot=new BrowserRobot(mj,xml,arena);
    try {
      robot.command({action:'goal',target:[1.4,1.3]});
      for(let i=0;i<4000 && robot.mode==='navigate';i++){robot.advance(10);robot.snapshot();}
      robot.advance(1000);
      expect(robot.mode).toBe('arrived');expect(robot.fallen).toBe(false);
      expect(robot.collisionSamples).toBe(0);
      expect(Math.hypot(robot.snapshot().center[0]-1.4,robot.snapshot().center[1]-1.3)).toBeLessThan(.15);
    } finally {robot.dispose();}
  },60000);
  it('plans only clear segments and rejects blocked targets',()=>{
    const path=planPath([0,0],[1.4,1.3],arena);
    expect(path.length).toBeGreaterThan(2);
    for(let i=1;i<path.length;i++)expect(segmentClear(path[i-1],path[i],arena)).toBe(true);
    expect(()=>planPath([0,0],[.95,.35],arena)).toThrow();
  });
  it('strafes and rotates through joint torques',()=>{
    const robot=new BrowserRobot(mj,xml,arena);
    try {
      robot.command({action:'drive',strafe:1});robot.advance(3500);
      expect(robot.snapshot().center[1]).toBeGreaterThan(.2);
      robot.command({action:'stop'});robot.advance(1000);
      robot.command({action:'drive',turn:1});robot.advance(4000);
      expect(robot.snapshot().yaw).toBeGreaterThan(.35);
      expect(robot.fallen).toBe(false);expect(robot.failures).toBe(0);
    } finally {robot.dispose();}
  },30000);
  it('pauses dynamics and resets state after movement',()=>{
    const robot=new BrowserRobot(mj,xml,arena);
    try {
      robot.command({action:'drive',forward:-1});robot.advance(1500);
      robot.command({action:'pause',paused:true});
      const time=robot.data.time,pose=Array.from(robot.data.qpos);
      robot.advance(500);expect(robot.data.time).toBe(time);expect(Array.from(robot.data.qpos)).toEqual(pose);
      robot.command({action:'reset'});
      expect(robot.data.time).toBe(0);expect(robot.data.qpos[0]).toBe(0);expect(robot.steps).toBe(0);
      expect(robot.paused).toBe(false);expect(robot.manual).toEqual([0,0,0]);
    } finally {robot.dispose();}
  });
});
