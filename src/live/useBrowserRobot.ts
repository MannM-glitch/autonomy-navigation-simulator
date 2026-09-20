import { useCallback, useEffect, useRef, useState } from 'react';
import loadMujoco, { type MainModule } from '@mujoco/mujoco';
import wasmUrl from '@mujoco/mujoco/mujoco.wasm?url';
import { BrowserRobot, type Command, type RobotState } from './physics';
import { createRobotDisplay, type DisplayMode } from './display';
import type { Arena } from './navigation';

let engine: Promise<MainModule> | null=null;
const movementKeys=new Set(['w','a','s','d','q','e','arrowup','arrowdown','arrowleft','arrowright']);
function inputCommand(held: Set<string>): Command {
  const down=(...names:string[])=>names.some(n=>held.has(n))?1:0;
  return {action:'drive',forward:down('w','arrowup')-down('s','arrowdown'),strafe:down('q')-down('e'),turn:down('a','arrowleft')-down('d','arrowright')};
}

export function useBrowserRobot() {
  const sceneRef=useRef<HTMLDivElement>(null),robotRef=useRef<BrowserRobot|null>(null);
  const [displayMode,setDisplayMode]=useState<DisplayMode>('3d');
  const input=useRef(new Set<string>());
  const [state,setState]=useState<RobotState|null>(null),[connected,setConnected]=useState(false);
  const [held,setHeld]=useState<Set<string>>(new Set()),[error,setError]=useState('');
  const send=useCallback((command:Command)=>{
    const robot=robotRef.current;
    if(robot){robot.command(command);setError('');}
  },[]);
  const release=useCallback((stop=true)=>{
    input.current.clear();setHeld(new Set());
    if(stop)robotRef.current?.stop();
  },[]);
  const setKey=useCallback((key:string,pressed:boolean)=>{
    const robot=robotRef.current;
    if(!robot || (pressed && (robot.paused || robot.fallen)))return;
    if(pressed)input.current.add(key);else input.current.delete(key);
    setHeld(new Set(input.current));robot.command(inputCommand(input.current));
  },[]);

  useEffect(()=>{
    let cancelled=false,frame=0,robot:BrowserRobot|null=null,renderer:ReturnType<typeof createRobotDisplay>|null=null;
    const controller=new AbortController();
    async function initialize() {
      try {
        engine ??= loadMujoco({locateFile:(path:string)=>path.endsWith('.wasm')?wasmUrl:path});
        const [mj,xmlResponse,arenaResponse]=await Promise.all([engine,fetch('sentry/arena.xml',{signal:controller.signal}),fetch('sentry/arena.json',{signal:controller.signal})]);
        if(!xmlResponse.ok || !arenaResponse.ok)throw new Error('Robot model could not load. Reload to retry.');
        const xml=await xmlResponse.text(),arena=await arenaResponse.json() as Arena;
        if(cancelled)return;
        robot=new BrowserRobot(mj,xml,arena);
        renderer=createRobotDisplay(sceneRef.current!,robot,setDisplayMode,new URLSearchParams(location.search).get('view')==='2d');
        robotRef.current=robot;
        setConnected(true);setError('');setState(robot.snapshot());
        let previous=performance.now(),accumulator=0,lastState=0,statsAt=previous,statsTime=0,frames=0,fps=60,factor=1;
        const tick=(now:number)=>{
          if(cancelled || !robot || !renderer)return;
          // Allow up to one second of fixed-step catch-up for embedded tabs.
          // Never jump the physics timestep or replay a long suspended interval.
          const elapsed=Math.min(1,(now-previous)/1000);previous=now;
          if(!document.hidden){
            accumulator+=elapsed;
            const steps=Math.min(500,Math.floor(accumulator/.002));accumulator-=steps*.002;
            robot.advance(steps);renderer.render();frames++;
            if(now-statsAt>1000){factor=Math.max(0,(robot.data.time-statsTime)/((now-statsAt)/1000));fps=frames/((now-statsAt)/1000);statsAt=now;statsTime=robot.data.time;frames=0;}
            if(now-lastState>70){setState({...robot.snapshot(),realTimeFactor:factor,fps});lastState=now;}
          } else accumulator=0;
          // Embedded browsers can throttle rAF even while visible. Keep the
          // physics clock independent of compositor scheduling, at 30 Hz.
          frame=window.setTimeout(()=>tick(performance.now()),Math.max(0,1000/30-(performance.now()-now)));
        };
        frame=window.setTimeout(()=>tick(performance.now()),0);
      } catch(e) {
        if(!cancelled){setConnected(false);setError(e instanceof Error?e.message:'Browser physics could not start.');}
        renderer?.dispose();robot?.dispose();renderer=null;robot=null;robotRef.current=null;
      }
    }
    void initialize();
    return ()=>{cancelled=true;controller.abort();clearTimeout(frame);robotRef.current=null;renderer?.dispose();robot?.dispose();};
  },[]);

  useEffect(()=>{
    const typing=(target:EventTarget|null)=>target instanceof HTMLElement && (target.isContentEditable || ['INPUT','TEXTAREA','SELECT'].includes(target.tagName));
    const down=(event:KeyboardEvent)=>{
      if(typing(event.target)||event.ctrlKey||event.metaKey||event.altKey)return;
      const key=event.key.toLowerCase();
      if(key===' '){event.preventDefault();release();}
      if(movementKeys.has(key)){event.preventDefault();if(!event.repeat)setKey(key,true);}
    };
    const up=(event:KeyboardEvent)=>{const key=event.key.toLowerCase();if(movementKeys.has(key)&&input.current.has(key)){event.preventDefault();setKey(key,false);}};
    const blur=()=>release();
    const visibility=()=>{if(document.hidden)release();};
    window.addEventListener('keydown',down);window.addEventListener('keyup',up);window.addEventListener('blur',blur);window.addEventListener('pagehide',blur);document.addEventListener('visibilitychange',visibility);
    return ()=>{release();window.removeEventListener('keydown',down);window.removeEventListener('keyup',up);window.removeEventListener('blur',blur);window.removeEventListener('pagehide',blur);document.removeEventListener('visibilitychange',visibility);};
  },[release,setKey]);
  return {state,connected,held,error,send,release,setKey,sceneRef,displayMode};
}
