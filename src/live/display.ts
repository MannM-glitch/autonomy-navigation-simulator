import { RobotRenderer } from './renderer';
import { SoftwareRobotRenderer } from './softwareRenderer';
import type { BrowserRobot } from './physics';

export type DisplayMode='3d'|'compatibility';
type Renderer={render:()=>void;dispose:()=>void};

/** A failed/lost GPU context must not prevent the physics engine from running. */
export function createRobotDisplay(host: HTMLElement,robot: BrowserRobot,onMode:(mode:DisplayMode)=>void,softwareOnly=false) {
  let renderer:Renderer|null=null,canvas:HTMLCanvasElement,disposed=false;
  const freshCanvas=()=>{
    const next=document.createElement('canvas');next.style.width='100%';next.style.height='100%';next.style.display='block';
    host.replaceChildren(next);return next;
  };
  const useSoftware=()=>{
    canvas?.removeEventListener('webglcontextlost',lost);
    renderer?.dispose();renderer=null;
    // Canvas context types cannot be changed after creation: always use a fresh element.
    canvas=freshCanvas();renderer=new SoftwareRobotRenderer(canvas,robot);onMode('compatibility');
  };
  const lost=(event:Event)=>{event.preventDefault();if(!disposed)useSoftware();};
  canvas=freshCanvas();
  if(softwareOnly)useSoftware();
  else {
    try {renderer=new RobotRenderer(canvas,robot);renderer.render();canvas.addEventListener('webglcontextlost',lost);onMode('3d');}
    catch {useSoftware();}
  }
  return {
    render:()=>renderer?.render(),
    dispose:()=>{disposed=true;canvas.removeEventListener('webglcontextlost',lost);renderer?.dispose();renderer=null;host.replaceChildren();},
  };
}
