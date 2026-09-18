import type { MainModule, MjModel, MjData, DoubleBuffer } from '@mujoco/mujoco';
import { solveQP } from 'quadprog';
import { distance, planPath, segmentClear, type Arena, type Point } from './navigation';

const clamp = (x: number, lo: number, hi: number) => Math.max(lo,Math.min(hi,x));
const mean = (points: number[][]) => [0,1,2].map(j => points.reduce((sum,p) => sum+(p[j] ?? 0),0)/points.length);
const rotate = (v: number[], yaw: number) => [Math.cos(yaw)*v[0]-Math.sin(yaw)*v[1],Math.sin(yaw)*v[0]+Math.cos(yaw)*v[1],v[2] ?? 0];
const smooth = (x: number) => { const u=clamp(x,0,1); return u*u*(3-2*u); };
const LEGS = ['FL','FR','RL','RR'];
const PERIOD=.95, SHIFT=.4, SWING=.4, SPEED=.06, TURN=.08;
export type Command = { action: string; forward?: number; strafe?: number; turn?: number; target?: Point; paused?: boolean; view?: 'follow'|'overview' };

/** Strictly convex force-allocation QP. MuJoCo still computes actual contacts. */
export function allocate(feet: number[][], com: number[], wrench: number[], supporting: boolean[]) {
  const ids = supporting.flatMap((v,i) => v ? [i] : []), n=ids.length*3;
  const A = Array.from({length:6},() => Array(n).fill(0) as number[]);
  ids.forEach((id,i) => {
    const [x,y,z]=feet[id].map((v,j) => v-com[j]);
    const block = [[1,0,0],[0,1,0],[0,0,1],[0,-z,y],[z,0,-x],[-y,x,0]];
    block.forEach((row,r) => row.forEach((v,j) => A[r][3*i+j]=v));
  });
  const weights=[1,1,1,16,16,4];
  // quadprog uses 1-based arrays and min(1/2 f'Hf - d'f), A'f >= b.
  const H=Array.from({length:n+1},() => Array(n+1).fill(0));
  const d=Array(n+1).fill(0);
  for (let i=0;i<n;i++) {
    for (let r=0;r<6;r++) d[i+1]+=A[r][i]*weights[r]*wrench[r];
    for (let j=0;j<n;j++) H[i+1][j+1]=A.reduce((sum,row,r) => sum+row[i]*weights[r]*row[j],i===j?.002:0);
  }
  const constraints = Array.from({length:n+1},() => Array(ids.length*6+1).fill(0));
  const b=Array(ids.length*6+1).fill(0);
  ids.forEach((_,i) => {
    [[-1,-1,.55],[-1,1,.55],[1,-1,.55],[1,1,.55],[0,0,1],[0,0,-1]].forEach((row,c) => {
      row.forEach((v,j) => constraints[3*i+j+1][6*i+c+1]=v);
    });
    b[6*i+6]=-180;
  });
  const result=solveQP(H,d,constraints,b);
  const ok=!result.message && result.solution?.slice(1).every(Number.isFinite);
  const forces=Array.from({length:4},() => [0,0,0]);
  ids.forEach((id,i) => forces[id]=ok ? result.solution.slice(3*i+1,3*i+4) : [0,0,clamp(wrench[2]/ids.length,0,180)]);
  return {forces,ok};
}

/** The browser port of the Python crawl controller; state changes only by mj_step. */
export class BrowserRobot {
  model: MjModel; data: MjData; trunk: number; sites: number[]; footIds: number[];
  jac: DoubleBuffer; comJac: DoubleBuffer; contactForce: DoubleBuffer;
  mass: number; tick=0; mode='manual'; paused=false; view: 'follow'|'overview'='follow';
  manual=[0,0,0]; path: Point[]=[]; waypoint=0; goal: Point|null=null; trail: Point[]=[];
  message='Ready. Drive with W/A/S/D or choose a destination.';
  targetXY:number[]=[]; holdXY:number[]=[]; startXY:number[]=[]; targetYaw=0; stepYaw=0;
  stepStart:number|null=null; swingLeg=0; swingStart:number[]=[]; swingEnd:number[]=[];
  supporting=[true,true,true,true]; phase='Standing'; steps=0; failures=0; fallen=false;
  forces=Array.from({length:4},() => [0,0,0]); collisionSamples=0;

  constructor(public mj: MainModule, xml: string, public arena: Arena) {
    this.model=mj.MjModel.from_xml_string(xml); this.data=new mj.MjData(this.model);
    this.trunk=mj.mj_name2id(this.model,1,'trunk');
    this.sites=LEGS.map(l => mj.mj_name2id(this.model,6,`${l}_toe`));
    this.footIds=LEGS.map(l => mj.mj_name2id(this.model,5,`${l}_foot`));
    this.jac=new mj.DoubleBuffer(3*this.model.nv); this.comJac=new mj.DoubleBuffer(3*this.model.nv);
    this.contactForce=new mj.DoubleBuffer(6);
    this.mass=Array.from(this.model.body_mass as Float64Array).reduce((a,b) => a+b,0);
    this.reset();
  }
  feet() { return this.sites.map(id => Array.from(this.data.site_xpos.slice(3*id,3*id+3)) as number[]); }
  com() { return Array.from(this.data.subtree_com.slice(3*this.trunk,3*this.trunk+3)) as number[]; }
  R() { return Array.from(this.data.xmat.slice(9*this.trunk,9*this.trunk+9)) as number[]; }
  reset() {
    this.mj.mj_resetData(this.model,this.data);
    const bend=Math.acos((.435-.026)/.5);
    for (let i=0;i<4;i++) this.data.qpos.set([0,-bend,2*bend],7+3*i);
    this.mj.mj_forward(this.model,this.data);
    this.targetXY=this.holdXY=this.startXY=this.com().slice(0,2);
    this.tick=0;this.mode='manual';this.paused=false;this.manual=[0,0,0];this.path=[];this.goal=null;this.trail=[];
    this.targetYaw=0;this.stepYaw=0;this.stepStart=null;this.swingLeg=0;this.steps=0;this.failures=0;this.fallen=false;this.collisionSamples=0;
    this.supporting=[true,true,true,true];this.phase='Standing';
    this.message='Ready. Drive with W/A/S/D or choose a destination.';
  }
  stop(message='Stopping — placing the swing foot before holding.') { this.manual=[0,0,0];this.path=[];this.goal=null;this.mode='manual';this.message=message; }
  command(event: Command) {
    switch (event.action) {
      case 'drive':
        this.manual=[event.forward ?? 0,event.strafe ?? 0,event.turn ?? 0].map(x => Number.isFinite(x)?clamp(x,-1,1):0);
        this.mode='manual';this.path=[];this.goal=null;
        if (this.manual.some(x=>x!==0)) this.message='Manual drive · release the controls to hold position.';
        break;
      case 'goal': {
        const center=mean(this.feet()).slice(0,2) as Point;
        try { this.path=planPath(center,event.target!,this.arena); }
        catch (e) { this.message=(e as Error).message; return; }
        this.waypoint=1;this.goal=event.target!;this.manual=[0,0,0];this.mode='navigate';
        this.message='Following a route through the known obstacle map.';break;
      }
      case 'stop': this.stop();break;
      case 'pause': this.paused=!!event.paused;this.manual=[0,0,0];this.message=this.paused?'Physics paused.':'Physics running.';break;
      case 'reset': this.reset();break;
      case 'camera': this.view=event.view ?? 'follow';break;
    }
  }
  navigation() {
    const R=this.R(),yaw=Math.atan2(R[3],R[0]),center=mean(this.feet());
    let command: number[];
    if (this.mode==='navigate' && this.path.length) {
      let target=this.path[this.waypoint],dist=distance(target,center);
      if (dist<.1) {
        if (this.waypoint===this.path.length-1) { this.mode='arrived';this.message='Destination reached. Finishing the step and holding.';return [0,0,0]; }
        target=this.path[++this.waypoint];dist=distance(target,center);
      }
      const delta=[target[0]-center[0],target[1]-center[1]],desired=Math.atan2(delta[1],delta[0]);
      const error=Math.atan2(Math.sin(desired-yaw),Math.cos(desired-yaw)),speed=Math.min(.05,dist/(4*PERIOD));
      command=[delta[0]/dist*speed,delta[1]/dist*speed,clamp(.4*error,-.06,.06)];
    } else if (this.mode==='arrived') return [0,0,0];
    else {
      const norm=Math.max(1,Math.hypot(...this.manual.slice(0,2))),v=rotate(this.manual.map(x=>x/norm),yaw);
      command=[v[0]*SPEED,v[1]*SPEED,this.manual[2]*TURN];
      if (Math.abs(command[2])>.02) {command[0]*=.75;command[1]*=.75;}
    }
    if (Math.hypot(command[0],command[1])>0 && !segmentClear(center,[center[0]+command[0]*4*PERIOD,center[1]+command[1]*4*PERIOD],this.arena,.55)) {
      command[0]=command[1]=0;this.message='Clearance stop. Steer away from the obstacle or wall.';
    }
    return command;
  }
  beginStep(command: number[]) {
    this.swingLeg=[0,3,1,2][this.steps%4];
    const feet=this.feet().map(p=>[p[0],p[1],.026]);
    this.startXY=[...this.targetXY];this.holdXY=mean(feet.filter((_,i)=>i!==this.swingLeg)).slice(0,2);
    this.swingStart=[...feet[this.swingLeg]];
    const center=mean(feet),offset=feet[this.swingLeg].map((v,j)=>v-center[j]);
    this.swingEnd=rotate(offset,command[2]*4*PERIOD).map((v,j)=>v+center[j]+(j<2?command[j]*4*PERIOD:0));
    this.swingEnd[2]=.026;this.stepYaw=this.targetYaw+command[2]*PERIOD;this.stepStart=this.data.time;
  }
  control() {
    const d=this.data,m=this.model,R=this.R(),command=this.navigation();
    if (d.qpos[2]<.22 || R[8]<.65 || !Array.from(d.qpos as Float64Array).every(Number.isFinite)) this.fallen=true;
    if (this.fallen) {
      this.mode='fallen';this.phase='Fallen — reset required';this.message='Robot lost balance. Reset to try again.';
      for (let j=0;j<12;j++) d.ctrl[j]=clamp(-2*d.qvel[6+j],-32,32);return;
    }
    const moving=Math.hypot(...command)>.001 && d.time>1;
    if (this.stepStart===null && moving) this.beginStep(command);
    let desiredFoot:number[]|null=null,footVelocity=[0,0,0];this.supporting.fill(true);
    if (this.stepStart!==null) {
      const elapsed=d.time-this.stepStart;
      this.targetXY=this.startXY.map((v,j)=>v+smooth(elapsed/SHIFT)*(this.holdXY[j]-v));
      this.targetYaw+=clamp(this.stepYaw-this.targetYaw,-.001,.001);this.phase=`Shift support · ${LEGS[this.swingLeg]}`;
      if (elapsed>=SHIFT && elapsed<SHIFT+SWING) {
        const u=(elapsed-SHIFT)/SWING;this.supporting[this.swingLeg]=false;
        desiredFoot=this.swingStart.map((v,j)=>v+smooth(u)*(this.swingEnd[j]-v));
        desiredFoot[2]+=.075*Math.sin(Math.PI*u)**2;
        footVelocity=this.swingEnd.map((v,j)=>(6*u-6*u*u)/SWING*(v-this.swingStart[j]));
        footVelocity[2]+=.075*Math.PI*Math.sin(2*Math.PI*u)/SWING;this.phase=`Swing · ${LEGS[this.swingLeg]}`;
      } else if (elapsed>=SHIFT+SWING) this.phase='Touchdown';
      if (elapsed>=PERIOD) {
        this.steps++;this.stepStart=null;
        if (moving) this.beginStep(command);
        else {this.startXY=[...this.targetXY];this.holdXY=mean(this.feet()).slice(0,2);}
      }
    } else { this.targetXY=this.targetXY.map((v,j)=>v+clamp(this.holdXY[j]-v,-.0005,.0005));this.phase='Standing'; }
    const com=this.com();this.mj.mj_jacSubtreeCom(m,d,this.comJac,this.trunk);
    const jacCom=this.comJac.GetView() as Float64Array;
    const vel=[0,1,2].map(r=>Array.from(d.qvel as Float64Array).reduce((sum,v,j)=>sum+jacCom[r*m.nv+j]*v,0));
    const err=[this.targetXY[0]-com[0],this.targetXY[1]-com[1],.395-d.qpos[2]];
    const force=err.map((v,j)=>this.mass*([100,100,180][j]*v-[20,20,27][j]*vel[j]+(j===2?9.81:0)));
    const cy=Math.cos(this.targetYaw),sy=Math.sin(this.targetYaw),desiredR=[cy,-sy,0,sy,cy,0,0,0,1];
    const E=Array.from({length:9},(_,i)=>[0,1,2].reduce((sum,k)=>sum+desiredR[Math.floor(i/3)*3+k]*R[(i%3)*3+k],0));
    const rotErr=[(E[7]-E[5])/2,(E[2]-E[6])/2,(E[3]-E[1])/2];
    const omega=[0,1,2].map(r=>[0,1,2].reduce((sum,j)=>sum+R[r*3+j]*d.qvel[3+j],0));
    const moment=rotErr.map((v,j)=>[200,260,150][j]*v-[20,24,16][j]*omega[j]);
    const feet=this.feet(),allocation=allocate(feet,com,[...force,...moment],this.supporting);
    this.forces=allocation.forces;if (!allocation.ok) this.failures++;
    const torque=Array.from({length:12},(_,j)=>d.qfrc_bias[6+j]-.8*d.qvel[6+j]);
    this.sites.forEach((site,i)=>{
      this.mj.mj_jacSite(m,d,this.jac,null,site);const J=this.jac.GetView() as Float64Array;
      let swingForce=[0,0,0];
      if (i===this.swingLeg && desiredFoot) swingForce=desiredFoot.map((v,r)=>700*(v-feet[i][r])+18*(footVelocity[r]-Array.from(d.qvel as Float64Array).reduce((sum,q,j)=>sum+J[r*m.nv+j]*q,0)));
      for (let j=0;j<12;j++) for (let r=0;r<3;r++) torque[j]+=J[r*m.nv+6+j]*(swingForce[r]-this.forces[i][r]);
    });
    torque.forEach((v,j)=>d.ctrl[j]=clamp(v,-32,32));
  }
  advance(steps: number) {
    if (this.paused) return;
    for (let i=0;i<steps;i++) {
      if (this.tick%5===0) this.control();
      this.mj.mj_step(this.model,this.data);this.tick++;
      if (this.tick%50===0) {this.trail.push([this.data.qpos[0],this.data.qpos[1]]);if(this.trail.length>700)this.trail.shift();}
    }
    this.mj.mj_forward(this.model,this.data);
  }
  snapshot() {
    const d=this.data,R=this.R(),loads=[0,0,0,0];
    const contacts=d.contact;
    try {
      for (let i=0;i<d.ncon;i++) {
        const c=contacts.get(i);if(!c)continue;
        try {
          const foot=this.footIds.findIndex(id=>id===c.geom1 || id===c.geom2);
          if(foot>=0){this.mj.mj_contactForce(this.model,d,i,this.contactForce);loads[foot]+=Math.max(0,this.contactForce.GetView()[0]);}
          // Arena crates are the first three world-body geometries in the exported model.
          if(c.geom1<3 || c.geom2<3)this.collisionSamples++;
        } finally {c.delete();}
      }
    } finally {contacts.delete();}
    return {
      ...this.arena,ready:true,time:d.time,position:Array.from(d.qpos.slice(0,3)) as number[],center:mean(this.feet()).slice(0,2) as Point,
      yaw:Math.atan2(R[3],R[0]),roll:Math.atan2(R[7],R[8])*180/Math.PI,pitch:Math.asin(clamp(-R[6],-1,1))*180/Math.PI,
      speed:Math.hypot(d.qvel[0],d.qvel[1]),torque:Math.max(...Array.from(d.ctrl as Float64Array).map(Math.abs)),loads,
      supporting:[...this.supporting],phase:this.phase,mode:this.mode,paused:this.paused,message:this.message,
      path:this.path,waypoint:this.waypoint,goal:this.goal,trail:[...this.trail],steps:this.steps,solverFailures:this.failures,
      collisionSamples:this.collisionSamples,camera:this.view,realTimeFactor:1,fps:60,
    };
  }
  dispose() {this.jac.delete();this.comJac.delete();this.contactForce.delete();this.data.delete();this.model.delete();}
}

export type RobotState = ReturnType<BrowserRobot['snapshot']>;
