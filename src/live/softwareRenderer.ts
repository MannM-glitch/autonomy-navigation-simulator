import type { BrowserRobot } from './physics';

/** Canvas2D projection of the same MuJoCo geometry; no GPU context required. */
export class SoftwareRobotRenderer {
  ctx: CanvasRenderingContext2D;
  observer: ResizeObserver;
  width=1; height=1; scale=1; center=[0,0];
  constructor(public canvas: HTMLCanvasElement, public robot: BrowserRobot) {
    const ctx=canvas.getContext('2d');
    if(!ctx)throw new Error('This browser could not create a drawing surface. Try opening the link in another browser.');
    this.ctx=ctx;
    this.observer=new ResizeObserver(()=>this.resize());
    this.observer.observe(canvas.parentElement!);this.resize();
  }
  resize() {
    const box=this.canvas.parentElement!.getBoundingClientRect(),ratio=Math.min(devicePixelRatio,2);
    this.width=Math.max(1,box.width);this.height=Math.max(1,box.height);
    this.canvas.width=Math.round(this.width*ratio);this.canvas.height=Math.round(this.height*ratio);
    this.ctx.setTransform(ratio,0,0,ratio,0,0);
  }
  project(p: number[]) {
    const x=p[0]-this.center[0],y=p[1]-this.center[1];
    return [this.width/2+(x+y)*.7071*this.scale,this.height*.58+(x-y)*.4082*this.scale-(p[2]??0)*.8165*this.scale];
  }
  polygon(points: number[][],color: string,stroke='#49615b') {
    const c=this.ctx;c.beginPath();
    points.forEach((p,i)=>{const [x,y]=this.project(p);if(i)c.lineTo(x,y);else c.moveTo(x,y);});
    c.closePath();c.fillStyle=color;c.fill();c.strokeStyle=stroke;c.lineWidth=.6;c.stroke();
  }
  line(points: number[][],color: string,width=1) {
    if(!points.length)return;
    const c=this.ctx;c.beginPath();points.forEach((p,i)=>{const [x,y]=this.project(p);if(i)c.lineTo(x,y);else c.moveTo(x,y);});
    c.strokeStyle=color;c.lineWidth=width;c.lineCap='round';c.stroke();
  }
  render() {
    const c=this.ctx,d=this.robot.data,m=this.robot.model,arena=this.robot.arena.arena;
    const overview=this.robot.view==='overview';
    this.center=overview?[0,0]:[d.qpos[0],d.qpos[1]];
    this.scale=overview?Math.min(this.width/(arena*3.3),this.height/(arena*2.5)):Math.min(this.width/3.3,this.height/2.1);
    c.fillStyle='#101c20';c.fillRect(0,0,this.width,this.height);
    const n=12,cell=2*arena/n;
    for(let x=0;x<n;x++)for(let y=0;y<n;y++) {
      const a=-arena+x*cell,b=-arena+y*cell;
      this.polygon([[a,b,0],[a+cell,b,0],[a+cell,b+cell,0],[a,b+cell,0]],(x+y)%2?'#263d3f':'#2d4545','#314b49');
    }
    this.line(this.robot.trail.map(p=>[...p,.01]),'#65a4ba',1.2);
    c.setLineDash([5,4]);this.line(this.robot.path.map(p=>[...p,.015]),'#adebad',1.5);c.setLineDash([]);
    if(this.robot.goal){const [x,y]=this.project([...this.robot.goal,.02]);c.beginPath();c.ellipse(x,y,9,5,0,0,Math.PI*2);c.strokeStyle='#b8f4b9';c.lineWidth=2;c.stroke();}
    const drawables: {depth:number;draw:()=>void}[]=[];
    for(let i=0;i<m.ngeom;i++) {
      const type=m.geom_type[i];if(type===0)continue;
      const pos=Array.from(d.geom_xpos.slice(3*i,3*i+3)) as number[];
      const size=Array.from(m.geom_size.slice(3*i,3*i+3)) as number[];
      const r=d.geom_xmat.subarray(9*i,9*i+9);
      const world=(p:number[])=>[0,1,2].map(j=>pos[j]+r[j*3]*p[0]+r[j*3+1]*p[1]+r[j*3+2]*p[2]);
      const rgb=Array.from(m.geom_rgba.slice(4*i,4*i+3)) as number[];
      const color=(shade:number)=>`rgb(${rgb.map(v=>Math.round(Math.min(255,(v*205+35)*shade))).join(',')})`;
      const depth=(p:number[])=>p[0]-p[1]+p[2];
      if(type===6) {
        const [x,y,z]=size;
        // Three faces visible from the fixed isometric camera, selected in world space.
        const faces=[[[x,-y,-z],[x,y,-z],[x,y,z],[x,-y,z]],[[-x,y,-z],[-x,-y,-z],[-x,-y,z],[-x,y,z]],
          [[-x,y,-z],[x,y,-z],[x,y,z],[-x,y,z]],[[x,-y,-z],[-x,-y,-z],[-x,-y,z],[x,-y,z]],
          [[-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]],[[-x,y,-z],[x,y,-z],[x,-y,-z],[-x,-y,-z]]];
        const normals=[[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]];
        faces.forEach((face,f)=>{
          const normal=normals[f],nw=[0,1,2].map(j=>r[j*3]*normal[0]+r[j*3+1]*normal[1]+r[j*3+2]*normal[2]);
          if(nw[0]-nw[1]+nw[2]<=0)return;
          const vertices=face.map(world),mid=[0,1,2].map(j=>vertices.reduce((s,p)=>s+p[j],0)/4);
          drawables.push({depth:depth(mid),draw:()=>this.polygon(vertices,color(.72+.25*Math.max(0,nw[2])),'#63766b')});
        });
      } else {
        drawables.push({depth:depth(pos),draw:()=>{
          if(type===3 || type===5){this.line([world([0,0,-size[1]]),world([0,0,size[1]])],color(1),Math.max(2,size[0]*2*this.scale));}
          else {const [x,y]=this.project(pos);c.beginPath();c.arc(x,y,Math.max(2,size[0]*this.scale),0,Math.PI*2);c.fillStyle=color(1);c.fill();}
        }});
      }
    }
    drawables.sort((a,b)=>a.depth-b.depth).forEach(item=>item.draw());
  }
  dispose() {this.observer.disconnect();}
}
