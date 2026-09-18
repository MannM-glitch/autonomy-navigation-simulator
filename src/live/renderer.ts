import * as THREE from 'three';
import type { BrowserRobot } from './physics';

/** Render MuJoCo's evaluated geom transforms; never animate robot poses here. */
export class RobotRenderer {
  renderer: THREE.WebGLRenderer;
  scene=new THREE.Scene();
  camera=new THREE.PerspectiveCamera(43,16/9,.02,40);
  meshes: THREE.Mesh[]=[];
  path=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x97e8ad}));
  goal: THREE.Mesh;
  observer: ResizeObserver;
  routeKey='';
  floorTexture: THREE.CanvasTexture;
  matrix=new THREE.Matrix4();

  constructor(canvas: HTMLCanvasElement, public robot: BrowserRobot) {
    this.renderer=new THREE.WebGLRenderer({canvas,antialias:true,powerPreference:'high-performance'});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio,1.7));
    this.renderer.shadowMap.enabled=true;this.renderer.shadowMap.type=THREE.PCFShadowMap;
    this.renderer.toneMapping=THREE.ACESFilmicToneMapping;this.renderer.toneMappingExposure=1.3;
    this.scene.background=new THREE.Color('#101c20');this.scene.fog=new THREE.Fog('#101c20',7,18);
    this.camera.up.set(0,0,1);
    this.scene.add(new THREE.HemisphereLight(0xe1f5ff,0x354537,2.2));
    const light=new THREE.DirectionalLight(0xffefd8,3.1);light.position.set(1,-3,6);light.castShadow=true;
    light.shadow.mapSize.set(2048,2048);light.shadow.camera.left=-4;light.shadow.camera.right=4;
    light.shadow.camera.top=4;light.shadow.camera.bottom=-4;light.shadow.normalBias=.012;
    this.scene.add(light);this.scene.add(light.target);
    const fill=new THREE.DirectionalLight(0xbfe1ff,.8);fill.position.set(-3,2,3);this.scene.add(fill);
    const textureCanvas=document.createElement('canvas');textureCanvas.width=textureCanvas.height=256;
    const ctx=textureCanvas.getContext('2d')!;
    ctx.fillStyle='#283d40';ctx.fillRect(0,0,256,256);ctx.fillStyle='#30464a';ctx.fillRect(0,0,128,128);ctx.fillRect(128,128,128,128);
    this.floorTexture=new THREE.CanvasTexture(textureCanvas);this.floorTexture.wrapS=this.floorTexture.wrapT=THREE.RepeatWrapping;this.floorTexture.repeat.set(15,15);this.floorTexture.colorSpace=THREE.SRGBColorSpace;
    const model=robot.model;
    for(let i=0;i<model.ngeom;i++) {
      const size=Array.from(model.geom_size.slice(3*i,3*i+3)) as number[],type=model.geom_type[i];
      let geometry: THREE.BufferGeometry;
      if(type===0)geometry=new THREE.PlaneGeometry(14,14);
      else if(type===2)geometry=new THREE.SphereGeometry(size[0],20,14);
      else if(type===3){geometry=new THREE.CapsuleGeometry(size[0],2*size[1],6,14);geometry.rotateX(Math.PI/2);}
      else if(type===5){geometry=new THREE.CylinderGeometry(size[0],size[0],2*size[1],24);geometry.rotateX(Math.PI/2);}
      else geometry=new THREE.BoxGeometry(2*size[0],2*size[1],2*size[2]);
      const rgb=model.geom_rgba.slice(4*i,4*i+4);
      const material=new THREE.MeshStandardMaterial({color:new THREE.Color().setRGB(rgb[0],rgb[1],rgb[2]),roughness:.62,metalness:.12});
      if(type===0){material.map=this.floorTexture;material.color.set(0xffffff);material.roughness=.95;material.metalness=0;}
      const mesh=new THREE.Mesh(geometry,material);mesh.castShadow=type!==0;mesh.receiveShadow=true;
      this.meshes.push(mesh);this.scene.add(mesh);
    }
    this.scene.add(this.path);
    this.goal=new THREE.Mesh(new THREE.TorusGeometry(.09,.009,8,32),new THREE.MeshBasicMaterial({color:0xadebad}));
    this.scene.add(this.goal);
    this.observer=new ResizeObserver(()=>this.resize(canvas));this.observer.observe(canvas.parentElement!);this.resize(canvas);
  }
  resize(canvas: HTMLCanvasElement) {
    const {width,height}=canvas.parentElement!.getBoundingClientRect();
    if(!width || !height)return;
    this.renderer.setSize(width,height,false);this.camera.aspect=width/height;this.camera.updateProjectionMatrix();
  }
  render() {
    const d=this.robot.data;
    this.meshes.forEach((mesh,i)=>{
      mesh.position.fromArray(d.geom_xpos,3*i);
      const r=d.geom_xmat.subarray(9*i,9*i+9);
      this.matrix.set(r[0],r[1],r[2],0,r[3],r[4],r[5],0,r[6],r[7],r[8],0,0,0,0,1);
      mesh.quaternion.setFromRotationMatrix(this.matrix);
    });
    if(this.robot.view==='overview'){this.camera.position.set(3.6,-4.1,5.3);this.camera.lookAt(0,0,0);}
    else {this.camera.position.set(d.qpos[0]+1.6,d.qpos[1]-1.6,1.35);this.camera.lookAt(d.qpos[0],d.qpos[1],.22);}
    const key=JSON.stringify(this.robot.path);
    if(key!==this.routeKey){this.routeKey=key;this.path.geometry.dispose();this.path.geometry=new THREE.BufferGeometry().setFromPoints(this.robot.path.map(p=>new THREE.Vector3(p[0],p[1],.025)));}
    this.goal.visible=!!this.robot.goal;
    if(this.robot.goal)this.goal.position.set(...this.robot.goal,.027);
    this.renderer.render(this.scene,this.camera);
  }
  dispose() {
    this.observer.disconnect();
    this.scene.traverse(object=>{if(object instanceof THREE.Mesh || object instanceof THREE.Line){object.geometry.dispose();const material=object.material;for(const m of Array.isArray(material)?material:[material])m.dispose();}});
    this.floorTexture.dispose();this.renderer.dispose();this.renderer.forceContextLoss();
  }
}
