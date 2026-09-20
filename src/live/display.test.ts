import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import type { BrowserRobot } from './physics';

const mocks=vi.hoisted(()=>({fail:false,gpu:vi.fn(),software:vi.fn(),disposeGpu:vi.fn(),disposeSoftware:vi.fn()}));
vi.mock('./renderer',()=>({RobotRenderer:class {
  constructor(...args:unknown[]){mocks.gpu(...args);if(mocks.fail)throw new Error('Error creating WebGL context.');}
  render(){} dispose(){mocks.disposeGpu();}
}}));
vi.mock('./softwareRenderer',()=>({SoftwareRobotRenderer:class {
  constructor(...args:unknown[]){mocks.software(...args);}
  render(){} dispose(){mocks.disposeSoftware();}
}}));
import { createRobotDisplay } from './display';

describe('graphics availability does not block robot control',()=>{
  let host:HTMLElement,canvases:EventTarget[];
  const robot={} as BrowserRobot;
  beforeEach(()=>{
    vi.clearAllMocks();mocks.fail=false;canvases=[];
    host={replaceChildren:vi.fn()} as unknown as HTMLElement;
    vi.stubGlobal('document',{createElement:()=>{const canvas=Object.assign(new EventTarget(),{style:{}});canvases.push(canvas);return canvas;}});
  });
  afterEach(()=>vi.unstubAllGlobals());
  it('falls back on startup failure using a fresh canvas and the same physics instance',()=>{
    mocks.fail=true;const mode=vi.fn(),display=createRobotDisplay(host,robot,mode);
    expect(mode).toHaveBeenCalledWith('compatibility');expect(canvases).toHaveLength(2);
    expect(mocks.software).toHaveBeenCalledWith(canvases[1],robot);
    expect(()=>display.render()).not.toThrow();display.dispose();expect(mocks.disposeSoftware).toHaveBeenCalledOnce();
  });
  it('switches after GPU context loss without resetting physics',()=>{
    const mode=vi.fn(),display=createRobotDisplay(host,robot,mode);
    expect(mode).toHaveBeenCalledWith('3d');
    canvases[0].dispatchEvent(new Event('webglcontextlost',{cancelable:true}));
    expect(mode).toHaveBeenLastCalledWith('compatibility');expect(mocks.disposeGpu).toHaveBeenCalledOnce();
    expect(mocks.software).toHaveBeenCalledWith(canvases[1],robot);display.dispose();
  });
  it('can explicitly start without requesting WebGL',()=>{
    const display=createRobotDisplay(host,robot,vi.fn(),true);
    expect(mocks.gpu).not.toHaveBeenCalled();expect(mocks.software).toHaveBeenCalledOnce();display.dispose();
  });
  it('ignores a delayed context-loss event after unmount',()=>{
    const display=createRobotDisplay(host,robot,vi.fn());display.dispose();
    canvases[0].dispatchEvent(new Event('webglcontextlost'));
    expect(mocks.software).not.toHaveBeenCalled();
  });
});
