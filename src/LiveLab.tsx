import { useBrowserRobot } from './live/useBrowserRobot';
import type { RobotState as State, Command } from './live/physics';
import type { Point } from './live/navigation';
import { ArrowDown, ArrowLeft, ArrowRight, ArrowUp, ArrowUpRight, CirclePause, CirclePlay, Crosshair, Expand, Keyboard, RotateCcw, ScanLine, Square, Terminal, Wifi, WifiOff } from 'lucide-react';
import './live.css';

const short = (value: number | undefined, digits = 2) => value === undefined ? '—' : value.toFixed(digits);

function NavigationMap({ state, enabled, onGoal }: { state: State | null; enabled: boolean; onGoal: (point: Point) => void }) {
  const arena = state?.arena ?? 2.4;
  const p = (x: number) => (x+arena)/(2*arena)*320;
  const y = (v: number) => 320-p(v);
  const length = (v: number) => v/(2*arena)*320;
  const points = (list: Point[]) => list.map(pt => `${p(pt[0])},${y(pt[1])}`).join(' ');
  return <svg className={`live-map ${enabled ? '' : 'disabled'}`} viewBox="0 0 320 320" role="img" aria-label="Top-down navigation map. Click clear floor to choose a destination; station buttons below provide keyboard-accessible destinations."
    onClick={event => {
      if (!enabled) return;
      const box = event.currentTarget.getBoundingClientRect();
      onGoal([(event.clientX-box.left)/box.width*2*arena-arena, arena-(event.clientY-box.top)/box.height*2*arena]);
    }}>
    <defs><pattern id="live-grid" width="32" height="32" patternUnits="userSpaceOnUse"><path d="M32 0H0V32" fill="none" stroke="#263b38" strokeWidth=".6"/></pattern></defs>
    <rect width="320" height="320" fill="#101e1d"/><rect width="320" height="320" fill="url(#live-grid)"/>
    <rect x="2" y="2" width="316" height="316" rx="4" fill="none" stroke="#58746a" strokeWidth="3"/>
    <rect x={length(state?.clearance ?? .65)} y={length(state?.clearance ?? .65)} width={320-2*length(state?.clearance ?? .65)} height={320-2*length(state?.clearance ?? .65)} rx="2" fill="none" stroke="#546d5d" strokeDasharray="3 4" opacity=".65"/>
    {state?.obstacles.map((o,i) => <g key={i}>
      <rect x={p(o.x-o.hx-state.clearance)} y={y(o.y+o.hy+state.clearance)} width={length(2*(o.hx+state.clearance))} height={length(2*(o.hy+state.clearance))} rx={length(state.clearance)} fill="#ac875918" stroke="#ac875940" strokeWidth=".7"/>
      <rect x={p(o.x-o.hx)} y={y(o.y+o.hy)} width={length(o.hx*2)} height={length(o.hy*2)} rx="2" fill="#52716c" stroke="#8aa397"/>
    </g>)}
    {state && <>
      <polyline points={points(state.trail)} fill="none" stroke="#65a4ba" strokeWidth="1.5" opacity=".6"/>
      <polyline points={points(state.path)} fill="none" stroke="#adebad" strokeWidth="1.8" strokeDasharray="4 3"/>
      {state.goal && <g><circle cx={p(state.goal[0])} cy={y(state.goal[1])} r="8" fill="none" stroke="#b1f5b8"/><circle cx={p(state.goal[0])} cy={y(state.goal[1])} r="2" fill="#b1f5b8"/></g>}
      <g transform={`translate(${p(state.position[0])},${y(state.position[1])}) rotate(${-state.yaw*180/Math.PI})`}>
        <rect x="-12" y="-10" width="24" height="20" rx="5" fill="#f1ba60" stroke="#f9d58a"/>
        <path d="M4 -5L12 0 4 5Z" fill="#293a30"/>
      </g>
    </>}
    <text x="12" y="19" className="map-axis">+Y</text><text x="292" y="307" className="map-axis">+X</text>
  </svg>;
}

export default function LiveLab() {
  const { state: s, connected, held, error, send, release, setKey, canvasRef } = useBrowserRobot();
  
  const available = connected && !s?.paused && s?.mode !== 'fallen';
  function goal(point: Point) { release(false); void send({action:'goal',target:point}); }
  function command(event: Command) { release(false); void send(event); }
  function driveButton(key: string, label: string, icon: React.ReactNode) {
    return <button className={`drive-key ${held.has(key) ? 'held' : ''}`} disabled={!available} aria-label={label} title={label}
      onPointerDown={e => { e.preventDefault(); e.currentTarget.setPointerCapture(e.pointerId); setKey(key,true); }}
      onPointerUp={() => setKey(key,false)} onPointerCancel={() => setKey(key,false)} onLostPointerCapture={() => { if (held.has(key)) setKey(key,false); }}
      onKeyDown={e => { if (e.key === 'Enter') setKey(key,true); }} onKeyUp={e => { if (e.key === 'Enter') setKey(key,false); }}>
      {icon}<span>{key.toUpperCase()}</span>
    </button>;
  }
  return <div className="sentry-app live-app">
    <header className="site-header"><a className="brand" href="#live"><span className="brand-mark"><ScanLine size={22}/></span>SENTRY<span className="brand-tag">LIVE ROBOTICS LAB</span></a><nav aria-label="Lab mode"><a className="nav-active" href="#live">Live control</a><a href="#replay">Recorded experiment</a></nav><a className="source-link" href="https://github.com/MannM-glitch/autonomy-navigation-simulator/blob/main/simulation/LIVE.md" target="_blank" rel="noreferrer">How it works <ArrowUpRight size={15}/></a></header>
    <main>
      <section className="live-hero"><div><div className="eyebrow"><span className="status-dot"/> TELEOPERATION + POINT-TO-POINT NAVIGATION</div><h1>Your robot. Your next move.</h1><p>Drive a real MuJoCo quadruped, or give it a destination. Every step is simulated live.</p></div><div className={`connection-pill ${connected ? 'online' : ''}`}>{connected ? <Wifi size={14}/> : <WifiOff size={14}/>} {connected ? s?.paused ? 'Physics paused' : 'Browser physics running' : error ? 'Load failed' : 'Loading physics'}</div></section>
      <div className="live-summary"><div><span>SIMULATION TIME</span><strong>{short(s?.time,1)}<small>s</small></strong></div><div><span>POSITION / METRES</span><strong>{short(s?.position[0])}<small>x</small> {short(s?.position[1])}<small>y</small></strong></div><div><span>COMPLETED STEPS</span><strong>{s?.steps ?? '—'}</strong></div><div><span>REAL-TIME FACTOR</span><strong>{connected ? short(s?.realTimeFactor) : '—'}<small>×</small><em>{connected ? `${short(s?.fps,0)} fps` : 'Loading engine'}</em></strong></div></div>
      <div className="live-workspace">
        <section className="live-scene-panel">
          <div className="panel-toolbar"><div className="panel-title"><span className="tiny-index">01</span> Live physics <span className="live-tag">{s?.mode === 'navigate' ? 'AUTONOMOUS' : 'USER CONTROL'}</span></div><div className="camera-toggle"><button disabled={!connected} aria-pressed={s?.camera === 'follow'} onClick={() => void send({action:'camera',view:'follow'})}><Crosshair size={13}/> Follow</button><button disabled={!connected} aria-pressed={s?.camera === 'overview'} onClick={() => void send({action:'camera',view:'overview'})}><Expand size={13}/> Arena</button></div></div>
          <div className="live-scene">
            <canvas ref={canvasRef} className="live-canvas" aria-label="Live MuJoCo quadruped in an obstacle arena"/>
            {connected && <><div className="scene-caption"><span className="status-dot"/>{s?.paused ? 'PAUSED' : 'LIVE · TORQUE-DRIVEN CRAWL'}</div><div className="scene-time">{short(s?.time,2)} s</div><div className="phase-badge"><span className="phase-kicker">GAIT PHASE</span><strong>{s?.phase}</strong></div><span className="live-render-note">{s?.camera === 'follow' ? 'Tracking camera' : 'Arena overview'}</span></>}
            {!connected && <div className="live-offline"><Terminal size={26}/><h2>{error ? 'Physics could not start.' : 'Loading real robot physics…'}</h2><p>{error || 'Downloading MuJoCo and preparing the robot. The simulation runs entirely in your browser.'}</p>{error && <button className="launch-command" onClick={() => location.reload()}>Reload physics</button>}<p className="offline-help">No Python server or account required. <a href="#replay">Watch the recorded experiment →</a></p></div>}
            {connected && s?.mode === 'fallen' && <div className="fallen-overlay"><strong>Balance lost</strong><span>Reset the robot to continue.</span><button onClick={() => command({action:'reset'})}><RotateCcw size={14}/> Reset robot</button></div>}
          </div>
          <div className="live-action-bar"><button className="live-stop" disabled={!connected} onClick={() => release()}><Square size={13} fill="currentColor"/> Stop <kbd>SPACE</kbd></button><button disabled={!connected} onClick={() => command({action:'pause',paused:!s?.paused})}>{s?.paused ? <CirclePlay size={16}/> : <CirclePause size={16}/>} {s?.paused ? 'Resume physics' : 'Pause physics'}</button><button disabled={!connected} onClick={() => command({action:'reset'})}><RotateCcw size={15}/> Reset robot</button></div>
          <div className="live-message" role="status"><span className={`status-dot ${error ? 'amber' : ''}`}/>{error || (connected ? s?.message : 'Loading MuJoCo in this browser…')}</div>
          <div className="live-control-deck"><div className="keyboard-instructions"><span className="metric-label"><Keyboard size={13}/> MANUAL CONTROL</span><h2>Hold a key. Take a step.</h2><p><b>W / S</b> forward & back · <b>A / D</b> turn<br/><b>Q / E</b> strafe · <b>Space</b> stop</p><span>Arrow keys work too. Releasing controls finishes the current step, then holds.</span></div><div className="drive-pad">{driveButton('q','Strafe left (Q)',<ArrowLeft size={17}/>)}{driveButton('w','Walk forward (W)',<ArrowUp size={18}/>)}{driveButton('e','Strafe right (E)',<ArrowRight size={17}/>)}{driveButton('a','Turn left (A)',<RotateCcw size={16}/>)}{driveButton('s','Walk backward (S)',<ArrowDown size={18}/>)}{driveButton('d','Turn right (D)',<RotateCcw className="mirror" size={16}/>)}</div></div>
        </section>
        <aside className="live-navigation"><div className="panel-title"><span className="tiny-index">02</span> Choose a destination <Crosshair size={14}/></div><NavigationMap state={s} enabled={available} onGoal={goal}/><div className="map-legend"><span><i className="legend-robot"/> Robot</span><span><i className="legend-route"/> Planned route</span><span><i className="legend-trail"/> Travelled</span></div><p className="map-help">Click clear floor to navigate. Shaded areas reserve room for the body and swinging legs.</p><div className="station-buttons"><button disabled={!available} onClick={() => goal([.1,1.3])}>North station <ArrowUpRight size={13}/></button><button disabled={!available} onClick={() => goal([1.4,1.3])}>Around the crates <ArrowUpRight size={13}/></button><button disabled={!available} onClick={() => goal([0,0])}>Return home <RotateCcw size={12}/></button></div><div className="goal-readout"><span>DESTINATION</span><strong>{s?.goal ? `${short(s.goal[0])}, ${short(s.goal[1])} m` : 'Choose on map'}</strong>{s?.goal && <small>{short(Math.hypot(s.goal[0]-s.center[0],s.goal[1]-s.center[1]))} m remaining</small>}</div></aside>
      </div>
      <section className="live-bottom"><div className="live-state-card"><div className="panel-title"><span className="tiny-index">03</span> Body & contacts</div><div className="live-values"><div><span>HEIGHT</span><strong>{short(s?.position[2],3)}<small>m</small></strong></div><div><span>ROLL / PITCH</span><strong>{short(s?.roll,1)}° / {short(s?.pitch,1)}°</strong></div><div><span>PEAK TORQUE</span><strong>{short(s?.torque,1)}<small>N·m</small></strong></div></div><div className="live-feet">{['FL','FR','RL','RR'].map((leg,i) => <div key={leg}><span className={`foot-dot ${(s?.loads[i] ?? 0)>1 ? 'grounded' : ''}`}/><b>{leg}</b><span>{short(s?.loads[i],0)} N</span><small>{s?.supporting[i] === false ? 'SWING' : 'STANCE'}</small></div>)}</div></div><div className="live-method-card"><div className="eyebrow">WHAT YOU ARE CONTROLLING</div><h2>Plan → shift weight → swing → land.</h2><p>A* plans around known obstacles. A slow crawl shifts the body over three supporting feet; contact-force allocation and foot tracking drive the 12 motors. Top speed is 0.06 m/s.</p><a href="#replay">Compare the original balance controllers <ArrowUpRight size={13}/></a></div></section>
      <p className="live-scope">Real-time MuJoCo simulation in your browser on a flat floor. Uses exact robot state and a known obstacle map. No perception or learned walking policy. Movement commands release when this page loses focus. The robot finishes its current step before holding; hidden tabs pause the simulation.</p>
    </main><footer><a className="brand" href="#live"><ScanLine size={17}/> SENTRY</a><span>500 Hz physics · 100 Hz control · 12 torque actuators</span><a href="#replay">Recorded experiments <ArrowUpRight size={13}/></a></footer>
  </div>;
}
