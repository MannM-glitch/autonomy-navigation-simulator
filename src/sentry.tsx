import React, { useEffect, useRef, useState } from 'react';
import { createRoot } from 'react-dom/client';
import { ArrowDownToLine, ArrowUpRight, Activity, Box, Check, ChevronRight, Code2, Cpu, ExternalLink, Pause, Play, RotateCcw, ScanLine } from 'lucide-react';
import './sentry.css';

type Mode = 'whole_body' | 'joint_pd';
type Frame = { time: number; phase: string; height: number; targetHeight: number; roll: number; pitch: number; drift: number; loads: number[]; push: number[]; torque: number };
type Metrics = { rmsTiltDeg: number; peakTiltDeg: number; heightRmseMm: number; maxDriftMm: number; recoverySeconds: (number | null)[]; fell: boolean; solverFailures: number };
type Run = { controller: Mode; config: { payloadKg: number; friction: number; pushScale: number; duration: number }; metrics: Metrics; frames: Frame[] };
type Report = { engine: string; sourceSha256: string; runs: Record<Mode, Run> };
type Benchmark = { criteria: string; rows: { case: string; controller: Mode; config: Run['config']; metrics: Metrics; meetsMission: boolean }[] };
const REPO = 'https://github.com/MannM-glitch/autonomy-navigation-simulator';
const ASSETS = 'sentry/';
const fmt = (n: number, digits = 2) => n.toFixed(digits);

function Trace({ runs, time, kind }: { runs: Record<Mode, Run>; time: number; kind: 'tilt' | 'height' }) {
  const width = 600, height = 126, left = 32, top = 12, bottom = 101;
  const value = (f: Frame) => kind === 'tilt' ? Math.hypot(f.roll, f.pitch) : 1000 * (f.height - f.targetHeight);
  const min = kind === 'tilt' ? 0 : -25, max = kind === 'tilt' ? 3 : 5;
  const x = (t: number) => left + t / 14 * (width - left - 10);
  const y = (v: number) => bottom - (v - min) / (max - min) * (bottom - top);
  return <svg className="trace" viewBox={`0 0 ${width} ${height}`} role="img" aria-label={`${kind === 'tilt' ? 'Body tilt in degrees' : 'Height error in millimeters'} for both controllers over 14 seconds`}>
    {[0, 1, 2].map(i => { const v = min + (max-min)*i/2; return <g key={i}><line x1={left} x2={width-10} y1={y(v)} y2={y(v)} className="chart-grid"/><text x={left-8} y={y(v)+3} textAnchor="end">{v}</text></g>; })}
    {[5.8, 10.5].map(t => <rect key={t} x={x(t)} width={x(.25)-x(0)} y={top} height={bottom-top} className="push-region"/>)}
    {(['joint_pd', 'whole_body'] as Mode[]).map(mode => <path key={mode} d={runs[mode].frames.map((f,i) => `${i ? 'L' : 'M'}${x(f.time).toFixed(1)},${y(value(f)).toFixed(1)}`).join(' ')} fill="none" stroke={mode === 'whole_body' ? '#87e5b1' : '#e3ad69'} strokeWidth="1.8"/>)}
    <line x1={x(time)} x2={x(time)} y1={top} y2={bottom} stroke="#d7e0df" strokeDasharray="3 3" opacity=".65"/>
    {[0, 3, 6, 9, 12, 14].map(t => <text key={t} x={x(t)} y={120} textAnchor="middle">{t}s</text>)}
  </svg>;
}

function App() {
  const [report, setReport] = useState<Report | null>(null);
  const [benchmark, setBenchmark] = useState<Benchmark | null>(null);
  const [error, setError] = useState('');
  const [mode, setMode] = useState<Mode>('whole_body');
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(true);
  const [videoError, setVideoError] = useState(false);
  const video = useRef<HTMLVideoElement>(null);
  const resumeTime = useRef(0);

  useEffect(() => {
    const abort = new AbortController();
    Promise.all(['experiment.json', 'benchmark.json'].map(async file => {
      const response = await fetch(`${ASSETS}${file}`, { signal: abort.signal });
      if (!response.ok) throw new Error(`Could not load ${file} (${response.status}).`);
      return response.json();
    })).then(([r,b]) => {
      if (!r.runs?.whole_body?.frames?.length || !r.runs?.joint_pd?.frames?.length || !b.rows?.length) throw new Error('The experiment files are incomplete.');
      setReport(r); setBenchmark(b);
    }).catch(e => { if (e.name !== 'AbortError') setError(e.message); });
    return () => abort.abort();
  }, []);

  useEffect(() => {
    if (!playing) return;
    let id: number;
    const tick = () => { if (video.current) setTime(video.current.currentTime); id = requestAnimationFrame(tick); };
    id = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(id);
  }, [playing]);

  function seek(t: number) {
    resumeTime.current = t;
    if (video.current) video.current.currentTime = t;
    setTime(t);
  }
  function toggle() {
    if (!video.current) return;
    if (video.current.paused) void video.current.play().catch(() => setPlaying(false));
    else video.current.pause();
  }
  function switchMode(next: Mode) {
    if (next === mode) return;
    resumeTime.current = video.current?.currentTime ?? time;
    setVideoError(false); setMode(next);
  }

  if (error) return <main className="load-state"><Cpu size={32}/><h1>Experiment unavailable</h1><p>{error}</p><p>Run <code>python -m simulation.export</code> and <code>python -m simulation.benchmark</code>, then reload.</p><button onClick={() => location.reload()}>Reload experiment</button></main>;
  if (!report || !benchmark) return <main className="load-state"><Cpu size={32}/><h1>Loading SENTRY</h1><p>Reading recorded physics and experiment results…</p></main>;
  const run = report.runs[mode];
  const f = run.frames[Math.min(run.frames.length-1, Math.floor(time*50))];
  const active = report.runs.whole_body.metrics, baseline = report.runs.joint_pd.metrics;
  const pushing = Math.hypot(...f.push) > 0;
  const nominalRows = benchmark.rows.filter(r => r.controller === 'whole_body');
  const phaseIndex = time < 3 ? 0 : time < 5 ? 1 : time < 8 ? 2 : time < 10 ? 3 : 4;

  return <div className="sentry-app">
    <header className="site-header">
      <a className="brand" href="#top" aria-label="SENTRY home"><span className="brand-mark"><ScanLine size={22}/></span>SENTRY<span className="brand-tag">ROBOTICS LAB</span></a>
      <nav aria-label="Main navigation"><a className="nav-active" href="#experiment">Experiment</a><a href="#evidence">Results</a><a href="#method">Method</a></nav>
      <a className="source-link" href={REPO} target="_blank" rel="noreferrer">View source <ArrowUpRight size={16}/></a>
    </header>

    <main id="top">
      <section className="hero">
        <div><div className="eyebrow"><span className="status-dot"/> WHOLE-BODY CONTROL / EXPERIMENT 001</div><h1>Four feet. One steady platform.</h1><p>A quadruped lowers its sensor payload, absorbs a push, and finds its balance.<br className="desktop-break"/> Explore the physics. Compare the controllers. See where it breaks.</p></div>
        <div className="hero-spec"><div><Cpu size={16}/><span>MuJoCo physics</span><span className="spec-dot"/></div><div><Box size={16}/><span>12 actuators · floating base</span></div><div><Activity size={16}/><span>500 Hz physics / 100 Hz control</span></div></div>
      </section>

      <section className="headline-metrics" aria-label="Nominal experiment results">
        <div><span className="metric-label">RMS BODY TILT</span><strong>{fmt(active.rmsTiltDeg)}<small>°</small></strong><span className="metric-note"><b>↓ {fmt(100*(1-active.rmsTiltDeg/baseline.rmsTiltDeg),0)}%</b> vs joint PD</span></div>
        <div><span className="metric-label">HEIGHT TRACKING ERROR</span><strong>{fmt(active.heightRmseMm,1)}<small>mm</small></strong><span className="metric-note"><b>↓ {fmt(100*(1-active.heightRmseMm/baseline.heightRmseMm),0)}%</b> vs joint PD</span></div>
        <div><span className="metric-label">PUSH RECOVERY</span><strong>{fmt(active.recoverySeconds[0] ?? 0)}<small>s</small></strong><span className="metric-note">Both nominal disturbances</span></div>
        <div><span className="metric-label">MEASURED, NOT KEYFRAMED</span><strong className="physics-label">Real dynamics<ArrowUpRight size={22}/></strong><span className="metric-note">Recorded simulation · no hidden supports</span></div>
      </section>

      <section id="experiment" className="experiment-grid">
        <div className="viewer-panel">
          <div className="panel-toolbar"><div className="panel-title"><span className="tiny-index">01</span> Simulation replay</div><div className="controller-toggle" aria-label="Controller comparison"><button aria-pressed={mode === 'whole_body'} className={mode === 'whole_body' ? 'selected' : ''} onClick={() => switchMode('whole_body')}>Whole-body</button><button aria-pressed={mode === 'joint_pd'} className={mode === 'joint_pd' ? 'selected baseline' : ''} onClick={() => switchMode('joint_pd')}>Joint PD</button></div></div>
          <div className="video-stage">
            <video key={mode} ref={video} src={`${ASSETS}${mode}.mp4`} poster={`${ASSETS}poster.jpg`} muted loop playsInline preload="auto" autoPlay={playing}
              onLoadedMetadata={() => { if (video.current) video.current.currentTime = resumeTime.current; }}
              onPlay={() => setPlaying(true)} onPause={() => setPlaying(false)} onSeeked={() => setTime(video.current?.currentTime ?? 0)} onError={() => setVideoError(true)} aria-label={`${mode === 'whole_body' ? 'Whole-body' : 'Joint PD'} controller MuJoCo recording`}/>
            <div className="scene-caption"><span className={mode === 'whole_body' ? 'status-dot' : 'status-dot amber'}/><span>{mode === 'whole_body' ? 'CENTROIDAL FORCE CONTROL' : 'JOINT POSITION CONTROL'}</span></div>
            <div className="scene-time">t + {fmt(time)}<span> / 14.00 s</span></div>
            <div className={`phase-badge ${pushing ? 'push-active' : ''}`}><span className="phase-kicker">{pushing ? 'EXTERNAL DISTURBANCE' : 'CURRENT PHASE'}</span><strong>{pushing ? `${fmt(Math.hypot(...f.push),0)} N ${f.push[1] ? 'lateral' : 'longitudinal'} push` : f.phase}</strong></div>
            <div className="scene-key"><i/> Measured foot loads <span>↑</span></div>
            {videoError && <div className="video-error">Video could not load. <a href={`${ASSETS}${mode}.mp4`}>Open the recording</a> or regenerate with <code>python -m simulation.export</code>.</div>}
          </div>
          <div className="transport"><button className="play-button" onClick={toggle} aria-label={playing ? 'Pause replay' : 'Play replay'}>{playing ? <Pause size={17} fill="currentColor"/> : <Play size={17} fill="currentColor"/>}</button><button className="restart-button" onClick={() => seek(0)} aria-label="Restart replay"><RotateCcw size={17}/></button><input type="range" aria-label="Replay time" min="0" max="13.96" step="0.02" value={Math.min(time,13.96)} onChange={e => seek(Number(e.target.value))}/><span className="transport-time">{fmt(time,1)} / 14 s</span></div>
          <div className="phase-track">{[['Stabilize',0],['Lower',3],['Inspect + push',5.8],['Rise',8],['Recover',10.5]].map(([label,t], i) => <button key={label} className={i === phaseIndex ? 'current' : ''} onClick={() => seek(Number(t))}><span>0{i+1}</span>{label}<ChevronRight size={12}/></button>)}</div>
        </div>

        <aside className="telemetry-panel"><div className="panel-title"><span className="tiny-index">02</span> Robot state <span className="telemetry-tag">SYNCED</span></div>
          <div className="height-display"><span className="metric-label">BODY HEIGHT</span><strong>{fmt(f.height,3)} <small>m</small></strong><div><span>Target {fmt(f.targetHeight,3)} m</span><b>{f.height-f.targetHeight >= 0 ? '+' : ''}{fmt(1000*(f.height-f.targetHeight),1)} mm</b></div></div>
          <div className="pose-grid"><div><span>ROLL</span><strong>{fmt(f.roll,2)}<small>°</small></strong></div><div><span>PITCH</span><strong>{fmt(f.pitch,2)}<small>°</small></strong></div><div><span>PLANAR DRIFT</span><strong>{fmt(f.drift*1000,1)}<small>mm</small></strong></div><div><span>PEAK TORQUE</span><strong>{fmt(f.torque,1)}<small>N·m</small></strong></div></div>
          <div className="contact-heading"><span className="metric-label">FOOT CONTACT LOADS</span><span>Normal force</span></div>
          <div className="contact-loads">{['FL','FR','RL','RR'].map((leg,i) => <div className="contact-row" key={leg}><span className={f.loads[i] > 1 ? 'foot-dot grounded' : 'foot-dot'}/><span>{leg}</span><div className="load-track"><div style={{width: `${Math.min(100,f.loads[i]/100*100)}%`}}/></div><strong>{fmt(f.loads[i],0)}<small>N</small></strong></div>)}</div>
          <div className="telemetry-footer"><span>Payload <b>1.5 kg</b></span><span>Ground friction <b>0.80</b></span></div>
        </aside>
      </section>

      <section className="chart-grid-layout"><div className="chart-card"><div className="chart-title"><h2>Body tilt <span>deg</span></h2><div className="legend"><i/>Whole-body<i className="pd"/>Joint PD</div></div><Trace runs={report.runs} time={time} kind="tilt"/></div><div className="chart-card"><div className="chart-title"><h2>Height error <span>mm</span></h2><span className="chart-hint">Shaded regions = applied pushes</span></div><Trace runs={report.runs} time={time} kind="height"/></div></section>
      <p className="replay-note"><Activity size={13}/> Video and telemetry come from the same MuJoCo run. Switch controllers at any point to compare. <a href={`${REPO}/tree/main/simulation#run-it`} target="_blank" rel="noreferrer">Run live physics <ArrowUpRight size={12}/></a></p>

      <section id="evidence" className="evidence-section"><div className="section-heading"><div><div className="eyebrow">EVIDENCE / PARAMETER SWEEP</div><h2>A good demo includes its limits.</h2><p>Five deterministic scenarios. The same mission criteria for both controllers.</p></div><a className="outline-link" href={`${ASSETS}benchmark.json`} download><ArrowDownToLine size={15}/> Download results</a></div>
        <div className="results-table-wrap"><table><thead><tr><th>Scenario</th><th>Payload / friction / push</th><th>Whole-body RMS tilt</th><th>Joint PD RMS tilt</th><th>Whole-body outcome</th></tr></thead><tbody>{nominalRows.map(row => { const pd = benchmark.rows.find(r => r.case === row.case && r.controller === 'joint_pd')!; return <tr key={row.case}><td>{row.case}</td><td>{row.config.payloadKg} kg <span>/</span> {row.config.friction.toFixed(2)} <span>/</span> {row.config.pushScale.toFixed(1)}×</td><td className={row.metrics.fell ? 'danger-text' : 'mint-text'}>{fmt(row.metrics.rmsTiltDeg)}°</td><td>{fmt(pd.metrics.rmsTiltDeg)}°</td><td><span className={`outcome ${row.meetsMission ? 'pass' : row.metrics.fell ? 'fail' : 'limit'}`}>{row.meetsMission ? <Check size={12}/> : null}{row.meetsMission ? 'Meets mission' : row.metrics.fell ? 'Falls' : 'Slow recovery'}</span></td></tr>; })}</tbody></table></div>
        <div className="evidence-footnote"><span>Mission: no fall · peak tilt &lt;3° · height RMSE &lt;6 mm · drift &lt;100 mm · both recoveries &lt;1 s.</span><span>Joint PD does not meet all criteria in any of these five cases.</span></div>
        <div className="failure-note"><span className="failure-marker">!</span><div><strong>Why it fails on very slippery ground</strong><p>The controller assumes four supporting feet and tries to return to the original position. With insufficient friction, it needs a recovery step it cannot take. In the last trial, whole-body control falls; joint PD stays upright but slides {fmt(benchmark.rows.find(r => r.case === 'Beyond stance limits' && r.controller === 'joint_pd')!.metrics.maxDriftMm/1000,2)} m. Neither completes the mission.</p></div></div>
      </section>

      <section id="method" className="method-section"><div className="section-heading"><div><div className="eyebrow">UNDER THE HOOD</div><h2>From body error to twelve motor torques.</h2></div><a className="text-link" href={`${REPO}/blob/main/simulation/README.md`} target="_blank" rel="noreferrer">Read the engineering notes <ArrowUpRight size={16}/></a></div><div className="method-grid"><article><span>01 / FEEDBACK</span><h3>Ask for a body wrench.</h3><p>Position and orientation errors produce a target force and moment. Gravity feedforward supports the robot and its payload.</p><code>error → [force, moment]</code></article><article><span>02 / FORCE ALLOCATION</span><h3>Share the work across feet.</h3><p>A constrained quadratic objective distributes the wrench. Feet can push, but cannot pull; tangential forces respect a friction pyramid.</p><code>min ½ ‖W(Af − w)‖² + λ ‖f‖²</code></article><article><span>03 / ACTUATION</span><h3>Turn contact forces into torque.</h3><p>Foot Jacobians map forces to joints. Add posture feedback, then apply the ±32 N·m actuator limit and step the physics.</p><code>τ = bias − Jᵀf + posture PD</code></article></div>
        <div className="scope-note"><Code2 size={18}/><p><strong>Scope, stated precisely.</strong> Reduced-order whole-body stance control with exact simulated state. No walking policy, perception pipeline, RL training, or hardware validation. The browser is a replay; Python runs the physics.</p></div>
      </section>
    </main>
    <footer><a className="brand" href="#top"><ScanLine size={17}/> SENTRY</a><span>{report.engine} · Original 12-DOF model · Reproducible experiments</span><a href={`${ASSETS}experiment.json`} download>Raw telemetry <ExternalLink size={13}/></a></footer>
  </div>;
}

createRoot(document.getElementById('root')!).render(<React.StrictMode><App/></React.StrictMode>);
