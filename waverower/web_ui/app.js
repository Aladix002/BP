/* global ROSLIB, React, ReactDOM */
"use strict";

const { useState, useEffect, useRef, useCallback } = React;

// ─── Constants ────────────────────────────────────────────────────────────────
const TELEOP_TOPIC   = "/teleop_cmd_vel";
const TWIST_TYPE     = "geometry_msgs/msg/Twist";
const CAMERA_TOPIC   = "/camera/camera_node/image_raw/compressed";
const CAMERA_TYPE    = "sensor_msgs/msg/CompressedImage";
const IMU_TOPIC      = "/imu";
const IMU_TYPE       = "sensor_msgs/msg/Imu";
const IMU_DBG_TOPIC  = "/motor_hat_node/drive_debug";
const IMU_DBG_TYPE   = "std_msgs/msg/Float64MultiArray";
const MOTOR_NODE     = "/motor_hat_node";
const WANDER_NODE    = "/lidar_wander_node";
// Teleop publish rate (vyššie = hladšie ovládanie; zvyšuje traffic cez rosbridge)
const PUBLISH_HZ     = 30;
// Plna skala: 0..100% z toho, co ma motor_hat_node ako teleop_max_* (fetch pri connect).
const SLIDER_SCALE   = { min: 0.0, max: 1.0, step: 0.01 };
const TURN_LIN_RATIO = 2.0;
// Zhoda s waverower/launch/runtime_stack.launch.py (WANDER_TURN_RATIO)
const WANDER_TURN_RATIO = 18.0;

const IMU_DEFAULTS = {
  imu_correction: false,
  imu_kp:         0.30,
  imu_ki:         0.05,
  imu_kd:         0.01,
  imu_deadband:   0.02,
  imu_windup:     0.30,
};

const PTYPE_BOOL   = 1;
const PTYPE_DOUBLE = 3;
const PTYPE_STRING = 4;

// ─── Helpers ──────────────────────────────────────────────────────────────────
const clamp = (n, lo, hi) => Math.min(hi, Math.max(lo, n));
const snap  = (n, step)   => Math.round(n / step) * step;
const defaultWsUrl = () => `ws://${window.location.hostname || "127.0.0.1"}:9090`;

const makeParam = (name, value) =>
  typeof value === "boolean"
    ? { name, value: { type: PTYPE_BOOL,   bool_value:   value } }
    : { name, value: { type: PTYPE_DOUBLE, double_value: value } };

const makeStringParam = (name, value) => ({
  name,
  value: { type: PTYPE_STRING, string_value: value },
});

const rosGetParams = (ros, node, names) =>
  new Promise((ok, err) =>
    new ROSLIB.Service({ ros, name: `${node}/get_parameters`, serviceType: "rcl_interfaces/srv/GetParameters" })
      .callService(new ROSLIB.ServiceRequest({ names }), ok, err));

const rosSetParams = (ros, node, parameters) =>
  new Promise((ok, err) =>
    new ROSLIB.Service({ ros, name: `${node}/set_parameters`, serviceType: "rcl_interfaces/srv/SetParameters" })
      .callService(new ROSLIB.ServiceRequest({ parameters }), ok, err));

// ─── Hook: useRos ─────────────────────────────────────────────────────────────
function useRos() {
  const [ros, setRos]             = useState(null);
  const [connected, setConnected] = useState(false);
  const [wsUrl, setWsUrl]         = useState(
    () => localStorage.getItem("waverower_ws_url") || defaultWsUrl()
  );
  const rosRef = useRef(null);

  const connect = useCallback((url) => {
    const u = url || wsUrl;
    localStorage.setItem("waverower_ws_url", u);
    if (rosRef.current) { try { rosRef.current.close(); } catch (_) {} }
    const r = new ROSLIB.Ros({ url: u });
    rosRef.current = r;
    r.on("connection", () => { setRos(r); setConnected(true); });
    r.on("error",      () => setConnected(false));
    r.on("close",      () => { setConnected(false); setRos(null); rosRef.current = null; });
  }, [wsUrl]);

  const disconnect = useCallback(() => {
    if (rosRef.current) { try { rosRef.current.close(); } catch (_) {} }
  }, []);

  return { ros, connected, wsUrl, setWsUrl, connect, disconnect };
}

// ─── Shared UI ────────────────────────────────────────────────────────────────
const SectionLabel = ({ children }) => (
  <span className="block text-[0.6rem] font-bold uppercase tracking-[0.1em] text-slate-500 mb-2">
    {children}
  </span>
);

const Card = ({ children, className = "" }) => (
  <div className={`bg-slate-900 border border-slate-800 rounded-xl shadow-lg shadow-black/40 mb-2 ${className}`}>
    {children}
  </div>
);

// ─── Header ───────────────────────────────────────────────────────────────────
function Header({ connected }) {
  return (
    <header className="flex items-center justify-between gap-3 pb-3 mb-3 border-b border-slate-800">
      <h1 className="font-display text-2xl font-semibold tracking-tight">Wave Rover</h1>
      <span className={`text-[0.65rem] font-bold uppercase tracking-widest px-2.5 py-1 rounded-full border transition-colors ${
        connected
          ? "text-emerald-400 border-emerald-500/40 bg-emerald-950/50"
          : "text-slate-500 border-slate-700 bg-slate-900"
      }`}>
        {connected ? "Connected" : "Disconnected"}
      </span>
    </header>
  );
}

// ─── ConnectionCard ───────────────────────────────────────────────────────────
function ConnectionCard({ wsUrl, setWsUrl, connected, connect, disconnect }) {
  return (
    <Card className="p-3">
      <SectionLabel>WebSocket</SectionLabel>
      <div className="flex flex-wrap gap-2">
        <input
          type="text"
          value={wsUrl}
          placeholder={defaultWsUrl()}
          autoComplete="off"
          onChange={e => setWsUrl(e.target.value)}
          onKeyDown={e => e.key === "Enter" && connect(wsUrl)}
          className="flex-1 min-w-0 px-3 py-2 rounded-lg bg-slate-950/70 border border-slate-700
                     text-sm text-slate-200 placeholder-slate-600
                     focus:outline-none focus:border-blue-500/50 transition-colors"
        />
        <div className="flex gap-2 shrink-0">
          <button
            onClick={() => connect(wsUrl)}
            className="px-4 py-2 rounded-lg text-sm font-semibold bg-blue-500 text-white
                       hover:bg-blue-400 active:scale-95 disabled:opacity-40 transition-all touch-manipulation"
          >Connect</button>
          <button
            onClick={disconnect}
            className="px-4 py-2 rounded-lg text-sm font-semibold bg-slate-800 text-slate-300
                       border border-slate-700 hover:bg-slate-700 active:scale-95 transition-all touch-manipulation"
          >Disconnect</button>
        </div>
      </div>
    </Card>
  );
}

// ─── ModeCard ─────────────────────────────────────────────────────────────────
function ModeCard({ ros, connected }) {
  const [mode, setMode]     = useState("manual");
  const [status, setStatus] = useState("—");
  const [busy, setBusy]     = useState(false);

  async function switchMode(m) {
    if (!connected || busy) return;
    setBusy(true); setStatus("Switching…");
    const isWander = m === "wander";
    try {
      const mr = await rosSetParams(ros, MOTOR_NODE, [
        makeStringParam("control_mode", isWander ? "auto" : "manual"),
      ]);
      if (!mr?.results?.every(x => x.successful)) {
        const why = mr?.results?.find(r => !r.successful)?.reason || "motor set_parameters failed";
        throw new Error(why);
      }
      try {
        const wr = await rosSetParams(ros, WANDER_NODE, [makeParam("enabled", isWander)]);
        if (!wr?.results?.every(x => x.successful)) {
          const why = wr?.results?.find(r => !r.successful)?.reason || "wander set_parameters failed";
          throw new Error(why);
        }
      } catch (e) {
        if (isWander) throw e;
      }
      setMode(m);
      setStatus(m === "manual" ? "Manual active" : "Wander active");
    } catch (e) {
      setStatus("Error: " + e);
    } finally {
      setBusy(false);
    }
  }

  return (
    <Card className="flex flex-wrap items-center gap-2 px-3 py-2.5">
      <SectionLabel>Mode</SectionLabel>
      <div className="flex rounded-lg overflow-hidden border border-slate-700 shrink-0">
        {["manual", "wander"].map((m, i) => (
          <button
            key={m}
            onClick={() => switchMode(m)}
            disabled={busy || !connected}
            className={`px-5 py-1.5 text-sm font-semibold transition-colors touch-manipulation disabled:opacity-40
              ${i > 0 ? "border-l border-slate-700" : ""}
              ${mode === m
                ? "bg-emerald-950/60 text-emerald-400"
                : "bg-slate-800/50 text-slate-400 hover:bg-slate-800"
              }`}
          >{m.charAt(0).toUpperCase() + m.slice(1)}</button>
        ))}
      </div>
      <span className="text-xs text-slate-500 flex-1 text-right">{status}</span>
    </Card>
  );
}

// ─── CameraPanel ──────────────────────────────────────────────────────────────
function CameraPanel({ ros, connected }) {
  const canvasRef       = useRef(null);
  const [live, setLive] = useState(false);
  const genRef          = useRef(0);
  const subRef          = useRef(null);

  useEffect(() => {
    if (!ros || !connected) {
      genRef.current++;
      setLive(false);
      if (subRef.current) { try { subRef.current.unsubscribe(); } catch (_) {} subRef.current = null; }
      return;
    }
    const gen = ++genRef.current;
    let primed = false, frameSeq = 0;
    // throttle_rate = min. interval medzi správami v ms (rosbridge); ~33 ms ≈ 30 Hz
    const sub = new ROSLIB.Topic({ ros, name: CAMERA_TOPIC, messageType: CAMERA_TYPE, throttle_rate: 33 });
    subRef.current = sub;

    sub.subscribe((m) => {
      if (gen !== genRef.current) return;
      const seq = ++frameSeq;
      let url, revoke = null;
      try {
        if (typeof m.data === "string") { url = "data:image/jpeg;base64," + m.data; }
        else { const b = new Blob([new Uint8Array(m.data)], { type: "image/jpeg" }); url = URL.createObjectURL(b); revoke = url; }
      } catch (_) { return; }
      const img = new Image();
      img.decoding = "async";
      img.onload = () => {
        if (gen !== genRef.current || seq !== frameSeq) { if (revoke) URL.revokeObjectURL(revoke); return; }
        const canvas = canvasRef.current;
        if (!canvas) { if (revoke) URL.revokeObjectURL(revoke); return; }
        const ctx = canvas.getContext("2d");
        const par = canvas.parentElement;
        const w = Math.max(1, par.clientWidth), h = Math.max(1, par.clientHeight);
        if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
        const s = Math.min(w / img.naturalWidth, h / img.naturalHeight);
        const dw = img.naturalWidth * s, dh = img.naturalHeight * s;
        ctx.fillStyle = "#030508"; ctx.fillRect(0, 0, w, h);
        ctx.drawImage(img, (w - dw) * 0.5, (h - dh) * 0.5, dw, dh);
        if (revoke) URL.revokeObjectURL(revoke);
        if (!primed) { primed = true; setLive(true); }
      };
      img.onerror = () => { if (revoke) URL.revokeObjectURL(revoke); };
      img.src = url;
    });
    return () => { genRef.current++; try { sub.unsubscribe(); } catch (_) {} subRef.current = null; setLive(false); };
  }, [ros, connected]);

  return (
    <div className="bg-slate-900 border border-slate-800 rounded-xl p-2.5 shadow-lg shadow-black/40 flex flex-col">
      <div className="flex items-center justify-between mb-2">
        <SectionLabel>Camera</SectionLabel>
        <span className={`text-[0.58rem] font-bold uppercase tracking-wider px-2 py-0.5 rounded-full border transition-colors ${
          live ? "text-emerald-400 border-emerald-500/40 bg-emerald-950/40" : "text-slate-500 border-slate-700"
        }`}>{live ? "Live" : "Standby"}</span>
      </div>
      {/* gradient border frame */}
      <div className="rounded-lg p-[3px] bg-gradient-to-br from-blue-500/20 via-slate-900 to-emerald-500/10">
        <div className="relative rounded-md overflow-hidden aspect-[4/3] bg-slate-950 flex items-center justify-center">
          {!live && (
            <div className="absolute inset-0 flex flex-col items-center justify-center gap-2 text-slate-600 pointer-events-none">
              <span className="text-3xl opacity-40">◉</span>
              <span className="text-xs text-center max-w-[8rem]">Connect to show the feed</span>
            </div>
          )}
          <canvas
            ref={canvasRef}
            className={`absolute inset-0 w-full h-full block bg-[#030508] z-10 ${live ? "" : "hidden"}`}
            aria-label="Robot camera feed"
          />
        </div>
      </div>
    </div>
  );
}

// ─── DrivePanel ───────────────────────────────────────────────────────────────
function DrivePanel({ ros, connected }) {
  const [speedScale, setSpeedScale] = useState(() => {
    const v = parseFloat(localStorage.getItem("waverower_web_speed_scale"));
    return Number.isFinite(v)
      ? snap(clamp(v, SLIDER_SCALE.min, SLIDER_SCALE.max), SLIDER_SCALE.step)
      : 0.5;
  });
  /** Musi sediet s /motor_hat_node teleop_max_* — inak sa skala „zasekne“ (napr. max uz pri 50 %). */
  const [teleopMax, setTeleopMax] = useState({ lin: 0.5, ang: 1.8 });
  const teleopMaxRef = useRef({ lin: 0.5, ang: 1.8 });
  const twistRef    = useRef({ linear: { x:0,y:0,z:0 }, angular: { x:0,y:0,z:0 } });
  const pubRef      = useRef(null);
  const cmdRef      = useRef(null);
  const scaleRef    = useRef(speedScale);
  const syncRef     = useRef(null);
  scaleRef.current = speedScale;
  teleopMaxRef.current = teleopMax;

  useEffect(() => {
    if (!ros || !connected) return;
    let cancelled = false;
    (async () => {
      try {
        const r = await rosGetParams(ros, MOTOR_NODE, [
          "teleop_max_linear",
          "teleop_max_angular",
        ]);
        if (cancelled || !r?.values) return;
        const next = { lin: 1.0, ang: 2.0 };
        const names = ["teleop_max_linear", "teleop_max_angular"];
        names.forEach((name, i) => {
          const v = r.values[i];
          if (!v || v.type !== PTYPE_DOUBLE) return;
          if (name === "teleop_max_linear"  && v.double_value >= 0.05) next.lin = v.double_value;
          if (name === "teleop_max_angular" && v.double_value >= 0.1)  next.ang = v.double_value;
        });
        teleopMaxRef.current = next;
        setTeleopMax(next);
      } catch (_) {
        /* fallback defaults */
      }
    })();
    return () => { cancelled = true; };
  }, [ros, connected]);

  useEffect(() => {
    if (!ros || !connected) {
      if (pubRef.current) { clearInterval(pubRef.current); pubRef.current = null; }
      cmdRef.current = null;
      return;
    }
    cmdRef.current = new ROSLIB.Topic({ ros, name: TELEOP_TOPIC, messageType: TWIST_TYPE });
    pubRef.current = setInterval(() => cmdRef.current?.publish(new ROSLIB.Message(twistRef.current)), 1000 / PUBLISH_HZ);
    return () => { if (pubRef.current) { clearInterval(pubRef.current); pubRef.current = null; } cmdRef.current = null; };
  }, [ros, connected]);

  useEffect(() => {
    if (!ros || !connected) return;
    if (syncRef.current) clearTimeout(syncRef.current);
    syncRef.current = setTimeout(async () => {
      try {
        const wanderFwd = scaleRef.current * teleopMaxRef.current.lin;
        const wanderTurn = wanderFwd * WANDER_TURN_RATIO;
        await rosSetParams(ros, WANDER_NODE, [
          makeParam("forward_speed", wanderFwd),
          makeParam("turn_speed", wanderTurn),
        ]);
      } catch (_) {
        // best-effort: slider musi fungovat aj ked wander node nebezi
      }
      syncRef.current = null;
    }, 250);
    return () => {
      if (syncRef.current) {
        clearTimeout(syncRef.current);
        syncRef.current = null;
      }
    };
  }, [speedScale, ros, connected, teleopMax.lin, teleopMax.ang]);

  const stop = () => {
    twistRef.current = { linear: { x:0,y:0,z:0 }, angular: { x:0,y:0,z:0 } };
    cmdRef.current?.publish(new ROSLIB.Message(twistRef.current));
  };

  const drive = (lx, az) => {
    const s = scaleRef.current;
    const { lin: maxL, ang: maxA } = teleopMaxRef.current;
    const lin = s * maxL;
    const ang = s * maxA;
    twistRef.current = {
      linear:  { x: lx * lin, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: az * ang },
    };
  };

  const dBtnClass = "aspect-square w-full rounded-xl font-bold text-slate-200 border border-white/5 " +
    "bg-gradient-to-b from-slate-700 to-slate-800 shadow-inner active:scale-95 active:brightness-110 " +
    "touch-manipulation transition-all select-none";

  const DBtn = ({ lx, az, label, children }) => (
    <button
      className={dBtnClass}
      aria-label={label}
      onMouseDown={() => drive(lx, az)} onMouseUp={stop} onMouseLeave={stop}
      onTouchStart={e => { e.preventDefault(); drive(lx, az); }}
      onTouchEnd={e => { e.preventDefault(); stop(); }}
    >{children}</button>
  );

  const onSlider = e => {
    const v = snap(clamp(parseFloat(e.target.value), SLIDER_SCALE.min, SLIDER_SCALE.max), SLIDER_SCALE.step);
    setSpeedScale(v);
    localStorage.setItem("waverower_web_speed_scale", String(v));
  };

  const maxLin = speedScale * teleopMax.lin;
  const maxAng = speedScale * teleopMax.ang;
  const speedPct = Math.round(speedScale * 100);

  return (
    <div className="bg-slate-900 border border-slate-800 rounded-xl p-2.5 shadow-lg shadow-black/40 flex flex-col">
      <SectionLabel>Drive</SectionLabel>

      {/* D-pad */}
      <div className="grid grid-cols-3 gap-1.5 w-full max-w-[11rem] mx-auto mt-1">
        <div /><DBtn lx={1}  az={0}  label="Forward">▲</DBtn><div />
        <DBtn lx={0} az={1}  label="Left">◀</DBtn>
        <button
          className="aspect-square w-full rounded-xl font-bold text-white border border-black/20 select-none
                     bg-gradient-to-b from-red-600 to-red-800 active:scale-95 touch-manipulation transition-all text-xs"
          onClick={stop} aria-label="Stop"
        >■</button>
        <DBtn lx={0} az={-1} label="Right">▶</DBtn>
        <div /><DBtn lx={-1} az={0}  label="Back">▼</DBtn><div />
      </div>

      {/* Speed sliders */}
      <div className="mt-3 pt-3 border-t border-slate-800 space-y-2.5">
        <div>
          <div className="flex items-center justify-between mb-1">
            <span className="text-[0.6rem] font-bold uppercase tracking-wider text-slate-500">Speed scale</span>
            <span className="text-xs font-semibold text-blue-400 tabular-nums">{speedPct}%</span>
          </div>
          <input type="range" min={SLIDER_SCALE.min} max={SLIDER_SCALE.max} step={SLIDER_SCALE.step}
            value={speedScale} onChange={onSlider} className="w-full" />
        </div>
        <div className="flex items-center justify-between">
          <span className="text-[0.6rem] font-bold uppercase tracking-wider text-slate-500">Effective <span className="normal-case font-normal">lin/ang</span></span>
          <span className="text-xs text-slate-500 tabular-nums">{maxLin.toFixed(2)} / {maxAng.toFixed(2)}</span>
        </div>
      </div>
    </div>
  );
}

// ─── ImuPanel ─────────────────────────────────────────────────────────────────
// Debug data layout from C++:
// [0]=active [1]=low_fwd [2]=yaw_rate [3]=error [4]=p [5]=i [6]=d [7]=total [8]=corr_pct [9]=pwm_l [10]=pwm_r
function ImuPanel({ ros, connected }) {
  const [wz, setWz]       = useState(null);
  const [imuSt, setImuSt] = useState("disconnected");
  const lastMsRef         = useRef(0);
  const subRef            = useRef(null);
  const timerRef          = useRef(null);

  const [dbg, setDbg]     = useState(null);   // raw Float64MultiArray data array
  const dbgSubRef         = useRef(null);

  const [pid, setPid]               = useState({ ...IMU_DEFAULTS });
  const [fetchMsg, setFetchMsg]     = useState("");
  const [applyMsg, setApplyMsg]     = useState("");
  const [fetchBusy, setFetchBusy]   = useState(false);
  const [applyBusy, setApplyBusy]   = useState(false);

  useEffect(() => {
    if (!ros || !connected) {
      lastMsRef.current = 0; setWz(null); setImuSt("disconnected"); setDbg(null);
      if (subRef.current)   { try { subRef.current.unsubscribe();   } catch (_) {} subRef.current   = null; }
      if (dbgSubRef.current){ try { dbgSubRef.current.unsubscribe();} catch (_) {} dbgSubRef.current = null; }
      if (timerRef.current) { clearInterval(timerRef.current); timerRef.current = null; }
      return;
    }

    // /imu subscriber (status indicator)
    const sub = new ROSLIB.Topic({ ros, name: IMU_TOPIC, messageType: IMU_TYPE });
    subRef.current = sub;
    sub.subscribe((msg) => {
      const av = msg.angular_velocity;
      let z = NaN;
      if (av && typeof av.z === "number") z = av.z;
      else if (av?.z != null) z = parseFloat(av.z);
      if (!Number.isFinite(z)) return;
      lastMsRef.current = Date.now(); setWz(z);
    });

    // debug topic – rosbridge throttle (ms); 50 → ~20 Hz (predtým 100 = 10 Hz)
    const dbgSub = new ROSLIB.Topic({
      ros, name: IMU_DBG_TOPIC, messageType: IMU_DBG_TYPE, throttle_rate: 50,
    });
    dbgSubRef.current = dbgSub;
    dbgSub.subscribe((msg) => setDbg(msg.data));

    timerRef.current = setInterval(() => {
      const age = lastMsRef.current ? Date.now() - lastMsRef.current : 999999;
      if (!lastMsRef.current) setImuSt("waiting");
      else if (age > 1200)    setImuSt("stale");
      else                    setImuSt("ok");
    }, 150);

    return () => {
      try { sub.unsubscribe();    } catch (_) {} subRef.current    = null;
      try { dbgSub.unsubscribe(); } catch (_) {} dbgSubRef.current = null;
      clearInterval(timerRef.current); timerRef.current = null;
    };
  }, [ros, connected]);

  async function fetchParams() {
    if (!connected || fetchBusy) return;
    setFetchBusy(true); setFetchMsg("Fetching…"); setApplyMsg("");
    try {
      const names = Object.keys(IMU_DEFAULTS);
      const r = await rosGetParams(ros, MOTOR_NODE, names);
      if (!r?.values) throw new Error("No response");
      const upd = { ...pid };
      names.forEach((name, i) => {
        const v = r.values[i];
        if (!v) return;
        if (v.type === PTYPE_BOOL)   upd[name] = v.bool_value;
        if (v.type === PTYPE_DOUBLE) upd[name] = v.double_value;
      });
      setPid(upd); setFetchMsg("Fetched OK");
    } catch (e) { setFetchMsg("Error: " + (e.message ?? e)); }
    finally { setFetchBusy(false); }
  }

  async function applyParams() {
    if (!connected || applyBusy) return;
    setApplyBusy(true); setApplyMsg("Applying…"); setFetchMsg("");
    try {
      const params = Object.entries(pid).map(([k, v]) => makeParam(k, v));
      const r = await rosSetParams(ros, MOTOR_NODE, params);
      setApplyMsg(r?.results?.every(x => x.successful) ? "Applied OK" : "Partial error");
    } catch (e) { setApplyMsg("Error: " + (e.message ?? e)); }
    finally { setApplyBusy(false); }
  }

  const ST_STYLE = {
    ok:           "text-emerald-400",
    stale:        "text-amber-400",
    waiting:      "text-slate-500",
    disconnected: "text-slate-600",
  };
  const ST_LABEL = { ok: "OK", stale: "No /imu", waiting: "Waiting…", disconnected: "Disconnected" };

  const NumField = ({ name, label, step }) => (
    <div>
      <span className="block text-[0.6rem] font-bold uppercase tracking-wider text-slate-500 mb-1">{label}</span>
      <input
        type="number" step={step} min={0}
        value={pid[name]}
        onChange={e => setPid(p => ({ ...p, [name]: parseFloat(e.target.value) || 0 }))}
        className="w-full px-2.5 py-1.5 rounded-lg bg-slate-950/60 border border-slate-700
                   text-sm text-slate-200 tabular-nums
                   focus:outline-none focus:border-blue-500/50 transition-colors"
      />
    </div>
  );

  return (
    <Card className="p-3">
      <SectionLabel>IMU · Regulation</SectionLabel>

      {/* Live metrics */}
      <div className="grid grid-cols-3 gap-2 mb-3">
        {[
          { label: "ωz", sub: "rad/s", val: wz !== null ? wz.toFixed(3) : "—", color: "text-blue-400" },
          { label: "|ω|", val: wz !== null ? Math.abs(wz).toFixed(3) : "—", color: "text-blue-400" },
          { label: "Status", val: ST_LABEL[imuSt], color: ST_STYLE[imuSt] },
        ].map(({ label, sub, val, color }) => (
          <div key={label} className="bg-slate-950/60 border border-slate-800 rounded-lg px-2.5 py-2">
            <span className="block text-[0.57rem] font-bold uppercase tracking-wider text-slate-600 mb-1">
              {label}{sub && <span className="ml-1 normal-case font-normal">{sub}</span>}
            </span>
            <span className={`text-base font-semibold tabular-nums ${color}`}>{val}</span>
          </div>
        ))}
      </div>

      {/* Correction visualizer – dáta z /motor_hat_node/drive_debug */}
      {/* Layout: [0]=pwm_l%, [1]=pwm_r%, [2]=left_cmd, [3]=right_cmd, [4]=imu_corr, [5]=imu_yaw_rate */}
      {(() => {
        const pwmL    = dbg ? dbg[0] : 0;
        const pwmR    = dbg ? dbg[1] : 0;
        const imuCorr = dbg ? dbg[4] : 0;
        const yawRate = dbg ? dbg[5] : (wz ?? 0);
        const active  = dbg ? Math.abs(dbg[4]) > 0.001 : false;
        // bar: yawRate ±0.5 rad/s → ±50%
        const barPct  = Math.min(Math.abs(yawRate) / 0.5, 1) * 50;
        const barLeft = yawRate < 0;
        const leftDown  = active && imuCorr > 0;
        const rightDown = active && imuCorr < 0;
        const hasDbg    = dbg !== null;
        return (
          <div className="bg-slate-950/60 border border-slate-800 rounded-lg px-3 py-2.5 mb-3 space-y-2">
            {/* Header */}
            <div className="flex items-center justify-between">
              <span className="text-[0.57rem] font-bold uppercase tracking-wider text-slate-600">
                Correction {hasDbg ? "(live)" : "(no debug topic)"}
              </span>
              <span className={`text-[0.6rem] font-bold uppercase tracking-wider px-1.5 py-0.5 rounded-full ${
                active ? "text-emerald-400 bg-emerald-950/50" : "text-slate-600 bg-slate-800/50"
              }`}>{active ? "Active" : "Off"}</span>
            </div>

            {/* yaw_rate bar */}
            <div className="relative h-2 bg-slate-800 rounded-full overflow-hidden">
              <div className="absolute top-0 bottom-0 w-px bg-slate-600 left-1/2" />
              <div className={`absolute top-0 bottom-0 rounded-full transition-all ${active ? "bg-blue-400" : "bg-slate-600"}`}
                style={{ width: barPct + "%", left: barLeft ? (50 - barPct) + "%" : "50%" }} />
            </div>

            {/* Wheel PWM + correction */}
            <div className="grid grid-cols-3 gap-1 items-center text-center">
              <div className={`text-xs font-bold rounded py-1 transition-colors ${
                leftDown  ? "text-amber-400 bg-amber-950/40" : "text-slate-500 bg-slate-800/40"
              }`}>
                L {hasDbg ? Math.round(pwmL) + "%" : "—"} {active ? (leftDown ? "▼" : "▲") : ""}
              </div>
              <div className="text-[0.62rem] tabular-nums text-slate-500 font-mono">
                {imuCorr !== 0 ? (imuCorr > 0 ? "+" : "") + imuCorr.toFixed(3) : "±0"}
              </div>
              <div className={`text-xs font-bold rounded py-1 transition-colors ${
                rightDown ? "text-amber-400 bg-amber-950/40" : "text-slate-500 bg-slate-800/40"
              }`}>
                R {hasDbg ? Math.round(pwmR) + "%" : "—"} {active ? (rightDown ? "▼" : "▲") : ""}
              </div>
            </div>
          </div>
        );
      })()}

      {/* Divider */}
      <div className="border-t border-slate-800 -mx-3 mb-4" />

      {/* PID header */}
      <div className="flex flex-wrap items-center justify-between gap-2 mb-3">
        <span className="text-[0.6rem] font-bold uppercase tracking-[0.1em] text-slate-500">PID Parameters</span>
        {/* Toggle */}
        <label className="flex items-center gap-2 cursor-pointer select-none">
          <span className="text-sm text-slate-300">IMU Correction</span>
          <div className="relative w-9 h-5 shrink-0">
            <input
              type="checkbox"
              className="sr-only peer"
              checked={pid.imu_correction}
              onChange={e => setPid(p => ({ ...p, imu_correction: e.target.checked }))}
            />
            <div className="absolute inset-0 rounded-full bg-slate-700 border border-slate-600 transition-all
                            peer-checked:bg-emerald-950/60 peer-checked:border-emerald-500" />
            <div className="absolute top-0.5 left-0.5 w-4 h-4 rounded-full bg-slate-400 pointer-events-none
                            transition-transform peer-checked:translate-x-4 peer-checked:bg-emerald-400" />
          </div>
        </label>
      </div>

      {/* PID inputs */}
      <div className="grid grid-cols-2 sm:grid-cols-3 gap-2.5 mb-4">
        <NumField name="imu_kp"       label="Kp"         step={0.01}  />
        <NumField name="imu_ki"       label="Ki"         step={0.01}  />
        <NumField name="imu_kd"       label="Kd"         step={0.001} />
        <NumField name="imu_deadband" label="Deadband"   step={0.001} />
        <NumField name="imu_windup"   label="Int. Limit" step={0.05}  />
      </div>

      {/* Footer */}
      <div className="flex items-center justify-between gap-2 flex-wrap">
        <div className="flex gap-3 flex-wrap">
          {fetchMsg && <span className={`text-xs ${fetchMsg.includes("Error") ? "text-red-400" : "text-emerald-400"}`}>{fetchMsg}</span>}
          {applyMsg && <span className={`text-xs ${applyMsg.includes("Error") ? "text-red-400" : "text-emerald-400"}`}>{applyMsg}</span>}
        </div>
        <div className="flex gap-2 shrink-0">
          <button
            onClick={fetchParams} disabled={!connected || fetchBusy}
            className="px-4 py-1.5 rounded-lg text-sm font-semibold bg-slate-800 text-slate-300
                       border border-slate-700 hover:bg-slate-700 active:scale-95 disabled:opacity-40
                       transition-all touch-manipulation"
          >{fetchBusy ? "…" : "Fetch"}</button>
          <button
            onClick={applyParams} disabled={!connected || applyBusy}
            className="px-4 py-1.5 rounded-lg text-sm font-semibold bg-blue-500 text-white
                       hover:bg-blue-400 active:scale-95 disabled:opacity-40 transition-all touch-manipulation"
          >{applyBusy ? "…" : "Apply"}</button>
        </div>
      </div>
    </Card>
  );
}

// ─── App ──────────────────────────────────────────────────────────────────────
function App() {
  const { ros, connected, wsUrl, setWsUrl, connect, disconnect } = useRos();

  return (
    <div className="max-w-[960px] mx-auto px-3 sm:px-5 py-4 pb-10">
      <Header connected={connected} />
      <ConnectionCard wsUrl={wsUrl} setWsUrl={setWsUrl} connected={connected} connect={connect} disconnect={disconnect} />
      <ModeCard ros={ros} connected={connected} />
      <div className="grid grid-cols-2 gap-2 mb-2">
        <CameraPanel ros={ros} connected={connected} />
        <DrivePanel  ros={ros} connected={connected} />
      </div>
      <ImuPanel ros={ros} connected={connected} />
    </div>
  );
}

ReactDOM.createRoot(document.getElementById("root")).render(<App />);
