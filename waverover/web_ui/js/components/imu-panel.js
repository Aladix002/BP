/* global window, ROSLIB, React */
"use strict";
// IMU + motor PID parametre cez roslib.

var WR = window.WR;

const IMU_TUNE_KEYS = ["imu_kp", "imu_ki", "imu_kd", "imu_deadband"];
const FLOW_TUNE_KEYS = ["correction_gain", "max_correction", "forward_threshold", "steer_deadzone"];

const IMU_TUNE_LABELS = {
  imu_kp: "Kp",
  imu_ki: "Ki",
  imu_kd: "Kd",
  imu_deadband: "Deadband rad/s",
};
const FLOW_TUNE_LABELS = {
  correction_gain: "Gain",
  max_correction: "Max |corr.|",
  forward_threshold: "Forward thresh.",
  steer_deadzone: "Steer deadzone",
};

const IMU_TUNE_STEPS = {
  imu_kp: 0.01, imu_ki: 0.005, imu_kd: 0.005, imu_deadband: 0.005,
};
const FLOW_TUNE_STEPS = {
  correction_gain: 0.1, max_correction: 0.05, forward_threshold: 0.01, steer_deadzone: 0.02,
};

function readParamDouble(pv) {
  if (!pv) return null;
  if (pv.type === WR.PTYPE_DOUBLE && typeof pv.double_value === "number") return pv.double_value;
  if (pv.double_value != null) return parseFloat(pv.double_value);
  return null;
}

function ImuPanel({ ros, connected }) {
  const [wz, setWz] = React.useState(null);
  const [imuSt, setImuSt] = React.useState("disconnected");
  const lastMsRef = React.useRef(0);
  const subRef = React.useRef(null);
  const timerRef = React.useRef(null);

  const [dbg, setDbg] = React.useState(null);
  const dbgSubRef = React.useRef(null);

  const [correctionMode, setCorrectionMode] = React.useState("imu");
  const [modeMsg, setModeMsg] = React.useState("");
  const [modeBusy, setModeBusy] = React.useState(false);

  const [imuTune, setImuTune] = React.useState({
    imu_kp: 0.3, imu_ki: 0.05, imu_kd: 0.01, imu_deadband: 0.02,
  });
  const [flowTune, setFlowTune] = React.useState({
    correction_gain: 1.5, max_correction: 0.3, forward_threshold: 0.05, steer_deadzone: 0.12,
  });
  const [tuneMsg, setTuneMsg] = React.useState("");
  const [imuTuneBusy, setImuTuneBusy] = React.useState(false);
  const [flowTuneBusy, setFlowTuneBusy] = React.useState(false);

  React.useEffect(() => {
    if (!ros || !connected) {
      lastMsRef.current = 0;
      setWz(null);
      setImuSt("disconnected");
      setDbg(null);
      if (subRef.current) { try { subRef.current.unsubscribe(); } catch (_) {} subRef.current = null; }
      if (dbgSubRef.current) { try { dbgSubRef.current.unsubscribe(); } catch (_) {} dbgSubRef.current = null; }
      if (timerRef.current) { clearInterval(timerRef.current); timerRef.current = null; }
      return;
    }

    const sub = new ROSLIB.Topic({ ros, name: WR.IMU_TOPIC, messageType: WR.IMU_TYPE });
    subRef.current = sub;
    sub.subscribe((msg) => {
      const av = msg.angular_velocity;
      let z = NaN;
      if (av && typeof av.z === "number") z = av.z;
      else if (av?.z != null) z = parseFloat(av.z);
      if (!Number.isFinite(z)) return;
      lastMsRef.current = Date.now();
      setWz(z);
    });

    const dbgSub = new ROSLIB.Topic({
      ros, name: WR.IMU_DBG_TOPIC, messageType: WR.IMU_DBG_TYPE, throttle_rate: 50,
    });
    dbgSubRef.current = dbgSub;
    dbgSub.subscribe((msg) => setDbg(msg.data));

    timerRef.current = setInterval(() => {
      const age = lastMsRef.current ? Date.now() - lastMsRef.current : 999999;
      if (!lastMsRef.current) setImuSt("waiting");
      else if (age > 1200) setImuSt("stale");
      else setImuSt("ok");
    }, 150);

    return () => {
      try { sub.unsubscribe(); } catch (_) {}
      subRef.current = null;
      try { dbgSub.unsubscribe(); } catch (_) {}
      dbgSubRef.current = null;
      clearInterval(timerRef.current);
      timerRef.current = null;
    };
  }, [ros, connected]);

  React.useEffect(() => {
    if (!connected || !ros) return;
    let cancelled = false;
    (async () => {
      try {
        const r = await WR.rosGetParams(ros, WR.MOTOR_NODE, ["correction_mode", ...IMU_TUNE_KEYS]);
        if (cancelled) return;
        const vals = r?.values || [];
        const cm = vals[0];
        if (cm?.type === WR.PTYPE_STRING) {
          setCorrectionMode(cm.string_value === "optical_flow" ? "optical_flow" : "imu");
        }
        const patch = {};
        IMU_TUNE_KEYS.forEach((k, i) => {
          const d = readParamDouble(vals[i + 1]);
          if (d != null) patch[k] = d;
        });
        if (Object.keys(patch).length) setImuTune((prev) => ({ ...prev, ...patch }));
      } catch (_) {}

      try {
        const r2 = await WR.rosGetParams(ros, WR.OPTICAL_NODE, FLOW_TUNE_KEYS);
        if (cancelled) return;
        const vals2 = r2?.values || [];
        const patch2 = {};
        FLOW_TUNE_KEYS.forEach((k, i) => {
          const d = readParamDouble(vals2[i]);
          if (d != null) patch2[k] = d;
        });
        if (Object.keys(patch2).length) setFlowTune((prev) => ({ ...prev, ...patch2 }));
      } catch (_) {}
    })();
    return () => { cancelled = true; };
  }, [connected, ros]);

  async function refreshTuneFromRobot() {
    if (!connected || !ros) return;
    setTuneMsg("Refreshing…");
    try {
      const r = await WR.rosGetParams(ros, WR.MOTOR_NODE, ["correction_mode", ...IMU_TUNE_KEYS]);
      const vals = r?.values || [];
      const cm = vals[0];
      if (cm?.type === WR.PTYPE_STRING) {
        setCorrectionMode(cm.string_value === "optical_flow" ? "optical_flow" : "imu");
      }
      const patch = {};
      IMU_TUNE_KEYS.forEach((k, i) => {
        const d = readParamDouble(vals[i + 1]);
        if (d != null) patch[k] = d;
      });
      setImuTune((prev) => ({ ...prev, ...patch }));
    } catch (e) {
      setTuneMsg("Motor read failed: " + (e.message ?? e));
      return;
    }
    try {
      const r2 = await WR.rosGetParams(ros, WR.OPTICAL_NODE, FLOW_TUNE_KEYS);
      const vals2 = r2?.values || [];
      const patch2 = {};
      FLOW_TUNE_KEYS.forEach((k, i) => {
        const d = readParamDouble(vals2[i]);
        if (d != null) patch2[k] = d;
      });
      setFlowTune((prev) => ({ ...prev, ...patch2 }));
      setTuneMsg("Parameters refreshed");
    } catch (e) {
      setTuneMsg("Optical flow node offline or unreadable (IMU OK)");
    }
  }

  async function applyImuTune() {
    if (!connected || !ros) return;
    setImuTuneBusy(true);
    setTuneMsg("");
    try {
      const parameters = IMU_TUNE_KEYS.map((k) => WR.makeParam(k, Number(imuTune[k])));
      const res = await WR.rosSetParams(ros, WR.MOTOR_NODE, parameters);
      if (!res?.results?.every((x) => x.successful)) {
        const why = res?.results?.find((r) => !r.successful)?.reason || "set_parameters failed";
        throw new Error(why);
      }
      setTuneMsg("IMU PID saved");
    } catch (e) {
      setTuneMsg("IMU save: " + (e.message ?? e));
    } finally {
      setImuTuneBusy(false);
    }
  }

  async function applyFlowTune() {
    if (!connected || !ros) return;
    setFlowTuneBusy(true);
    setTuneMsg("");
    try {
      const parameters = FLOW_TUNE_KEYS.map((k) => WR.makeParam(k, Number(flowTune[k])));
      const res = await WR.rosSetParams(ros, WR.OPTICAL_NODE, parameters);
      if (!res?.results?.every((x) => x.successful)) {
        const why = res?.results?.find((r) => !r.successful)?.reason || "set_parameters failed";
        throw new Error(why);
      }
      setTuneMsg("Optical flow saved");
    } catch (e) {
      setTuneMsg("Flow save: " + (e.message ?? e));
    } finally {
      setFlowTuneBusy(false);
    }
  }

  async function switchCorrectionMode(nextMode) {
    if (!connected || modeBusy || nextMode === correctionMode) return;
    setModeBusy(true);
    setModeMsg("Switching mode...");
    try {
      const enableImu = nextMode === "imu";
      const motorRes = await WR.rosSetParams(ros, WR.MOTOR_NODE, [
        WR.makeStringParam("correction_mode", nextMode),
        WR.makeParam("imu_correction", enableImu),
      ]);
      if (!motorRes?.results?.every(x => x.successful)) {
        const why = motorRes?.results?.find(r => !r.successful)?.reason || "motor set_parameters failed";
        throw new Error(why);
      }

      try {
        const flowRes = await WR.rosSetParams(ros, WR.OPTICAL_NODE, [WR.makeParam("enabled", !enableImu)]);
        if (!flowRes?.results?.every(x => x.successful)) {
          const why = flowRes?.results?.find(r => !r.successful)?.reason || "optical flow set_parameters failed";
          throw new Error(why);
        }
      } catch (e) {
        if (!enableImu) throw e;
      }

      setCorrectionMode(nextMode);
      setModeMsg(nextMode === "imu" ? "IMU correction active" : "Optical flow active");
    } catch (e) {
      setModeMsg("Error: " + (e.message ?? e));
    } finally {
      setModeBusy(false);
    }
  }

  const ST_STYLE = {
    ok: "text-emerald-400",
    stale: "text-amber-400",
    waiting: "text-slate-500",
    disconnected: "text-slate-600",
  };
  const ST_LABEL = { ok: "OK", stale: "No /imu", waiting: "Waiting...", disconnected: "Disconnected" };

  return (
    <WR.Card className="p-3">
      <WR.SectionLabel>IMU · Regulation</WR.SectionLabel>
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

      {(() => {
        const pwmL = dbg ? dbg[0] : 0;
        const pwmR = dbg ? dbg[1] : 0;
        const imuCorr = dbg ? dbg[4] : 0;
        const yawRate = dbg ? dbg[5] : (wz ?? 0);
        const active = dbg ? Math.abs(dbg[4]) > 0.001 : false;
        const barPct = Math.min(Math.abs(yawRate) / 0.5, 1) * 50;
        const barLeft = yawRate < 0;
        const leftDown = active && imuCorr > 0;
        const rightDown = active && imuCorr < 0;
        const hasDbg = dbg !== null;
        return (
          <div className="bg-slate-950/60 border border-slate-800 rounded-lg px-3 py-2.5 mb-3 space-y-2">
            <div className="flex items-center justify-between">
              <span className="text-[0.57rem] font-bold uppercase tracking-wider text-slate-600">
                Correction {hasDbg ? "(live)" : "(no debug topic)"}
              </span>
              <span className={`text-[0.6rem] font-bold uppercase tracking-wider px-1.5 py-0.5 rounded-full ${
                active ? "text-emerald-400 bg-emerald-950/50" : "text-slate-600 bg-slate-800/50"
              }`}>{active ? "Active" : "Off"}</span>
            </div>

            <div className="relative h-2 bg-slate-800 rounded-full overflow-hidden">
              <div className="absolute top-0 bottom-0 w-px bg-slate-600 left-1/2" />
              <div className={`absolute top-0 bottom-0 rounded-full transition-all ${active ? "bg-blue-400" : "bg-slate-600"}`}
                style={{ width: barPct + "%", left: barLeft ? (50 - barPct) + "%" : "50%" }} />
            </div>

            <div className="grid grid-cols-3 gap-1 items-center text-center">
              <div className={`text-xs font-bold rounded py-1 transition-colors ${
                leftDown ? "text-amber-400 bg-amber-950/40" : "text-slate-500 bg-slate-800/40"
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

      <div className="border-t border-slate-800 -mx-3 mb-4" />
      <div className="flex flex-wrap items-center justify-between gap-2 mb-3">
        <span className="text-[0.6rem] font-bold uppercase tracking-[0.1em] text-slate-500">Correction Mode</span>
        <div className="flex rounded-lg overflow-hidden border border-slate-700 shrink-0">
          {[
            ["imu", "IMU"],
            ["optical_flow", "Optical Flow"],
          ].map(([key, label], i) => (
            <button
              key={key}
              onClick={() => switchCorrectionMode(key)}
              disabled={!connected || modeBusy}
              className={`px-3 py-1.5 text-xs font-semibold transition-colors touch-manipulation disabled:opacity-40
                ${i > 0 ? "border-l border-slate-700" : ""}
                ${correctionMode === key
                  ? "bg-emerald-950/60 text-emerald-400"
                  : "bg-slate-800/50 text-slate-400 hover:bg-slate-800"
                }`}
            >{label}</button>
          ))}
        </div>
      </div>

      <div className="flex items-center justify-end gap-2 mb-2">
        {modeMsg && <span className={`text-xs ${modeMsg.includes("Error") ? "text-red-400" : "text-emerald-400"}`}>{modeMsg}</span>}
      </div>

      <div className="border-t border-slate-800 -mx-3 mb-3" />
      <div className="flex flex-wrap items-center justify-between gap-2 mb-2">
               <span className="block text-[0.6rem] font-bold uppercase tracking-[0.1em] text-slate-500">Tune · motor_hat_node (IMU PID)</span>
        <div className="flex gap-2">
          <button
            type="button"
            onClick={refreshTuneFromRobot}
            disabled={!connected}
            className="px-2.5 py-1 text-[0.65rem] font-semibold rounded-lg border border-slate-600 text-slate-300
              bg-slate-800/60 hover:bg-slate-800 disabled:opacity-40 touch-manipulation"
          >Refresh</button>
          <button
            type="button"
            onClick={applyImuTune}
            disabled={!connected || imuTuneBusy}
            className="px-2.5 py-1 text-[0.65rem] font-semibold rounded-lg border border-emerald-600/50 text-emerald-400
              bg-emerald-950/40 hover:bg-emerald-950/60 disabled:opacity-40 touch-manipulation"
          >Apply IMU</button>
        </div>
      </div>
      <div className="grid grid-cols-1 sm:grid-cols-2 gap-x-4 gap-y-1.5 mb-4">
        {IMU_TUNE_KEYS.map((key) => (
          <div key={key} className="flex items-center justify-between gap-2">
            <label htmlFor={`imu-${key}`} className="text-[0.65rem] text-slate-500 shrink-0">{IMU_TUNE_LABELS[key]}</label>
            <input
              id={`imu-${key}`}
              type="number"
              step={IMU_TUNE_STEPS[key]}
              value={imuTune[key]}
              onChange={(e) => {
                const v = parseFloat(e.target.value);
                setImuTune((p) => ({ ...p, [key]: Number.isFinite(v) ? v : p[key] }));
              }}
              className="w-[6rem] bg-slate-950 border border-slate-700 rounded-md px-2 py-1 text-sm text-slate-200 text-right tabular-nums"
            />
          </div>
        ))}
      </div>

      <div className="border-t border-slate-800 -mx-3 mb-3" />
      <div className="flex flex-wrap items-center justify-between gap-2 mb-2">
        <span className="block text-[0.6rem] font-bold uppercase tracking-[0.1em] text-slate-500">Tune · optical_flow_node</span>
        <button
          type="button"
          onClick={applyFlowTune}
          disabled={!connected || flowTuneBusy}
          className="px-2.5 py-1 text-[0.65rem] font-semibold rounded-lg border border-sky-600/50 text-sky-400
            bg-sky-950/30 hover:bg-sky-950/50 disabled:opacity-40 touch-manipulation"
        >Apply flow</button>
      </div>
      <div className="grid grid-cols-1 sm:grid-cols-2 gap-x-4 gap-y-1.5 mb-2">
        {FLOW_TUNE_KEYS.map((key) => (
          <div key={key} className="flex items-center justify-between gap-2">
            <label htmlFor={`flow-${key}`} className="text-[0.65rem] text-slate-500 shrink-0">{FLOW_TUNE_LABELS[key]}</label>
            <input
              id={`flow-${key}`}
              type="number"
              step={FLOW_TUNE_STEPS[key]}
              value={flowTune[key]}
              onChange={(e) => {
                const v = parseFloat(e.target.value);
                setFlowTune((p) => ({ ...p, [key]: Number.isFinite(v) ? v : p[key] }));
              }}
              className="w-[6rem] bg-slate-950 border border-slate-700 rounded-md px-2 py-1 text-sm text-slate-200 text-right tabular-nums"
            />
          </div>
        ))}
      </div>

      {tuneMsg && (
        <div className={`text-xs mt-1 ${tuneMsg.includes("failed") || tuneMsg.includes("Error") || tuneMsg.includes("save:") ? "text-amber-400" : "text-slate-400"}`}>
          {tuneMsg}
        </div>
      )}
    </WR.Card>
  );
}

Object.assign(WR, { ImuPanel });
