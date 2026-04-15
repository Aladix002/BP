/* global window, ROSLIB, React */
"use strict";
// Panel: omega_z z /imu, drive_debug z motora, get/set PID a correction_mode na motor_hat_node cez roslib services.

var WR = window.WR;

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
    (async () => {
      try {
        const r = await WR.rosGetParams(ros, WR.MOTOR_NODE, ["correction_mode"]);
        const v = r?.values?.[0];
        if (v?.type === WR.PTYPE_STRING) {
          setCorrectionMode(v.string_value === "optical_flow" ? "optical_flow" : "imu");
        }
      } catch (_) {}
    })();
  }, [connected, ros]);

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

      <div className="flex items-center justify-end">
        {modeMsg && <span className={`text-xs ${modeMsg.includes("Error") ? "text-red-400" : "text-emerald-400"}`}>{modeMsg}</span>}
      </div>
    </WR.Card>
  );
}

Object.assign(WR, { ImuPanel });
