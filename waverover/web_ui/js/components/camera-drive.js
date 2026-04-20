/* global window, ROSLIB, React */
"use strict";
// Kamera: predvolene WebRTC (SRTP/UDP) cez webrtc_camera_node; fallback rosbridge + canvas (TCP).
// genRef zabranuje race pri reconnect.

var WR = window.WR;

function CameraPanel({ ros, connected }) {
  const canvasRef = React.useRef(null);
  const videoRef = React.useRef(null);
  const pcRef = React.useRef(null);
  const [live, setLive] = React.useState(false);
  const [transport, setTransport] = React.useState("none"); // none | webrtc | ros
  const genRef = React.useRef(0);
  const subRef = React.useRef(null);

  React.useEffect(() => {
    if (!ros || !connected) {
      genRef.current++;
      setLive(false);
      setTransport("none");
      if (pcRef.current) {
        try { pcRef.current.close(); } catch (_) {}
        pcRef.current = null;
      }
      if (videoRef.current) videoRef.current.srcObject = null;
      if (subRef.current) {
        try { subRef.current.unsubscribe(); } catch (_) {}
        subRef.current = null;
      }
      return;
    }

    const gen = ++genRef.current;
    let cancelled = false;

    const startRosbridgeCanvas = () => {
      let primed = false;
      let frameSeq = 0;
      const sub = new ROSLIB.Topic({ ros, name: WR.CAMERA_TOPIC, messageType: WR.CAMERA_TYPE, throttle_rate: 33 });
      subRef.current = sub;
      setTransport("ros");

      sub.subscribe((m) => {
        if (gen !== genRef.current || cancelled) return;
        const seq = ++frameSeq;
        let url;
        let revoke = null;
        try {
          if (typeof m.data === "string") {
            url = "data:image/jpeg;base64," + m.data;
          } else {
            const b = new Blob([new Uint8Array(m.data)], { type: "image/jpeg" });
            url = URL.createObjectURL(b);
            revoke = url;
          }
        } catch (_) {
          return;
        }
        const img = new Image();
        img.decoding = "async";
        img.onload = () => {
          if (gen !== genRef.current || cancelled || seq !== frameSeq) { if (revoke) URL.revokeObjectURL(revoke); return; }
          const canvas = canvasRef.current;
          if (!canvas) { if (revoke) URL.revokeObjectURL(revoke); return; }
          const ctx = canvas.getContext("2d");
          const par = canvas.parentElement;
          const w = Math.max(1, par.clientWidth);
          const h = Math.max(1, par.clientHeight);
          if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
          const s = Math.min(w / img.naturalWidth, h / img.naturalHeight);
          const dw = img.naturalWidth * s;
          const dh = img.naturalHeight * s;
          ctx.fillStyle = "#030508";
          ctx.fillRect(0, 0, w, h);
          ctx.drawImage(img, (w - dw) * 0.5, (h - dh) * 0.5, dw, dh);
          if (revoke) URL.revokeObjectURL(revoke);
          if (!primed) { primed = true; setLive(true); }
        };
        img.onerror = () => { if (revoke) URL.revokeObjectURL(revoke); };
        img.src = url;
      });
    };

    (async () => {
      if (WR.USE_WEBRTC_CAMERA && typeof RTCPeerConnection !== "undefined") {
        const base = WR.webrtcSignalBaseUrl();
        try {
          const h = await fetch(`${base}/health`, { method: "GET", cache: "no-store", mode: "cors" });
          if (!h.ok) throw new Error("health");
          const pc = new RTCPeerConnection({
            iceServers: [{ urls: "stun:stun.l.google.com:19302" }],
          });
          pc.ontrack = (ev) => {
            if (cancelled || gen !== genRef.current) return;
            if (videoRef.current) videoRef.current.srcObject = ev.streams[0];
            setLive(true);
          };
          pc.addTransceiver("video", { direction: "recvonly" });
          const offer = await pc.createOffer();
          await pc.setLocalDescription(offer);
          const res = await fetch(`${base}/offer`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type }),
            mode: "cors",
          });
          if (!res.ok) throw new Error("offer");
          const answer = await res.json();
          await pc.setRemoteDescription({ type: answer.type, sdp: answer.sdp });
          if (cancelled || gen !== genRef.current) {
            pc.close();
            return;
          }
          pcRef.current = pc;
          setTransport("webrtc");
          return;
        } catch (e) {
          console.warn("WebRTC camera unavailable, using rosbridge", e);
        }
      }
      if (cancelled || gen !== genRef.current) return;
      startRosbridgeCanvas();
    })();

    return () => {
      cancelled = true;
      genRef.current++;
      if (pcRef.current) {
        try { pcRef.current.close(); } catch (_) {}
        pcRef.current = null;
      }
      if (videoRef.current) videoRef.current.srcObject = null;
      try { subRef.current?.unsubscribe(); } catch (_) {}
      subRef.current = null;
      setLive(false);
      setTransport("none");
    };
  }, [ros, connected]);

  const showFeed = live && (transport === "webrtc" || transport === "ros");

  return (
    <div className="bg-slate-900 border border-slate-800 rounded-xl p-2.5 shadow-lg shadow-black/40 flex flex-col">
      <div className="flex items-center justify-between mb-2">
        <WR.SectionLabel>Camera</WR.SectionLabel>
        <span className={`text-[0.58rem] font-bold uppercase tracking-wider px-2 py-0.5 rounded-full border transition-colors ${
          live ? "text-emerald-400 border-emerald-500/40 bg-emerald-950/40" : "text-slate-500 border-slate-700"
        }`}>{live ? "Live" : "Standby"}</span>
      </div>
      <div className="rounded-lg p-[3px] bg-gradient-to-br from-blue-500/20 via-slate-900 to-emerald-500/10">
        <div className="relative rounded-md overflow-hidden aspect-[4/3] bg-slate-950 flex items-center justify-center">
          {!showFeed && (
            <div className="absolute inset-0 flex flex-col items-center justify-center gap-2 text-slate-600 pointer-events-none z-0">
              <span className="text-3xl opacity-40">◉</span>
              <span className="text-xs text-center max-w-[8rem]">Connect to show the feed</span>
            </div>
          )}
          <video
            ref={videoRef}
            className={`absolute inset-0 w-full h-full object-contain bg-[#030508] z-10 ${transport === "webrtc" && showFeed ? "" : "hidden"}`}
            autoPlay
            playsInline
            muted
            aria-label="Robot camera WebRTC"
          />
          <canvas
            ref={canvasRef}
            className={`absolute inset-0 w-full h-full block bg-[#030508] z-10 ${transport === "ros" && showFeed ? "" : "hidden"}`}
            aria-label="Robot camera feed"
          />
        </div>
      </div>
    </div>
  );
}

function DrivePanel({ ros, connected }) {
  const [linScale, setLinScale] = React.useState(() => {
    const v = parseFloat(localStorage.getItem("waverover_web_speed_scale_lin"));
    return Number.isFinite(v)
      ? WR.snap(WR.clamp(v, WR.SLIDER_SCALE.min, WR.SLIDER_SCALE.max), WR.SLIDER_SCALE.step)
      : 0.5;
  });
  const [angScale, setAngScale] = React.useState(() => {
    const v = parseFloat(localStorage.getItem("waverover_web_speed_scale_ang"));
    return Number.isFinite(v)
      ? WR.snap(WR.clamp(v, WR.SLIDER_SCALE.min, WR.SLIDER_SCALE.max), WR.SLIDER_SCALE.step)
      : 0.45;
  });
  const [teleopMax, setTeleopMax] = React.useState({ lin: 0.5, ang: 1.8 });
  const [wanderThresholdM, setWanderThresholdM] = React.useState(() => {
    const v = parseFloat(localStorage.getItem("waverover_wander_threshold_m"));
    return Number.isFinite(v)
      ? WR.snap(WR.clamp(v, WR.WANDER_THRESHOLD_RANGE.min, WR.WANDER_THRESHOLD_RANGE.max), WR.WANDER_THRESHOLD_RANGE.step)
      : 0.30;
  });
  const teleopMaxRef = React.useRef({ lin: 0.5, ang: 1.8 });
  const twistRef = React.useRef({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
  const pubRef = React.useRef(null);
  const cmdRef = React.useRef(null);
  const linScaleRef = React.useRef(linScale);
  const angScaleRef = React.useRef(angScale);
  const syncRef = React.useRef(null);
  linScaleRef.current = linScale;
  angScaleRef.current = angScale;
  teleopMaxRef.current = teleopMax;

  React.useEffect(() => {
    if (!ros || !connected) return;
    let cancelled = false;
    (async () => {
      try {
        const r = await WR.rosGetParams(ros, WR.MOTOR_NODE, [
          "teleop_max_linear",
          "teleop_max_angular",
        ]);
        if (cancelled || !r?.values) return;
        const next = { lin: 1.0, ang: 2.0 };
        const names = ["teleop_max_linear", "teleop_max_angular"];
        names.forEach((name, i) => {
          const v = r.values[i];
          if (!v || v.type !== WR.PTYPE_DOUBLE) return;
          if (name === "teleop_max_linear" && v.double_value >= 0.05) next.lin = v.double_value;
          if (name === "teleop_max_angular" && v.double_value >= 0.1) next.ang = v.double_value;
        });
        teleopMaxRef.current = next;
        setTeleopMax(next);
      } catch (_) {}
    })();
    (async () => {
      try {
        const r = await WR.rosGetParams(ros, WR.WANDER_NODE, ["threshold_m"]);
        if (cancelled || !r?.values?.[0] || r.values[0].type !== WR.PTYPE_DOUBLE) return;
        const t = r.values[0].double_value;
        if (t >= WR.WANDER_THRESHOLD_RANGE.min && t <= WR.WANDER_THRESHOLD_RANGE.max) {
          const snapped = WR.snap(t, WR.WANDER_THRESHOLD_RANGE.step);
          setWanderThresholdM(snapped);
        }
      } catch (_) {}
    })();
    return () => { cancelled = true; };
  }, [ros, connected]);

  React.useEffect(() => {
    if (!ros || !connected) {
      if (pubRef.current) { clearInterval(pubRef.current); pubRef.current = null; }
      cmdRef.current = null;
      return;
    }
    cmdRef.current = new ROSLIB.Topic({ ros, name: WR.TELEOP_TOPIC, messageType: WR.TWIST_TYPE });
    pubRef.current = setInterval(() => cmdRef.current?.publish(new ROSLIB.Message(twistRef.current)), 1000 / WR.PUBLISH_HZ);
    return () => {
      if (pubRef.current) { clearInterval(pubRef.current); pubRef.current = null; }
      cmdRef.current = null;
    };
  }, [ros, connected]);

  React.useEffect(() => {
    if (!ros || !connected) return;
    if (syncRef.current) clearTimeout(syncRef.current);
    syncRef.current = setTimeout(async () => {
      try {
        const wanderFwd = linScaleRef.current * teleopMaxRef.current.lin;
        const wanderTurn = wanderFwd * WR.WANDER_TURN_RATIO;
        await WR.rosSetParams(ros, WR.WANDER_NODE, [
          WR.makeParam("forward_speed", wanderFwd),
          WR.makeParam("turn_speed", wanderTurn),
          WR.makeParam("threshold_m", wanderThresholdM),
        ]);
      } catch (_) {}
      syncRef.current = null;
    }, 250);
    return () => {
      if (syncRef.current) {
        clearTimeout(syncRef.current);
        syncRef.current = null;
      }
    };
  // teleopMax.ang sa v tele efektu nepouziva (len .lin cez ref) - odstraneho aby sa wander sync
  // netrieroval zbytocne pri zmene angular maxima (napr. po Fetch z robota).
  }, [linScale, wanderThresholdM, ros, connected, teleopMax.lin]);

  const stop = () => {
    twistRef.current = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
    cmdRef.current?.publish(new ROSLIB.Message(twistRef.current));
  };

  const drive = (lx, az) => {
    const { lin: maxL, ang: maxA } = teleopMaxRef.current;
    const lin = linScaleRef.current * maxL;
    const ang = angScaleRef.current * maxA;
    twistRef.current = {
      linear: { x: lx * lin, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: az * ang },
    };
    // Publish immediately on press to avoid startup hiccup
    cmdRef.current?.publish(new ROSLIB.Message(twistRef.current));
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

  const onLinSlider = e => {
    const v = WR.snap(WR.clamp(parseFloat(e.target.value), WR.SLIDER_SCALE.min, WR.SLIDER_SCALE.max), WR.SLIDER_SCALE.step);
    setLinScale(v);
    localStorage.setItem("waverover_web_speed_scale_lin", String(v));
  };
  const onAngSlider = e => {
    const v = WR.snap(WR.clamp(parseFloat(e.target.value), WR.SLIDER_SCALE.min, WR.SLIDER_SCALE.max), WR.SLIDER_SCALE.step);
    setAngScale(v);
    localStorage.setItem("waverover_web_speed_scale_ang", String(v));
  };
  const onWanderThrSlider = e => {
    const R = WR.WANDER_THRESHOLD_RANGE;
    const v = WR.snap(WR.clamp(parseFloat(e.target.value), R.min, R.max), R.step);
    setWanderThresholdM(v);
    localStorage.setItem("waverover_wander_threshold_m", String(v));
  };

  const maxLin = linScale * teleopMax.lin;
  const maxAng = angScale * teleopMax.ang;
  const linPct = Math.round(linScale * 100);
  const angPct = Math.round(angScale * 100);

  return (
    <div className="bg-slate-900 border border-slate-800 rounded-xl p-2.5 shadow-lg shadow-black/40 flex flex-col">
      <WR.SectionLabel>Drive</WR.SectionLabel>
      <div className="grid grid-cols-3 gap-1.5 w-full max-w-[11rem] mx-auto mt-1">
        <div /><DBtn lx={1} az={0} label="Forward">▲</DBtn><div />
        <DBtn lx={0} az={1} label="Left">◀</DBtn>
        <button
          className="aspect-square w-full rounded-xl font-bold text-white border border-black/20 select-none
                     bg-gradient-to-b from-red-600 to-red-800 active:scale-95 touch-manipulation transition-all text-xs"
          onClick={stop} aria-label="Stop"
        >■</button>
        <DBtn lx={0} az={-1} label="Right">▶</DBtn>
        <div /><DBtn lx={-1} az={0} label="Back">▼</DBtn><div />
      </div>
      <div className="mt-3 pt-3 border-t border-slate-800 space-y-2.5">
        <div>
          <div className="flex items-center justify-between mb-1">
            <span className="text-[0.6rem] font-bold uppercase tracking-wider text-slate-500">Linear scale</span>
            <span className="text-xs font-semibold text-blue-400 tabular-nums">{linPct}%</span>
          </div>
          <input type="range" min={WR.SLIDER_SCALE.min} max={WR.SLIDER_SCALE.max} step={WR.SLIDER_SCALE.step}
            value={linScale} onChange={onLinSlider} className="w-full" />
        </div>
        <div>
          <div className="flex items-center justify-between mb-1">
            <span className="text-[0.6rem] font-bold uppercase tracking-wider text-slate-500">Angular scale</span>
            <span className="text-xs font-semibold text-blue-400 tabular-nums">{angPct}%</span>
          </div>
          <input type="range" min={WR.SLIDER_SCALE.min} max={WR.SLIDER_SCALE.max} step={WR.SLIDER_SCALE.step}
            value={angScale} onChange={onAngSlider} className="w-full" />
        </div>
        <div className="flex items-center justify-between">
          <span className="text-[0.6rem] font-bold uppercase tracking-wider text-slate-500">Effective <span className="normal-case font-normal">lin/ang</span></span>
          <span className="text-xs text-slate-500 tabular-nums">{maxLin.toFixed(2)} / {maxAng.toFixed(2)}</span>
        </div>
        <div className="pt-2 border-t border-slate-800">
          <div className="flex items-center justify-between mb-1">
            <span className="text-[0.6rem] font-bold uppercase tracking-wider text-slate-500">Wander LiDAR stop</span>
            <span className="text-xs font-semibold text-amber-400/90 tabular-nums">{wanderThresholdM.toFixed(2)} m</span>
          </div>
          <input
            type="range"
            min={WR.WANDER_THRESHOLD_RANGE.min}
            max={WR.WANDER_THRESHOLD_RANGE.max}
            step={WR.WANDER_THRESHOLD_RANGE.step}
            value={wanderThresholdM}
            onChange={onWanderThrSlider}
            className="w-full"
            aria-label="LiDAR distance threshold for wander stop and turn"
          />
        </div>
      </div>
    </div>
  );
}

Object.assign(WR, { CameraPanel, DrivePanel });
