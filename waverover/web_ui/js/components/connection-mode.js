/* global window, ROSLIB, React */
"use strict";
// WebSocket; shutdown cez /waverover/shutdown.

var WR = window.WR;

function ConnectionCard({ ros, wsUrl, setWsUrl, connected, connect, disconnect }) {
  const [shutdownBusy, setShutdownBusy] = React.useState(false);
  const [shutdownMsg, setShutdownMsg] = React.useState("");

  async function requestShutdown() {
    if (!connected || shutdownBusy) return;
    if (!window.confirm("Vypnut Raspberry Pi? (sudo systemctl poweroff)")) return;
    setShutdownBusy(true);
    setShutdownMsg("Vypinam...");
    try {
      const r = await WR.rosTrigger(ros, "/waverover/shutdown");
      setShutdownMsg(r?.success ? "Vypinanie..." : ("Chyba: " + (r?.message || "?")));
    } catch (e) {
      setShutdownMsg("Chyba: " + e);
    } finally {
      setShutdownBusy(false);
    }
  }

  return (
    <WR.Card className="p-3">
      <WR.SectionLabel>WebSocket</WR.SectionLabel>
      <div className="flex flex-wrap gap-2">
        <input
          type="text"
          value={wsUrl}
          placeholder={WR.defaultWsUrl()}
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
          <button
            onClick={requestShutdown}
            disabled={!connected || shutdownBusy}
            title="Regularny shutdown RPi (systemctl poweroff)"
            className="px-4 py-2 rounded-lg text-sm font-semibold bg-red-900/60 text-red-400
                       border border-red-800/50 hover:bg-red-900 active:scale-95 disabled:opacity-40
                       transition-all touch-manipulation"
            >{shutdownBusy ? "..." : "Shutdown"}</button>
        </div>
      </div>
      {shutdownMsg && (
        <p className={`mt-1.5 text-xs ${shutdownMsg.startsWith("Chyba") ? "text-red-400" : "text-amber-400"}`}>
          {shutdownMsg}
        </p>
      )}
    </WR.Card>
  );
}

function ModeCard({ ros, connected }) {
  const [mode, setMode] = React.useState("manual");
  const [status, setStatus] = React.useState("—");
  const [busy, setBusy] = React.useState(false);

  async function switchMode(m) {
    if (!connected || busy) return;
    setBusy(true);
    setStatus("Switching...");
    const isWander = m === "wander";
    try {
      const mr = await WR.rosSetParams(ros, WR.MOTOR_NODE, [
        WR.makeStringParam("control_mode", isWander ? "auto" : "manual"),
      ]);
      if (!mr?.results?.every(x => x.successful)) {
        const why = mr?.results?.find(r => !r.successful)?.reason || "motor set_parameters failed";
        throw new Error(why);
      }
      try {
        const wr = await WR.rosSetParams(ros, WR.WANDER_NODE, [WR.makeParam("enabled", isWander)]);
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
    <WR.Card className="flex flex-wrap items-center gap-2 px-3 py-2.5">
      <WR.SectionLabel>Mode</WR.SectionLabel>
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
    </WR.Card>
  );
}

Object.assign(WR, { ConnectionCard, ModeCard });
