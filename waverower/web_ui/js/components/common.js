/* global window, React */
"use strict";

var WR = window.WR;

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

Object.assign(WR, { SectionLabel, Card, Header });
