/* global window, ROSLIB, React */
"use strict";

var WR = window.WR;

const clamp = (n, lo, hi) => Math.min(hi, Math.max(lo, n));
const snap = (n, step) => Math.round(n / step) * step;
const defaultWsUrl = () => `ws://${window.location.hostname || "127.0.0.1"}:9090`;

const makeParam = (name, value) =>
  typeof value === "boolean"
    ? { name, value: { type: WR.PTYPE_BOOL, bool_value: value } }
    : { name, value: { type: WR.PTYPE_DOUBLE, double_value: value } };

const makeStringParam = (name, value) => ({
  name,
  value: { type: WR.PTYPE_STRING, string_value: value },
});

const rosGetParams = (ros, node, names) =>
  new Promise((ok, err) =>
    new ROSLIB.Service({ ros, name: `${node}/get_parameters`, serviceType: "rcl_interfaces/srv/GetParameters" })
      .callService(new ROSLIB.ServiceRequest({ names }), ok, err));

const rosSetParams = (ros, node, parameters) =>
  new Promise((ok, err) =>
    new ROSLIB.Service({ ros, name: `${node}/set_parameters`, serviceType: "rcl_interfaces/srv/SetParameters" })
      .callService(new ROSLIB.ServiceRequest({ parameters }), ok, err));

const rosTrigger = (ros, service) =>
  new Promise((ok, err) =>
    new ROSLIB.Service({ ros, name: service, serviceType: "std_srvs/srv/Trigger" })
      .callService(new ROSLIB.ServiceRequest({}), ok, err));

function useRos() {
  const [ros, setRos] = React.useState(null);
  const [connected, setConnected] = React.useState(false);
  const [wsUrl, setWsUrl] = React.useState(
    () => localStorage.getItem("waverower_ws_url") || defaultWsUrl()
  );
  const rosRef = React.useRef(null);

  const connect = React.useCallback((url) => {
    const u = url || wsUrl;
    localStorage.setItem("waverower_ws_url", u);
    if (rosRef.current) {
      try { rosRef.current.close(); } catch (_) {}
    }
    const r = new ROSLIB.Ros({ url: u });
    rosRef.current = r;
    r.on("connection", () => { setRos(r); setConnected(true); });
    r.on("error", () => setConnected(false));
    r.on("close", () => { setConnected(false); setRos(null); rosRef.current = null; });
  }, [wsUrl]);

  const disconnect = React.useCallback(() => {
    if (rosRef.current) {
      try { rosRef.current.close(); } catch (_) {}
    }
  }, []);

  return { ros, connected, wsUrl, setWsUrl, connect, disconnect };
}

Object.assign(window.WR, {
  clamp,
  snap,
  defaultWsUrl,
  makeParam,
  makeStringParam,
  rosGetParams,
  rosSetParams,
  rosTrigger,
  useRos,
});
