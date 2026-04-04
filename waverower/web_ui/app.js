/* global ROSLIB */
(function () {
  const TELEOP_TOPIC = "/teleop_cmd_vel";
  const TWIST_TYPE = "geometry_msgs/msg/Twist";
  const CAMERA_TOPIC = "/camera/camera_node/image_raw/compressed";
  const CAMERA_TYPE = "sensor_msgs/msg/CompressedImage";
  const MOTOR_NODE = "/motor_hat_node";
  const RCL_DOUBLE = 3;
  const PUBLISH_HZ = 20;
  const WEB_TELEOP_SCALE_DIVISOR = 1.0;
  const LS_MAX_LIN = "waverower_web_max_lin";
  const LS_MAX_ANG = "waverower_web_max_ang";

  const DEFAULTS = {
    teleop_max_linear_m_s: 0.5,
    teleop_max_angular_rad_s: 1.8,
  };

  const el = {
    status: document.getElementById("status"),
    wsUrl: document.getElementById("wsUrl"),
    btnConnect: document.getElementById("btnConnect"),
    btnDisconnect: document.getElementById("btnDisconnect"),
    cam: document.getElementById("cam"),
    camPlaceholder: document.getElementById("camPlaceholder"),
    btnStop: document.getElementById("btnStop"),
    btnManual: document.getElementById("btnManual"),
    btnWander: document.getElementById("btnWander"),
    modeStatus: document.getElementById("modeStatus"),
    maxLin: document.getElementById("maxLin"),
    maxAng: document.getElementById("maxAng"),
    maxLinVal: document.getElementById("maxLinVal"),
    maxAngVal: document.getElementById("maxAngVal"),
  };

  let ros = null;
  let cmdVel = null;
  let camSub = null;
  let pubTimer = null;
  let currentTwist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
  let liveMaxLin =
    parseFloat(localStorage.getItem(LS_MAX_LIN), 10) ||
    parseFloat(el.maxLin.value, 10) ||
    DEFAULTS.teleop_max_linear_m_s;
  let liveMaxAng =
    parseFloat(localStorage.getItem(LS_MAX_ANG), 10) ||
    parseFloat(el.maxAng.value, 10) ||
    DEFAULTS.teleop_max_angular_rad_s;
  if (el.maxLin) el.maxLin.value = String(liveMaxLin);
  if (el.maxAng) el.maxAng.value = String(liveMaxAng);

  function makeDoubleParam(name, v) {
    const x = Number(v);
    return {
      name: name,
      value: { type: RCL_DOUBLE, double_value: x },
    };
  }

  function callSetParameters(nodeName, params, onOk, onErr) {
    if (!ros || !ros.isConnected) {
      onErr && onErr();
      return;
    }
    const svc = new ROSLIB.Service({
      ros: ros,
      name: nodeName + "/set_parameters",
      serviceType: "rcl_interfaces/srv/SetParameters",
    });
    const req = new ROSLIB.ServiceRequest({ parameters: params });
    svc.callService(
      req,
      function (res) {
        const results = res && res.results ? res.results : [];
        const ok =
          results.length === params.length &&
          results.every(function (r) {
            return r && (r.successful === true || r.successful === 1);
          });
        if (ok) {
          onOk && onOk(res);
        } else {
          const reason = results[0] && results[0].reason ? results[0].reason : "unknown";
          onErr && onErr(reason);
        }
      },
      function (err) {
        onErr && onErr(err);
      }
    );
  }

  function applyWebTeleopDivisorToMotor() {
    if (!ros || !ros.isConnected) return;
    const params = [
      makeDoubleParam("teleop_max_linear_m_s", WEB_TELEOP_SCALE_DIVISOR),
      makeDoubleParam("teleop_max_angular_rad_s", WEB_TELEOP_SCALE_DIVISOR),
    ];
    callSetParameters(MOTOR_NODE, params, function () {}, function () {});
  }

  function onMaxLinInput() {
    liveMaxLin = parseFloat(el.maxLin.value, 10);
    el.maxLinVal.textContent = liveMaxLin.toFixed(2);
    localStorage.setItem(LS_MAX_LIN, String(liveMaxLin));
  }

  function onMaxAngInput() {
    liveMaxAng = parseFloat(el.maxAng.value, 10);
    el.maxAngVal.textContent = liveMaxAng.toFixed(2);
    localStorage.setItem(LS_MAX_ANG, String(liveMaxAng));
  }

  function defaultWsUrl() {
    const h = window.location.hostname;
    if (h && h !== "localhost" && h !== "127.0.0.1") {
      return "ws://" + h + ":9090";
    }
    return "ws://" + (window.location.hostname || "127.0.0.1") + ":9090";
  }

  el.wsUrl.placeholder = defaultWsUrl();
  el.wsUrl.value = localStorage.getItem("waverower_ws_url") || defaultWsUrl();

  function subscribeCam() {
    if (camSub) {
      try {
        camSub.unsubscribe();
      } catch (e) {}
      camSub = null;
    }
    if (!ros) return;
    camSub = new ROSLIB.Topic({
      ros: ros,
      name: CAMERA_TOPIC,
      messageType: CAMERA_TYPE,
      throttle_rate: 80,
    });
    let lastT = 0;
    camSub.subscribe(function (m) {
      const now = Date.now();
      if (now - lastT < 80) return;
      lastT = now;
      try {
        let src;
        if (typeof m.data === "string") {
          src = "data:image/jpeg;base64," + m.data;
        } else {
          const bytes = new Uint8Array(m.data);
          const blob = new Blob([bytes], { type: "image/jpeg" });
          src = URL.createObjectURL(blob);
          if (el.cam._blobUrl) URL.revokeObjectURL(el.cam._blobUrl);
          el.cam._blobUrl = src;
        }
        el.cam.src = src;
        el.cam.classList.remove("hidden");
        el.camPlaceholder.classList.add("hidden");
      } catch (err) {
        console.warn("Camera frame error:", err, typeof m.data, m.data && m.data.length);
      }
    });
  }

  function setStatus(text, on) {
    el.status.textContent = text;
    el.status.classList.toggle("on", !!on);
    el.status.classList.toggle("off", !on);
  }

  function stopMotion() {
    currentTwist = {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    if (cmdVel && ros && ros.isConnected) {
      cmdVel.publish(new ROSLIB.Message(currentTwist));
    }
  }

  function publishLoop() {
    if (cmdVel && ros && ros.isConnected) {
      cmdVel.publish(new ROSLIB.Message(currentTwist));
    }
  }

  function connect() {
    const url = el.wsUrl.value.trim() || defaultWsUrl();
    localStorage.setItem("waverower_ws_url", url);

    if (ros) {
      try {
        ros.close();
      } catch (e) {}
    }

    ros = new ROSLIB.Ros({ url: url });

    ros.on("connection", function () {
      setStatus("Connected", true);
      cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: TELEOP_TOPIC,
        messageType: TWIST_TYPE,
      });
      if (pubTimer) clearInterval(pubTimer);
      pubTimer = setInterval(publishLoop, 1000 / PUBLISH_HZ);
      subscribeCam();
      applyWebTeleopDivisorToMotor();
    });

    ros.on("error", function (e) {
      setStatus("Rosbridge error", false);
      console.warn(e);
    });

    ros.on("close", function () {
      setStatus("Disconnected", false);
      if (pubTimer) {
        clearInterval(pubTimer);
        pubTimer = null;
      }
      cmdVel = null;
      if (camSub) {
        try {
          camSub.unsubscribe();
        } catch (e) {}
        camSub = null;
      }
    });
  }

  function disconnect() {
    stopMotion();
    if (pubTimer) {
      clearInterval(pubTimer);
      pubTimer = null;
    }
    if (ros) {
      try {
        ros.close();
      } catch (e) {}
      ros = null;
    }
    cmdVel = null;
    el.cam.classList.add("hidden");
    el.camPlaceholder.classList.remove("hidden");
    setStatus("Disconnected", false);
  }

  function callModeService(serviceName, activeBtn, inactiveBtn, modeLabel) {
    if (!ros || !ros.isConnected) {
      el.modeStatus.textContent = "Connect to rosbridge first";
      return;
    }
    el.modeStatus.textContent = "Switching…";
    activeBtn.disabled = true;
    inactiveBtn.disabled = true;
    const svc = new ROSLIB.Service({
      ros: ros,
      name: serviceName,
      serviceType: "std_srvs/srv/Trigger",
    });
    svc.callService(
      new ROSLIB.ServiceRequest({}),
      function (result) {
        activeBtn.disabled = false;
        inactiveBtn.disabled = false;
        if (result && result.success) {
          activeBtn.classList.add("active");
          inactiveBtn.classList.remove("active");
          el.modeStatus.textContent = modeLabel;
        } else {
          el.modeStatus.textContent = "Error: " + (result ? result.message : "no response");
        }
      },
      function (err) {
        activeBtn.disabled = false;
        inactiveBtn.disabled = false;
        el.modeStatus.textContent = "Service error: " + err;
      }
    );
  }

  function addTapListener(btn, handler) {
    btn.addEventListener("click", handler);
    btn.addEventListener("touchend", function (e) {
      e.preventDefault();
      handler();
    });
  }

  addTapListener(el.btnManual, function () {
    callModeService("/waverower/switch_to_manual", el.btnManual, el.btnWander, "Manual");
  });
  addTapListener(el.btnWander, function () {
    callModeService("/waverower/switch_to_wander", el.btnWander, el.btnManual, "Wander");
  });

  el.btnConnect.addEventListener("click", connect);
  el.btnDisconnect.addEventListener("click", disconnect);
  el.btnStop.addEventListener("click", stopMotion);

  el.maxLin.addEventListener("input", onMaxLinInput);
  el.maxAng.addEventListener("input", onMaxAngInput);

  el.maxLinVal.textContent = liveMaxLin.toFixed(2);
  el.maxAngVal.textContent = liveMaxAng.toFixed(2);

  document.querySelectorAll("button.drive").forEach(function (btn) {
    const lx = parseFloat(btn.getAttribute("data-lx"), 10) || 0;
    const az = parseFloat(btn.getAttribute("data-az"), 10) || 0;

    function apply() {
      currentTwist = {
        linear: { x: lx * liveMaxLin, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: az * liveMaxAng },
      };
    }

    btn.addEventListener("touchstart", function (e) {
      e.preventDefault();
      apply();
    });
    btn.addEventListener("mousedown", apply);
    btn.addEventListener("touchend", function (e) {
      e.preventDefault();
      stopMotion();
    });
    btn.addEventListener("mouseup", stopMotion);
    btn.addEventListener("mouseleave", stopMotion);
  });

  window.addEventListener("beforeunload", disconnect);
})();
