/* global ROSLIB */
(function () {
  const TELEOP_TOPIC = "/teleop_cmd_vel";
  const TWIST_TYPE = "geometry_msgs/msg/Twist";
  const CAMERA_TOPIC = "/camera/camera_node/image_raw/compressed";
  const CAMERA_TYPE = "sensor_msgs/msg/CompressedImage";

  const MOTOR_NODE = "/motor_hat_node";
  /** Lucas–Kanade vs Farnebäck majú rôzne meno uzla – skúsime obe. */
  const FLOW_NODE_CANDIDATES = ["/optical_flow_node", "/optical_flow_dense_node"];

  /** @see rcl_interfaces/msg/ParameterType.PARAMETER_DOUBLE */
  const RCL_DOUBLE = 3;

  const PUBLISH_HZ = 20;

  const DEFAULTS = {
    teleop_max_linear_m_s: 0.5,
    teleop_max_angular_rad_s: 1.8,
    imu_yaw_kp: 0.15,
    imu_yaw_ki: 0.05,
    imu_yaw_kd: 0.01,
    imu_yaw_deadband: 0.02,
    imu_yaw_integral_limit: 0.3,
    correction_gain: 1.5,
    max_correction: 0.3,
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
    imuKp: document.getElementById("imuKp"),
    imuKi: document.getElementById("imuKi"),
    imuKd: document.getElementById("imuKd"),
    imuDb: document.getElementById("imuDb"),
    imuIl: document.getElementById("imuIl"),
    flowGain: document.getElementById("flowGain"),
    flowMax: document.getElementById("flowMax"),
    btnApplyPid: document.getElementById("btnApplyPid"),
    btnApplyFlow: document.getElementById("btnApplyFlow"),
    settingsStatus: document.getElementById("settingsStatus"),
  };

  let ros = null;
  let cmdVel = null;
  let camSub = null;
  let pubTimer = null;
  let currentTwist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
  /** Aktuálne max. rýchlosti (zhodné s motorom po sync) */
  let liveMaxLin = parseFloat(el.maxLin.value, 10) || DEFAULTS.teleop_max_linear_m_s;
  let liveMaxAng = parseFloat(el.maxAng.value, 10) || DEFAULTS.teleop_max_angular_rad_s;
  let teleopSyncTimer = null;

  function setSettingsStatus(msg, ok) {
    if (!el.settingsStatus) return;
    el.settingsStatus.textContent = msg || "";
    el.settingsStatus.style.color = ok ? "var(--ok, #3ecf8e)" : "var(--muted, #8b9bb4)";
  }

  function makeDoubleParam(name, v) {
    const x = Number(v);
    return {
      name: name,
      value: { type: RCL_DOUBLE, double_value: x },
    };
  }

  function looksLikeMissingService(err) {
    const s = String(err);
    return /does not exist|not advertise|unknown service|404/i.test(s);
  }

  /**
   * Skúša set_parameters postupne na uzloch; pri „service neexistuje“ skúsi ďalší.
   */
  function callSetParametersFirstMatch(nodeNames, params, onOk, onErr) {
    const msgNone =
      "Žiadny optical flow uzol nebeží. Spusti napr.: ros2 launch waverower runtime_stack.launch.py correction_mode:=optical_flow use_camera:=true use_web:=true";
    function attempt(at) {
      if (at >= nodeNames.length) {
        onErr && onErr(msgNone);
        return;
      }
      callSetParameters(
        nodeNames[at],
        params,
        onOk,
        function (err) {
          if (looksLikeMissingService(err) && at + 1 < nodeNames.length) {
            attempt(at + 1);
          } else if (looksLikeMissingService(err)) {
            onErr && onErr(msgNone);
          } else {
            onErr && onErr(err);
          }
        }
      );
    }
    attempt(0);
  }

  function callGetParametersForNode(nodeName, names, onOk, onErr) {
    if (!ros || !ros.isConnected) return;
    const svc = new ROSLIB.Service({
      ros: ros,
      name: nodeName + "/get_parameters",
      serviceType: "rcl_interfaces/srv/GetParameters",
    });
    const req = new ROSLIB.ServiceRequest({ names: names });
    svc.callService(req, onOk, onErr);
  }

  function fetchFlowParametersFirstMatch() {
    const names = ["correction_gain", "max_correction"];
    function attempt(at) {
      if (at >= FLOW_NODE_CANDIDATES.length) return;
      callGetParametersForNode(
        FLOW_NODE_CANDIDATES[at],
        names,
        function (res) {
          const map = parseGetParametersResponse(res);
          if (map.correction_gain != null || map.max_correction != null) {
            if (map.correction_gain != null) el.flowGain.value = String(map.correction_gain);
            if (map.max_correction != null) el.flowMax.value = String(map.max_correction);
          } else if (at + 1 < FLOW_NODE_CANDIDATES.length) {
            attempt(at + 1);
          }
        },
        function () {
          if (at + 1 < FLOW_NODE_CANDIDATES.length) attempt(at + 1);
        }
      );
    }
    attempt(0);
  }

  function callSetParameters(nodeName, params, onOk, onErr) {
    if (!ros || !ros.isConnected) {
      setSettingsStatus("Najprv pripojenie k rosbridge", false);
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

  function extractDouble(pv) {
    if (!pv || typeof pv !== "object") return null;
    if (pv.type === RCL_DOUBLE && typeof pv.double_value === "number") return pv.double_value;
    if (typeof pv.double_value === "number") return pv.double_value;
    return null;
  }

  function applyParamsToForm(map) {
    function setNum(id, key, fallback) {
      const v = map[key];
      const eln = el[id];
      if (!eln) return;
      const n = typeof v === "number" && !isNaN(v) ? v : fallback;
      eln.value = String(n);
    }
    if (map.teleop_max_linear_m_s != null) {
      liveMaxLin = map.teleop_max_linear_m_s;
      el.maxLin.value = String(liveMaxLin);
      el.maxLinVal.textContent = Number(liveMaxLin).toFixed(2);
    }
    if (map.teleop_max_angular_rad_s != null) {
      liveMaxAng = map.teleop_max_angular_rad_s;
      el.maxAng.value = String(liveMaxAng);
      el.maxAngVal.textContent = Number(liveMaxAng).toFixed(2);
    }
    setNum("imuKp", "imu_yaw_kp", DEFAULTS.imu_yaw_kp);
    setNum("imuKi", "imu_yaw_ki", DEFAULTS.imu_yaw_ki);
    setNum("imuKd", "imu_yaw_kd", DEFAULTS.imu_yaw_kd);
    setNum("imuDb", "imu_yaw_deadband", DEFAULTS.imu_yaw_deadband);
    setNum("imuIl", "imu_yaw_integral_limit", DEFAULTS.imu_yaw_integral_limit);
    setNum("flowGain", "correction_gain", DEFAULTS.correction_gain);
    setNum("flowMax", "max_correction", DEFAULTS.max_correction);
  }

  function parseGetParametersResponse(res) {
    const map = {};
    const list = res && res.values ? res.values : [];
    for (let i = 0; i < list.length; i++) {
      const p = list[i];
      if (!p || !p.name) continue;
      const d = extractDouble(p.value);
      if (d != null) map[p.name] = d;
    }
    return map;
  }

  function fetchMotorParameters() {
    if (!ros || !ros.isConnected) return;
    const names = [
      "teleop_max_linear_m_s",
      "teleop_max_angular_rad_s",
      "imu_yaw_kp",
      "imu_yaw_ki",
      "imu_yaw_kd",
      "imu_yaw_deadband",
      "imu_yaw_integral_limit",
    ];
    const svc = new ROSLIB.Service({
      ros: ros,
      name: MOTOR_NODE + "/get_parameters",
      serviceType: "rcl_interfaces/srv/GetParameters",
    });
    const req = new ROSLIB.ServiceRequest({ names: names });
    svc.callService(
      req,
      function (res) {
        const map = parseGetParametersResponse(res);
        if (Object.keys(map).length === 0) {
          applyParamsToForm(DEFAULTS);
          setSettingsStatus("Parametre načítané (predvolené – get_parameters nevrátil dáta)", false);
          return;
        }
        applyParamsToForm(map);
        setSettingsStatus("Parametre motora načítané", true);
      },
      function () {
        applyParamsToForm(DEFAULTS);
        setSettingsStatus("Nepodarilo sa načítať parametre (predvolené)", false);
      }
    );

    fetchFlowParametersFirstMatch();
  }

  function scheduleTeleopSyncToMotor() {
    if (teleopSyncTimer) clearTimeout(teleopSyncTimer);
    teleopSyncTimer = setTimeout(function () {
      teleopSyncTimer = null;
      if (!ros || !ros.isConnected) return;
      const params = [
        makeDoubleParam("teleop_max_linear_m_s", liveMaxLin),
        makeDoubleParam("teleop_max_angular_rad_s", liveMaxAng),
      ];
      callSetParameters(
        MOTOR_NODE,
        params,
        function () {
          setSettingsStatus("Max. rýchlosti uložené na motor", true);
        },
        function (err) {
          setSettingsStatus("Chyba uloženia rýchlostí: " + err, false);
        }
      );
    }, 350);
  }

  function onMaxLinInput() {
    liveMaxLin = parseFloat(el.maxLin.value, 10);
    el.maxLinVal.textContent = liveMaxLin.toFixed(2);
    scheduleTeleopSyncToMotor();
  }

  function onMaxAngInput() {
    liveMaxAng = parseFloat(el.maxAng.value, 10);
    el.maxAngVal.textContent = liveMaxAng.toFixed(2);
    scheduleTeleopSyncToMotor();
  }

  function applyImuPid() {
    const params = [
      makeDoubleParam("imu_yaw_kp", el.imuKp.value),
      makeDoubleParam("imu_yaw_ki", el.imuKi.value),
      makeDoubleParam("imu_yaw_kd", el.imuKd.value),
      makeDoubleParam("imu_yaw_deadband", el.imuDb.value),
      makeDoubleParam("imu_yaw_integral_limit", el.imuIl.value),
    ];
    setSettingsStatus("Ukladám IMU PID…", false);
    callSetParameters(
      MOTOR_NODE,
      params,
      function () {
        setSettingsStatus("IMU PID uložený", true);
      },
      function (err) {
        setSettingsStatus("Chyba IMU PID: " + err, false);
      }
    );
  }

  function applyFlow() {
    const params = [
      makeDoubleParam("correction_gain", el.flowGain.value),
      makeDoubleParam("max_correction", el.flowMax.value),
    ];
    setSettingsStatus("Ukladám optical flow…", false);
    callSetParametersFirstMatch(
      FLOW_NODE_CANDIDATES,
      params,
      function () {
        setSettingsStatus("Optical flow parametre uložené", true);
      },
      function (err) {
        setSettingsStatus(String(err), false);
      }
    );
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
      setStatus("Pripojené k rosbridge", true);
      cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: TELEOP_TOPIC,
        messageType: TWIST_TYPE,
      });
      if (pubTimer) clearInterval(pubTimer);
      pubTimer = setInterval(publishLoop, 1000 / PUBLISH_HZ);
      subscribeCam();
      fetchMotorParameters();
    });

    ros.on("error", function (e) {
      setStatus("Chyba rosbridge", false);
      console.warn(e);
    });

    ros.on("close", function () {
      setStatus("Odpojené", false);
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
    if (teleopSyncTimer) {
      clearTimeout(teleopSyncTimer);
      teleopSyncTimer = null;
    }
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
    setStatus("Odpojené", false);
    setSettingsStatus("", false);
  }

  function callModeService(serviceName, activeBtn, inactiveBtn, modeLabel) {
    if (!ros || !ros.isConnected) {
      el.modeStatus.textContent = "Najprv sa pripoj k rosbridge";
      return;
    }
    el.modeStatus.textContent = "Prepínam…";
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
          el.modeStatus.textContent = "Režim: " + modeLabel;
        } else {
          el.modeStatus.textContent = "Chyba: " + (result ? result.message : "no response");
        }
      },
      function (err) {
        activeBtn.disabled = false;
        inactiveBtn.disabled = false;
        el.modeStatus.textContent = "Chyba služby: " + err;
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
  el.btnApplyPid.addEventListener("click", applyImuPid);
  el.btnApplyFlow.addEventListener("click", applyFlow);

  el.maxLinVal.textContent = parseFloat(el.maxLin.value, 10).toFixed(2);
  el.maxAngVal.textContent = parseFloat(el.maxAng.value, 10).toFixed(2);

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
