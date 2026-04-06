/* global ROSLIB */
(function () {
  const TELEOP_TOPIC = "/teleop_cmd_vel";
  const TWIST_TYPE = "geometry_msgs/msg/Twist";
  const CAMERA_TOPIC = "/camera/camera_node/image_raw/compressed";
  const CAMERA_TYPE = "sensor_msgs/msg/CompressedImage";
  const IMU_TOPIC = "/imu";
  const IMU_TYPE = "sensor_msgs/msg/Imu";
  const PUBLISH_HZ = 20;
  const LS_MAX_LIN = "waverower_web_max_lin";

  // linearne slidery; musi sediet s runtime_stack motor_hat teleop_max
  const SLIDER_LIN = { min: 0.01, max: 0.08, step: 0.01 };
  const LAUNCH_TELEOP_MAX_LIN = 0.5;
  const LAUNCH_TELEOP_MAX_ANG = 1.0;
  /** motor_hat: PWM cca m * 100 * pwm_boost */
  const LAUNCH_PWM_BOOST = 2.35;

  const DEFAULTS = {
    teleop_max_linear_m_s: 0.04,
  };

  function clamp(n, lo, hi) {
    return Math.min(hi, Math.max(lo, n));
  }

  function snapStep(n, step) {
    return Math.round(n / step) * step;
  }

  const el = {
    status: document.getElementById("status"),
    wsUrl: document.getElementById("wsUrl"),
    btnConnect: document.getElementById("btnConnect"),
    btnDisconnect: document.getElementById("btnDisconnect"),
    camCanvas: document.getElementById("camCanvas"),
    camPlaceholder: document.getElementById("camPlaceholder"),
    btnStop: document.getElementById("btnStop"),
    btnManual: document.getElementById("btnManual"),
    btnWander: document.getElementById("btnWander"),
    modeStatus: document.getElementById("modeStatus"),
    maxLin: document.getElementById("maxLin"),
    maxLinVal: document.getElementById("maxLinVal"),
    maxAngVal: document.getElementById("maxAngVal"),
    camLive: document.getElementById("camLive"),
    imuWz: document.getElementById("imuWz"),
    imuWabs: document.getElementById("imuWabs"),
    imuStale: document.getElementById("imuStale"),
  };

  let ros = null;
  let cmdVel = null;
  let camSub = null;
  let pubTimer = null;
  /** pri disconnect zvysit - zahodit stare snimky */
  let camFeedGen = 0;
  let imuSub = null;
  let imuUiTimer = null;
  let lastImuMs = 0;
  let currentTwist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
  let liveMaxLin = parseFloat(localStorage.getItem(LS_MAX_LIN), 10);
  if (!Number.isFinite(liveMaxLin)) {
    liveMaxLin =
      parseFloat(el.maxLin && el.maxLin.value, 10) ||
      DEFAULTS.teleop_max_linear_m_s;
  }
  liveMaxLin = snapStep(clamp(liveMaxLin, SLIDER_LIN.min, SLIDER_LIN.max), SLIDER_LIN.step);

  /** 0..1 podla slidera */
  function teleopIntensity() {
    return liveMaxLin / SLIDER_LIN.max;
  }

  let liveMaxAng =
    teleopIntensity() * (LAUNCH_TELEOP_MAX_ANG / LAUNCH_PWM_BOOST);

  if (el.maxLin) el.maxLin.value = String(liveMaxLin);

  function onMaxLinInput() {
    liveMaxLin = snapStep(
      clamp(parseFloat(el.maxLin.value, 10), SLIDER_LIN.min, SLIDER_LIN.max),
      SLIDER_LIN.step
    );
    liveMaxAng = teleopIntensity() * (LAUNCH_TELEOP_MAX_ANG / LAUNCH_PWM_BOOST);
    el.maxLin.value = String(liveMaxLin);
    el.maxLinVal.textContent = liveMaxLin.toFixed(2);
    if (el.maxAngVal) el.maxAngVal.textContent = liveMaxAng.toFixed(2);
    localStorage.setItem(LS_MAX_LIN, String(liveMaxLin));
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

  function compressedMessageToImageUrl(m) {
    if (typeof m.data === "string") {
      return { url: "data:image/jpeg;base64," + m.data, revoke: null };
    }
    const blob = new Blob([new Uint8Array(m.data)], { type: "image/jpeg" });
    const url = URL.createObjectURL(blob);
    return { url: url, revoke: url };
  }

  function drawCamFrame(img) {
    const canvas = el.camCanvas;
    const ctx = canvas.getContext("2d");
    const par = canvas.parentElement;
    const w = Math.max(1, Math.floor(par.clientWidth));
    const h = Math.max(1, Math.floor(par.clientHeight));
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    const iw = img.naturalWidth;
    const ih = img.naturalHeight;
    if (!iw || !ih) return;
    const s = Math.min(w / iw, h / ih);
    const dw = iw * s;
    const dh = ih * s;
    const x = (w - dw) * 0.5;
    const y = (h - dh) * 0.5;
    ctx.fillStyle = "#030508";
    ctx.fillRect(0, 0, w, h);
    ctx.drawImage(img, x, y, dw, dh);
  }

  function resetImuDisplay() {
    lastImuMs = 0;
    if (el.imuWz) el.imuWz.textContent = "—";
    if (el.imuWabs) el.imuWabs.textContent = "—";
    if (el.imuStale) {
      el.imuStale.textContent = "disconnected";
      el.imuStale.className = "imu-stale unknown";
    }
  }

  function tickImuUi() {
    if (!ros || !ros.isConnected) {
      resetImuDisplay();
      return;
    }
    const now = Date.now();
    const age = lastImuMs ? now - lastImuMs : 999999;
    if (!el.imuStale) return;
    if (!lastImuMs || age > 1200) {
      el.imuStale.textContent = age > 1200 && lastImuMs ? "no /imu" : "waiting...";
      el.imuStale.className = "imu-stale " + (lastImuMs ? "warn" : "unknown");
    } else {
      el.imuStale.textContent = "OK";
      el.imuStale.className = "imu-stale ok";
    }
  }

  function subscribeImu() {
    if (imuSub) {
      try {
        imuSub.unsubscribe();
      } catch (e) {}
      imuSub = null;
    }
    if (!ros) return;
    imuSub = new ROSLIB.Topic({
      ros: ros,
      name: IMU_TOPIC,
      messageType: IMU_TYPE,
    });
    imuSub.subscribe(function (msg) {
      const av = msg.angular_velocity;
      let z = NaN;
      if (av && typeof av.z === "number") z = av.z;
      else if (av && av.z != null) z = parseFloat(av.z);
      if (!Number.isFinite(z)) return;
      lastImuMs = Date.now();
      if (el.imuWz) el.imuWz.textContent = z.toFixed(3);
      if (el.imuWabs) el.imuWabs.textContent = Math.abs(z).toFixed(3);
    });
  }

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
      throttle_rate: 66,
    });
    const feedGen = ++camFeedGen;
    let camUiPrimed = false;
    let camFrameSeq = 0;
    camSub.subscribe(function (m) {
      if (feedGen !== camFeedGen) return;
      const seq = ++camFrameSeq;
      let revokeUrl = null;
      let url;
      try {
        const u = compressedMessageToImageUrl(m);
        url = u.url;
        revokeUrl = u.revoke;
      } catch (err) {
        console.warn("Camera frame error:", err, typeof m.data, m.data && m.data.length);
        return;
      }
      const img = new Image();
      img.decoding = "async";
      img.onload = function () {
        if (feedGen !== camFeedGen || seq !== camFrameSeq) {
          if (revokeUrl) URL.revokeObjectURL(revokeUrl);
          return;
        }
        try {
          drawCamFrame(img);
        } catch (e) {
          console.warn("drawCamFrame:", e);
        }
        if (revokeUrl) URL.revokeObjectURL(revokeUrl);
        if (!camUiPrimed) {
          camUiPrimed = true;
          el.camCanvas.classList.remove("hidden");
          el.camPlaceholder.classList.add("hidden");
          if (el.camLive) {
            el.camLive.textContent = "Live";
            el.camLive.classList.add("live");
          }
        }
      };
      img.onerror = function () {
        if (revokeUrl) URL.revokeObjectURL(revokeUrl);
      };
      img.src = url;
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
      if (imuUiTimer) clearInterval(imuUiTimer);
      resetImuDisplay();
      subscribeCam();
      subscribeImu();
      imuUiTimer = setInterval(tickImuUi, 400);
    });

    ros.on("error", function (e) {
      setStatus("Rosbridge error", false);
      console.warn(e);
    });

    ros.on("close", function () {
      camFeedGen++;
      setStatus("Disconnected", false);
      if (pubTimer) {
        clearInterval(pubTimer);
        pubTimer = null;
      }
      if (imuUiTimer) {
        clearInterval(imuUiTimer);
        imuUiTimer = null;
      }
      cmdVel = null;
      if (camSub) {
        try {
          camSub.unsubscribe();
        } catch (e) {}
        camSub = null;
      }
      if (imuSub) {
        try {
          imuSub.unsubscribe();
        } catch (e) {}
        imuSub = null;
      }
      resetImuDisplay();
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
    el.camCanvas.classList.add("hidden");
    el.camPlaceholder.classList.remove("hidden");
    if (el.camLive) {
      el.camLive.textContent = "Standby";
      el.camLive.classList.remove("live");
    }
    setStatus("Disconnected", false);
  }

  function callModeService(serviceName, activeBtn, inactiveBtn, modeLabel) {
    if (!ros || !ros.isConnected) {
      el.modeStatus.textContent = "Connect to rosbridge first";
      return;
    }
    el.modeStatus.textContent = "Switching...";
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

  el.maxLinVal.textContent = liveMaxLin.toFixed(2);
  if (el.maxAngVal) el.maxAngVal.textContent = liveMaxAng.toFixed(2);

  document.querySelectorAll("button.drive").forEach(function (btn) {
    const lx = parseFloat(btn.getAttribute("data-lx"), 10) || 0;
    const az = parseFloat(btn.getAttribute("data-az"), 10) || 0;

    function apply() {
      const t = teleopIntensity();
      const g = 1.0 / LAUNCH_PWM_BOOST;
      currentTwist = {
        linear: { x: lx * t * LAUNCH_TELEOP_MAX_LIN * g, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: az * t * LAUNCH_TELEOP_MAX_ANG * g },
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
