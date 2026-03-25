/* global ROSLIB */
(function () {
  const TELEOP_TOPIC = "/teleop_cmd_vel";
  const TWIST_TYPE = "geometry_msgs/msg/Twist";
  const CAMERA_TOPIC = "/camera/camera_node/image_raw/compressed";
  const CAMERA_TYPE = "sensor_msgs/msg/CompressedImage";

  const MAX_LINEAR = 0.45;
  const MAX_ANGULAR = 1.0;
  const PUBLISH_HZ = 20;

  const el = {
    status: document.getElementById("status"),
    wsUrl: document.getElementById("wsUrl"),
    btnConnect: document.getElementById("btnConnect"),
    btnDisconnect: document.getElementById("btnDisconnect"),
    cam: document.getElementById("cam"),
    camPlaceholder: document.getElementById("camPlaceholder"),
    btnStop: document.getElementById("btnStop"),
  };

  let ros = null;
  let cmdVel = null;
  let camSub = null;
  let pubTimer = null;
  let currentTwist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };

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
      try { camSub.unsubscribe(); } catch (e) {}
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
      try { ros.close(); } catch (e) {}
    }

    ros = new ROSLIB.Ros({ url });

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
        try { camSub.unsubscribe(); } catch (e) {}
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
      try { ros.close(); } catch (e) {}
      ros = null;
    }
    cmdVel = null;
    el.cam.classList.add("hidden");
    el.camPlaceholder.classList.remove("hidden");
    setStatus("Odpojené", false);
  }

  el.btnConnect.addEventListener("click", connect);
  el.btnDisconnect.addEventListener("click", disconnect);
  el.btnStop.addEventListener("click", stopMotion);

  document.querySelectorAll("button.drive").forEach(function (btn) {
    const lx = parseFloat(btn.getAttribute("data-lx"), 10) || 0;
    const az = parseFloat(btn.getAttribute("data-az"), 10) || 0;

    function apply() {
      currentTwist = {
        linear: { x: lx * MAX_LINEAR, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: az * MAX_ANGULAR },
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
