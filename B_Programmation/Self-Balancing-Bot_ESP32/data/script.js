const gateway = `ws://${window.location.hostname}/ws`;
let websocket;
const joysticks = {};

let motorsOn = false;
let recording = false;
let receivingCsv = false;
let csvData = "";

const sendMessage = (message) => {
  if (!websocket || websocket.readyState !== WebSocket.OPEN) {
    return false;
  }

  websocket.send(JSON.stringify(message));
  return true;
};

// Page system
function showPage(pageId) {
  document.querySelectorAll(".page").forEach((page) => {
    page.style.display = "none";
  });

  document.getElementById(pageId).style.display = "block";
}

function showPidPage() {
  requestPid();
  showPage("pid-page");
}

function showControlPage() {
  showPage("control-page");
}

// Window
window.addEventListener("load", onLoad);

function onLoad() {
  showControlPage(); // Show control page by default
  initWebSocket();
  createJoystick("joystick1", "left");
  createJoystick("joystick2", "right");
  setJoystickEnabled(false); // Default disabled
  setButtonEnabled(false);   // Default disabled
}

// Web sockets parameters
function initWebSocket() {
  websocket = new WebSocket(gateway);

  websocket.onopen = () => {
    setJoystickEnabled(true);
    setButtonEnabled(true);
    updateConnectionStatus(true);
  };

  websocket.onclose = () => {
    setJoystickEnabled(false);
    setButtonEnabled(false);
    updateBatteryInfo(null);
    updateAngleInfo(null);
    updateTargetAngleInfo(null);
    updateConnectionStatus(false);
    setRecordingState(false);
    receivingCsv = false;
    csvData = "";
    setTimeout(initWebSocket, 2000); // Retry connection
  };
  
  websocket.onmessage = (event) => {
    // Recording mode (raw CSV stream)
    if (receivingCsv) {
      if (event.data === "save_end") {
        receivingCsv = false;
        saveCsv(csvData);
        csvData = "";
        return;
      }

      csvData += event.data;
      return;
    }

    let message;
    try {
      message = JSON.parse(event.data);
    } catch {
      console.warn("Unknown non-JSON message:", event.data);
      return;
    }

    switch (message.id) {
      case "save_start":
        receivingCsv = true;
        csvData = "";
        break;
      case "battery":
        updateBatteryInfo(message);
        break;
      case "angle":
        updateAngleInfo(message);
        break;
      case "motors_on":
      case "motors_off":
        updateMotorsButton(message.id);
        break;
      case "pid_values":
        updatePidValues(message);
        break;
      case "calibration_angle":
        updateCalibrationAngle(message);
        break;
      case "target_angle":
        updateTargetAngleInfo(message);
        break;
    }
  };
}

function createJoystick(containerId, idPrefix) {
  const container = document.getElementById(containerId);

  const containerWidth = container.offsetWidth;
  const joystickSize = containerWidth * 0.8;

  // Destroy existing joystick if any
  if (joysticks[idPrefix]) {
    joysticks[idPrefix].destroy();
  }

  const joystick = nipplejs.create({
    zone: container,
    mode: "static",
    position: { left: "50%", top: "50%" },
    size: joystickSize,
    color: "white",
    restOpacity: 0.3,
    lockX: idPrefix === "right",
    lockY: idPrefix === "left",
  });

  joysticks[idPrefix] = joystick;
  joystick.enabled = false;

  // Limit packets to one every 50 ms (20 Hz).
  const sendInterval = 50; // ms
  let lastX = null;
  let lastY = null;
  let lastSent = 0;
  
  joystick.on("move", (_event, data) => {
    const now = Date.now();
    if (
      data &&
      data.vector &&
      joystick.enabled &&
      now - lastSent > sendInterval
    ) {
      lastSent = now;

      const x = idPrefix === "left" ? 0 : Math.round(data.vector.x * 100);
      const y = idPrefix === "right" ? 0 : Math.round(data.vector.y * 100);

      // Only send a packet when the values change
      if (x !== lastX || y !== lastY) {
        lastX = x;
        lastY = y;
        const message = {
          id: `joystick-${idPrefix}`,
          x,
          y,
        };
        sendMessage(message);
      }
    }
  });

  joystick.on("end", () => {
    if (joystick.enabled) {
      sendMessage({ id: `joystick-${idPrefix}`, x: 0, y: 0 });
    }
  });
}

function setJoystickEnabled(enabled) {
  const joystickElements = document.querySelectorAll(".joystick-wrapper");

  joystickElements.forEach((el) => {
    if (enabled) {
      el.classList.remove("disabled");
    } else {
      el.classList.add("disabled");
    }
  });

  Object.values(joysticks).forEach((joystick) => {
    joystick.enabled = enabled;
  });
}

function updateBatteryInfo(message) {
  const batteryLevel = document.getElementById("battery-level");

  if (message == null) {
    document.getElementById("battery-percentage").textContent = `--- %`;
    document.getElementById("battery-voltage").textContent = "--.- V";
    batteryLevel.style.width = "0px";
    return;
  }

  const batteryPercentage = Math.round(message.percentage);
  const batteryVoltage = message.voltage;

  document.getElementById("battery-percentage").textContent = `${batteryPercentage} %`;
  document.getElementById("battery-voltage").textContent = `${batteryVoltage} V`;
  batteryLevel.style.width = `${batteryPercentage * 0.77}%`;

  if (batteryPercentage < 20) {
    batteryLevel.setAttribute("fill", "red");
  } else {
    batteryLevel.setAttribute("fill", "#29b524");
  }
}

function updateAngleInfo(message) {
  const angleText = document.getElementById("angle-text");

  if (message == null) {
    angleText.textContent = "---.-°";
    return;
  }

  const angleValue = Number(message.value);
  const sign = angleValue < 0 ? "-" : "+";
  const absValue = Math.abs(angleValue);
  const [integer, decimal] = absValue.toFixed(1).split(".");

  angleText.textContent = `${sign}${integer.padStart(2, "0")}.${decimal}°`;
}

function updateTargetAngleInfo(message) {
  const targetAngleText = document.getElementById("target-angle-text");

  if (message == null) {
    targetAngleText.textContent = "---.-°";
    return;
  }

  const targetAngle = Number(message.value);
  const sign = targetAngle < 0 ? "-" : "+";
  const absValue = Math.abs(targetAngle);
  const [integer, decimal] = absValue.toFixed(2).split(".");

  targetAngleText.textContent = `${sign}${integer.padStart(1, "0")}.${decimal} °`;
}

function updateConnectionStatus(connected) {
  const status = document.getElementById("connection-status");

  if (connected) {
    status.textContent = "Connected";
    status.className = "connected";
  } else {
    status.textContent = "Disconnected";
    status.className = "disconnected";
  }
}

function updateMotorsButton(state) {
  if (state === "motors_on") {
    motorsOn = true;
    document.getElementById("button-motors-toggle").textContent = "Disable Motors";
  } else {
    motorsOn = false;
    document.getElementById("button-motors-toggle").textContent = "Enable Motors";
  }
}

function updatePidValues(message) {
  // Update sliders
  pidSliders.balanceP.value = message.balance.kp;
  pidSliders.balanceI.value = message.balance.ti;
  pidSliders.balanceD.value = message.balance.td;
  pidSliders.velocityP.value = message.velocity.kp;
  pidSliders.velocityI.value = message.velocity.ti;
  pidSliders.velocityD.value = message.velocity.td;
  // Update text
  const balancePValue = document.getElementById("balancePValue");
  const balanceIValue = document.getElementById("balanceIValue");
  const balanceDValue = document.getElementById("balanceDValue");

  const velocityPValue = document.getElementById("velocityPValue");
  const velocityIValue = document.getElementById("velocityIValue");
  const velocityDValue = document.getElementById("velocityDValue");

  balancePValue.textContent = Number(message.balance.kp).toFixed(2);
  balanceIValue.textContent = Number(message.balance.ti).toFixed(2);
  balanceDValue.textContent = Number(message.balance.td).toFixed(2);

  velocityPValue.textContent = Number(message.velocity.kp).toFixed(2);
  velocityIValue.textContent = Number(message.velocity.ti).toFixed(2);
  velocityDValue.textContent = Number(message.velocity.td).toFixed(2);
}

function updateCalibrationAngle(message) {
  const angleText = document.getElementById("calibration-angle-text");

  if (message == null) {
    angleText.textContent = "---.-°";
    return;
  }

  const angleValue = Number(message.value);
  const sign = angleValue < 0 ? "-" : "+";
  const absValue = Math.abs(angleValue);
  const [integer, decimal] = absValue.toFixed(1).split(".");

  angleText.textContent = `${sign}${integer.padStart(3, "0")}.${decimal}°`;
}

function toggleMotors() {
  if (motorsOn) {
    sendMessage({ id: "motors_off" });
  } else {
    sendMessage({ id: "motors_on" });
  }
}

function setButtonEnabled(enabled) {
  const buttonElements = document.querySelectorAll(".button");
  buttonElements.forEach((el) => {
    if (enabled) {
      el.classList.remove("disabled");
    } else {
      el.classList.add("disabled");
    }
  });
  // Also disable the color input.
  const colorInput = document.getElementById("robot-color");
  if (colorInput) {
    colorInput.disabled = !enabled;
  }
}

function setRecordingState(enabled) {
  recording = enabled;
  const recordButton = document.getElementById("button-record");
  recordButton.style.backgroundColor = enabled ? "red" : "green";
  recordButton.textContent = enabled ? "Stop" : "Record";
}

function toggleRecording() {
  const nextState = !recording;
  const messageId = nextState ? "record_start" : "record_stop";
  if (!sendMessage({ id: messageId })) {
    return;
  }

  setRecordingState(nextState);
}


if ("serviceWorker" in navigator) {
  window.addEventListener("load", () => {
    navigator.serviceWorker.register("/service-worker.js");
  });
}

window.addEventListener("resize", () => {
  const connected = websocket && websocket.readyState === WebSocket.OPEN;
  // Recreate joysticks to adjust to new layout
  createJoystick("joystick1", "left");
  createJoystick("joystick2", "right");
  setJoystickEnabled(connected);
});

function calibrate() {
  sendMessage({ id: "calibrate" });
}

// PID
const sliders = document.querySelectorAll("input[type='range']");
const pidSliders = {
  balanceP: document.getElementById("balanceP"),
  balanceI: document.getElementById("balanceI"),
  balanceD: document.getElementById("balanceD"),
  velocityP: document.getElementById("velocityP"),
  velocityI: document.getElementById("velocityI"),
  velocityD: document.getElementById("velocityD"),
};

sliders.forEach((slider) => {
  const valueSpan = document.getElementById(slider.id + "Value");

  valueSpan.textContent = slider.value;

  slider.addEventListener("input", () => {
    valueSpan.textContent = slider.value;
  });
});

// Request PID info from the server
function requestPid() {
  sendMessage({ id: "pid_request" });
}

function savePid() {
  const pidValues = {
    id: "pid_values",
    balance: {
      kp: parseFloat(pidSliders.balanceP.value),
      ti: parseFloat(pidSliders.balanceI.value),
      td: parseFloat(pidSliders.balanceD.value),
    },
    velocity: {
      kp: parseFloat(pidSliders.velocityP.value),
      ti: parseFloat(pidSliders.velocityI.value),
      td: parseFloat(pidSliders.velocityD.value),
    },
  };

  sendMessage(pidValues);
}

// CSV
function saveCsv(data) {

  const blob = new Blob(
    [data],
    { type: "text/csv" }
  );

  const url = URL.createObjectURL(blob);

  const a = document.createElement("a");

  a.href = url;

  a.download =
    `recording_${Date.now()}.csv`;

  document.body.appendChild(a);

  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}
