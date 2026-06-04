//const gateway = `ws://localhost:8080`;
const gateway = `ws://${window.location.hostname}/ws`;
let websocket;
let joysticks = {};

// save old joystick positions
let old_x_r = null;
let old_y_l = null;

let motors_on = false;


function showPage(pageId) {
    document.querySelectorAll(".page").forEach(p => {
        p.style.display = "none";
    });

    document.getElementById(pageId).style.display = "block";
}

function showPIDPage() {
    requestPID();
    showPage("pid-page");
}

function showControlPage() {
    showPage("control-page");
} 

window.addEventListener("load", onLoad);

function onLoad() {
  showControlPage(); // Show control page by default
  initWebSocket();
  createJoystick("joystick1", "left");
  createJoystick("joystick2", "right");
  setJoystickEnabled(false); // Default disabled
  setButtonEnabled(false);   // Default disabled
}

function initWebSocket() {
  websocket = new WebSocket(gateway);

  websocket.onopen = () => {
    console.log("WebSocket connected");
    setJoystickEnabled(true);
    setButtonEnabled(true);
  };

  websocket.onclose = () => {
    console.log("WebSocket disconnected, retrying...");
    setJoystickEnabled(false);
    setButtonEnabled(false);
    updateBatteryInfo(null);
    updateAngleInfo(null);
    setTimeout(initWebSocket, 2000); // Retry connection
  };
  
  websocket.onmessage = (event) => {

    const message = JSON.parse(event.data);
    if (message.id === "battery") { 
      updateBatteryInfo(message);
    }
    else if (message.id === "angle"){
      updateAngleInfo(message);
    }
    else if(message.id === "motors_on" || message.id === "motors_off"){
      update_motors_button(message.id);
      console.log(message.id);
    }
     // IF pid values received.
    else if(message.id === "pid_values")
    {
      update_pid_values(message);
    }
    else if(message.id === "calibration_angle"){
      update_calibration_angle(message);
    }
    else if(message.id === "speed"){
      updateSpeedInfo(message);
    }
  } 
    
};


function createJoystick(containerId, idPrefix) {
  const container = document.getElementById(containerId);

  const containerWidth = container.offsetWidth;
  const joystickSize = containerWidth * 0.8; // adjust as needed

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
  joystick.enabled = false; // Add manual enabled flag

  // Limit the packet to each 20 ms
  const sendInterval = 100; // ms
  let lastX = null;
  let lastY = null;
  lastSent = 0;
  
  joystick.on("move", (evt, data) => {
    const now = Date.now();
    if (data && data.vector && joystick.enabled && (now - lastSent > sendInterval)) {
      lastSent = now;

      const x = idPrefix === "left" ? 0 : Math.round(data.vector.x * 100);
      const y = idPrefix === "right" ? 0 : Math.round(data.vector.y * 100);

      // Only send a packet when the values change
      if (x !== lastX || y !== lastY){
        lastX = x;
        lastY = y;
        const message = {
          id: `joystick-${idPrefix}`,
          x: x,
          y: y,
        };
        websocket.send(JSON.stringify(message));
      }
    }
  });

  joystick.on("end", () => {
    if (joystick.enabled) {
      websocket.send(JSON.stringify({ id: `joystick-${idPrefix}`, x: 0, y: 0 }));
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

  if (message != null) {
    const batteryPercentage = Math.round(message.percentage);
    const batteryVoltage = message.voltage;

    document.getElementById("battery-percentage").textContent = `${batteryPercentage} %`;
    document.getElementById("battery-voltage").textContent = `${(batteryVoltage / 1000).toFixed(2)} V`;

    // Set new width relative to viewBox or SVG proportions
    batteryLevel.style.width = `${batteryPercentage * 0.77}%`;

    // Change color depending on level
    if (batteryPercentage < 20) {
      batteryLevel.setAttribute("fill", "red");
    } else {
      batteryLevel.setAttribute("fill", "#29b524");
    }
  } else {
    document.getElementById("battery-percentage").textContent = `--- %`;
    document.getElementById("battery-voltage").textContent = "--.- V";
    batteryLevel.style.width = `0px`;
  }
}

function updateAngleInfo(message) {
  const angle_text = document.getElementById("angle-text");

  if (message != null) {
    const angle_value = Number(message.value);

    const sign = angle_value < 0 ? '-' : '+';

    const absValue = Math.abs(angle_value);
    const [integer, decimal] = absValue.toFixed(1).split('.');

    const formatted =
      `${sign}${integer.padStart(2, '0')}.${decimal}°`;

    angle_text.textContent = formatted;
    
  } else {
    document.getElementById("angle-text").textContent = `---.-°`;
  }
}
function updateSpeedInfo(message) {
  const speed_text = document.getElementById("speed-text");

  if (message != null) {
    const speed_value = Number(message.value);

    const sign = speed_value < 0 ? '-' : '+';

    const absValue = Math.abs(speed_value);
    const [integer, decimal] = absValue.toFixed(2).split('.');

    const formatted =
      `${sign}${integer.padStart(1, '0')}.${decimal} m/s`;

    speed_text.textContent = formatted;

  } else {
    speed_text.textContent = "---.-- m/s";
  }
}

function update_motors_button(state){
  if (state === "motors_on"){
    motors_on = true;
    document.getElementById("button_motors_toggle").textContent="Disable Motors";
  }
  else{
    motors_on = false;
    document.getElementById("button_motors_toggle").textContent="Enable Motors";
  }
}
function update_pid_values(message){
  // Update sliders
  balanceP.value = message.balance.kp;
  balanceI.value = message.balance.ti;
  balanceD.value = message.balance.td;
  velocityP.value = message.velocity.kp;
  velocityI.value = message.velocity.ti;
  velocityD.value = message.velocity.td;
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
function update_calibration_angle(message) {
  const angle_text = document.getElementById("calibration-angle-text");

  if (message != null) {
    const angle_value = Number(message.value);

    const sign = angle_value < 0 ? '-' : '+';

    const absValue = Math.abs(angle_value);
    const [integer, decimal] = absValue.toFixed(1).split('.');

    const formatted =
      `${sign}${integer.padStart(3, '0')}.${decimal}°`;

    angle_text.textContent = formatted;

  } else {
    angle_text.textContent = "---.-°";
  }
}

function send_button(button) {
  if (button === "motors_toggle"){
    if (motors_on){
      websocket.send(JSON.stringify({ id: `motors_off` }));
    }
    else{
      websocket.send(JSON.stringify({ id: `motors_on` }));
    }
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
   // Also disable the <input type="color">
  const colorInput = document.getElementById("robot-color");
  if (colorInput) {
    colorInput.disabled = !enabled;
  }
}


if ('serviceWorker' in navigator) {
  window.addEventListener('load', () => {
    navigator.serviceWorker.register('/service-worker.js')
      .then((reg) => {
        console.log('Service worker registered.', reg);
      });
  });
}

window.addEventListener("resize", () => {
  // Recreate joysticks to adjust to new layout
  createJoystick("joystick1", "left");
  createJoystick("joystick2", "right");
  joysticks["left"].enabled = true;
  joysticks["right"].enabled = true;
});

function calibrate(){
  websocket.send(JSON.stringify({ id: `calibrate` }));
}

// PID
const sliders = document.querySelectorAll("input[type='range']");

sliders.forEach(slider => {
    const valueSpan =
        document.getElementById(slider.id + "Value");

    valueSpan.textContent = slider.value;

    slider.addEventListener("input", () => {
        valueSpan.textContent = slider.value;
    });
});

// Request PID info from the server
function requestPID(){
  websocket.send(JSON.stringify({"id":"pid_request"}));
}

function savePID() {

    const pidValues = {
        id: "pid_values",
        balance: {
            kp: parseFloat(balanceP.value),
            ti: parseFloat(balanceI.value),
            td: parseFloat(balanceD.value)
        },
        velocity: {
            kp: parseFloat(velocityP.value),
            ti: parseFloat(velocityI.value),
            td: parseFloat(velocityD.value)
        }
    };
    // Sned those values to the server
    websocket.send(JSON.stringify(pidValues));
}
