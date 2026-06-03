//const gateway = `ws://localhost:8080`;
const gateway = `ws://${window.location.hostname}/ws`;
let websocket;
let joysticks = {};

// save old joystick positions
let old_x_r = null;
let old_y_l = null;

let motors_on = false;



window.addEventListener("load", onLoad);

function onLoad() {
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
  const sendInterval = 20; // ms
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
    const angle_value = Math.round(message.value);

    angle_text.textContent = `${angle_value} °`;


    // Change color depending on level
    if (angle_value < -20 || angle_value > 20) {
    } else {
    }
  } else {
    document.getElementById("angle_indicator").value = `--- °`;

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
function send_button(button) {
  if (button == "motors_toggle"){
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