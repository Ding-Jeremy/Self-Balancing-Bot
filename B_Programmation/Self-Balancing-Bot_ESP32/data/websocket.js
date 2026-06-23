const gateway = `ws://localhost:8080`;
//const gateway = `ws://${window.location.hostname}/ws`;

const websocket = new WebSocket(
    `ws://${window.location.hostname}/ws`
);

websocket.onopen = () => {
    console.log("WebSocket connected");
};