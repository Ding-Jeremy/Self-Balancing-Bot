/*
 *   name:         main.cpp
 *   author:       Ding Jérémy
 *   date:         05.2026
 *   device:       ESP-WROOM-32
 *
 *   description:  This code is used to control the "self-balancing-robot" from the bachelor's degree work
 *                 of the same name.
 *
 *
 */
#include <Arduino.h>

//-------------- INCLUDES ---------------
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include "LittleFS.h"
#include <SPI.h>
#include "TMC2226.h"

//-------------- DEFINES ---------------
#define D_UART_BAUDRATE 9600

// UART pints
#define D_PIN_TX2 25
#define D_PIN_RX2 26
#define D_PIN_DIR 34

// WiFi credentials
#define D_SSID "self-balancing-bot"
#define D_PASSWORD "123456789"

//-------------- ENUMS ---------------

//-------------- FUNCTION PROTOTYPES ---------------
void init_littlefs();
void init_wifi();

void notify_clients(String msg);
void handle_websocket_message(void *arg, uint8_t *data, size_t len);
void on_event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void init_websocket();

void calculate_crc(uint8_t *frame, uint8_t framelength);
//-------------- GLOBAL VARIABLES ---------------
AsyncWebServer server(80); // Web server
AsyncWebSocket ws("/ws");  // WebSocket endpoint

//-------------- SETUP FUNCTION ---------------
void setup()
{
  // Begin serial transmission
  Serial.begin(D_UART_BAUDRATE); // Open serial port
  Serial.println("Serial1 initialized");

  // Initializes TMC
  TMC_init(9600, D_PIN_RX2, D_PIN_TX2);

  // Set chopper config
  U_TMC_CHOPCONF chopconf;

  chopconf.value = D_TMC_REGDFV_CHOPCONF; // DEFAULT VALUE
  chopconf.bits.mres = 0b0110;            // Microstepping
  TMC_write_to_register(E_TMC_REG_CHOPCONF, chopconf.bytes);
  U_TMC_VACTUAL vactual;
  vactual.value = 1000;
  TMC_write_to_register(E_TMC_REG_VACTUAL, vactual.bytes);

  // TMC_reset();
  /*U_TMC_VACTUAL vactual;
  vactual.value = 1000;
  TMC_write_to_register(E_TMC_REG_VACTUAL, vactual.bytes);*/

  /*init_wifi();
  init_littlefs();
  init_websocket();


  // Se¢ve root HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", "text/html"); });

  // Serve static files
  server.serveStatic("/", LittleFS, "/");

  server.begin();*/
}

//-------------- MAIN LOOP ---------------
void loop()
{
  /*U_TMC_CHOPCONF chopconf;
  U_TMC_GCONF gconf;
  TMC_read_from_register(E_TMC_REG_CHOPCONF, chopconf.bytes);
  TMC_read_from_register(E_TMC_REG_GCONF, gconf.bytes);

  Serial.print("Chopconf: ");
  Serial.println(chopconf.value, HEX);
  Serial.print("GConf: ");
  Serial.println(gconf.value, HEX);
  // TMC_step();
  delay(1000);*/
  TMC_step();
  delay(10);
}

//-------------- FUNCTION IMPLEMENTATIONS ---------------

// SERVER //
/*
 * Initializes the LittleFS file system
 */
void init_littlefs()
{
  if (!LittleFS.begin(true))
  {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  else
  {
    Serial.println("LittleFS mounted successfully");
  }
}

/*
 * Initializes the ESP32 in Access Point mode
 */
void init_wifi()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(D_SSID, D_PASSWORD);
  Serial.println("Starting Access Point...");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
}

/*
 * Sends a text message to all connected WebSocket clients
 */
void notify_clients(String msg)
{
  ws.textAll(msg);
}

/*
 * Handles incoming WebSocket messages from the client
 */
void handle_websocket_message(void *arg, uint8_t *data, size_t len)
{
  // Get frame infos
  AwsFrameInfo *info = (AwsFrameInfo *)arg;

  // Check if the received websocket message is valid
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    // Parse Data and Message.
    String message = (char *)data;
    JSONVar msg = JSON.parse(message);

    // If the received message is correct
    if (JSON.typeof(msg) == "object")
    {
      String ctrl_id = (const char *)msg["id"];
      String type = (const char *)msg["type"];

      // Check what action to take based on the received msg.
    }
  }
}

/*
 * WebSocket event callback handler
 */
void on_event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handle_websocket_message(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

/*
 * Initializes the WebSocket server and binds events
 */
void init_websocket()
{
  ws.onEvent(on_event);
  server.addHandler(&ws);
}
