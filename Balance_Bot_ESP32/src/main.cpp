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
  Serial.begin(D_UART_BAUDRATE); // Open serial port
  Serial.println("Serial1 initialized");

  init_wifi();
  init_littlefs();
  init_websocket();
  TMC_init(9600, D_PIN_RX2, D_PIN_TX2);

  // Serve root HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", "text/html"); });

  // Serve static files
  server.serveStatic("/", LittleFS, "/");

  server.begin();
}

//-------------- MAIN LOOP ---------------
void loop()
{
  // sent a frame making the motor move
  uint8_t frame[7];
  frame[0] = 0x05;        // Init
  frame[1] = 0x00;        // Chip address
  frame[2] = 0x80 | 0x22; // Reg address + W
  uint32_t speed = 100;
  frame[3] = (speed & 0xFF0000) >> 16;
  frame[4] = (speed & 0x00FF00) >> 8;
  frame[5] = speed & 0xFF;
  calculate_crc(frame, 7);

  Serial.print("Sent: ");
  for (int i = 0; i < 7; i++)
  {
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }

  TMC_send_frame(frame, 7);

  delay(1000);
}

//-------------- FUNCTION IMPLEMENTATIONS ---------------

void calculate_crc(uint8_t *frame, uint8_t framelength)
{
  int i, j;
  uint8_t crc = 0;
  uint8_t current_byte;

  for (i = 0; i < (framelength - 1); i++)
  {
    current_byte = frame[i];
    for (j = 0; j < 8; j++)
    {
      if ((crc >> 7) ^ (current_byte & 0x01))
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc = (crc << 1);
      }
      current_byte = current_byte >> 1;
    }
  }
  frame[framelength - 1] = crc;
}

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
