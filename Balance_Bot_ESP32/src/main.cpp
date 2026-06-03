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
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//-------------- DEFINES ---------------

#define D_UART_BAUDRATE 9600

// REGULATOR
#define D_REG_KPV 1
#define D_REG_TDV 0
#define D_REG_TIV 0

#define D_REG_KPT 500
#define D_REG_TDT 0
#define D_REG_TIT 0

#define D_REG_RESTTILT 0

#define D_ANGLE_TIMEOUT 500      // Timeout between angle sents
#define D_ANGLE_LIMITS {-20, 20} // Limit angle (degrees)

// WiFi credentials
#define D_SSID "self-balancing-bot"
#define D_PASSWORD "123456789"
// Stuctures
typedef struct PIDController
{
  float kp;
  float ki;
  float kd;

  float integral;
  float previousError;

  float outputMin;
  float outputMax;
} PIDController;

//-------------- FUNCTION PROTOTYPES ---------------
void init_littlefs();
void init_wifi();

void notify_clients(String msg);
void handle_websocket_message(void *arg, uint8_t *data, size_t len);
void on_event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void init_websocket();

float PID_update(PIDController *pid, float error, float dt);

void send_angle(float theta);
void send_motor_state(bool state);
bool is_angle_withing_range();
//-------------- GLOBAL VARIABLES ---------------
AsyncWebServer server(80); // Web server
AsyncWebSocket ws("/ws");  // WebSocket endpoint

Adafruit_MPU6050 mpu;
float speed_target = 0;
float speed = 0;
float angleX = 0.0f;
float angleX_offset = 0.0f;
unsigned long lastMicros = 0;
float e_v = 0, e_theta = 0;
float motor_speed = 0;
uint32_t angle_timer = 0;
int8_t angle_limits[2] = D_ANGLE_LIMITS;
bool motor_state = false;
// PID controllers
PIDController speedPID =
    {
        .kp = D_REG_KPV,
        .ki = D_REG_TIV,
        .kd = D_REG_TDV,

        .integral = 0.0f,
        .previousError = 0.0f,

        .outputMin = -10.0f, // max desired tilt
        .outputMax = 10.0f};

PIDController tiltPID =
    {
        .kp = D_REG_KPT,
        .ki = D_REG_TIT,
        .kd = D_REG_TDT,

        .integral = 0.0f,
        .previousError = 0.0f,

        .outputMin = -2000.0f, // Motor speed
        .outputMax = 2000.0f};

//-------------- SETUP FUNCTION ---------------
void setup()
{
  // Begin serial transmission
  Serial.begin(D_UART_BAUDRATE); // Open serial port
  Serial.println("Serial1 initialized");

  // Initializes TMC
  TMC_init();
  // Set chopper config
  U_TMC_CHOPCONF chopconf;

  chopconf.value = D_TMC_REGDFV_CHOPCONF; // DEFAULT VALUE
  chopconf.bits.mres = 0b0110;            // Microstepping
  TMC_write_to_register(0x00, E_TMC_REG_CHOPCONF, chopconf.bytes);
  TMC_write_to_register(0x01, E_TMC_REG_CHOPCONF, chopconf.bytes);

  // Start MPU
  if (!mpu.begin())
  {
    Serial.println("MPU6050 not found");
    while (1)
      ;
  }

  // Initialize MPU settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Start WIFI and server

  init_wifi();
  init_littlefs();
  init_websocket();

  // Se¢ve root HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", "text/html"); });

  // Serve static files
  server.serveStatic("/", LittleFS, "/");

  server.begin();
}

//-------------- MAIN LOOP ---------------
void loop()
{
  // ANGLE READING
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  // Accelerometer angle
  float accelAngleX =
      atan2(accel.acceleration.x,
            sqrt(accel.acceleration.y * accel.acceleration.y +
                 accel.acceleration.z * accel.acceleration.z)) *
      180.0f / PI;

  // Gyro rate (Adafruit returns rad/s)
  float gyroRateX = gyro.gyro.x * 180.0f / PI;

  // Complementary filter
  angleX = 0.98f * (angleX + gyroRateX * dt) + 0.02f * accelAngleX;

  // REGULATOR (outer loop)
  float speed_error = speed_target - speed;

  float theta_target =
      PID_update(&speedPID, speed_error, dt);

  // Defines the error from speed (and saved offset)
  float thetaError = theta_target + angleX_offset - angleX;

  float motor_speed =
      PID_update(&tiltPID, thetaError, dt);

  // Update motor speed
  motor_speed = motor_speed > 3000 ? 3000 : motor_speed < -3000 ? -3000
                                                                : motor_speed;
  // If angle out of safe range.
  if (!is_angle_withing_range())
  {
    if (motor_state)
    {
      motor_state = false;
      motor_speed = 0;
      TMC_disable();
      send_motor_state(false);
    }
  }
  // run motors (standalone speed)
  TMC_runspeed(0x00, (int32_t)motor_speed);
  TMC_runspeed(0x01, -(int32_t)motor_speed);
  if (millis() - angle_timer > D_ANGLE_TIMEOUT)
  {
    angle_timer = millis();
    send_angle(angleX);
  }
  delay(5);
}

//-------------- FUNCTION IMPLEMENTATIONS ---------------

/// @brief Reads current angleX and assess if in the range
/// @return
bool is_angle_withing_range()
{
  return (angleX > angle_limits[0] && angleX < angle_limits[1]);
}
/// @brief Send the state of the motor to the clients
/// @param state
void send_motor_state(bool state)
{
  JSONVar json;
  if (state)
    json["id"] = "motors_on";
  else
    json["id"] = "motors_off";
  String msg = JSON.stringify(json);
  notify_clients(msg);
}
void send_angle(float theta)
{
  // Create JSON message, send it
  JSONVar json;
  json["id"] = "angle";
  json["value"] = theta;
  String msg = JSON.stringify(json);
  notify_clients(msg);
}
// PID //
float PID_update(PIDController *pid, float error, float dt)
{
  // Integral term
  pid->integral += error * dt;

  // Derivative term
  float derivative = (error - pid->previousError) / dt;

  // PID output
  float output =
      pid->kp * error +
      pid->ki * pid->integral +
      pid->kd * derivative;

  // Output saturation
  if (output > pid->outputMax)
    output = pid->outputMax;
  else if (output < pid->outputMin)
    output = pid->outputMin;

  pid->previousError = error;

  return output;
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
      String msg_id = (const char *)msg["id"];
      if (msg_id == "motors_off")
      {
        motor_state = false;
        TMC_disable();
        send_motor_state(false);
      }
      else if (msg_id == "motors_on")
      {
        if (is_angle_withing_range())
        {
          TMC_enable();
          motor_state = true;
          send_motor_state(true);
        }
      }
      else if (msg_id == "calibrate")
      {
        angleX_offset = angleX;
      }
      else if (msg_id == "pid_values")
      {
        Serial.println("Received: PID");
        Serial.println(msg);
      }
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
