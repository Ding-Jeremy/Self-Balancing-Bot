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

#define D_REG_KPT 1
#define D_REG_TDT 0
#define D_REG_TIT 0

#define D_REG_RESTTILT 0

#define D_ANGLE_TIMEOUT 500      // Timeout between angle sents
#define D_ANGLE_LIMITS {-20, 20} // Limit angle (degrees)
#define D_SPEED_LIMITS {-1, 1}   // Speed limits (when moving) [m/s]

#define D_WHEEL_RADIUS 0.055 // [m]
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
void send_speed(float speed);
void send_motor_state(bool state);

bool is_angle_withing_range();
float map_f(float value, float min1, float max1, float min2, float max2);
float rad_s_to_ustp_s(float rad_s, uint8_t micro_stepping_value);
//-------------- GLOBAL VARIABLES ---------------
AsyncWebServer server(80); // Web server
AsyncWebSocket ws("/ws");  // WebSocket endpoint

Adafruit_MPU6050 mpu;
float position_target = 0.0f;
float speed = 0.0f;
float angleX = 0.0f;
float angleX_offset = 0.0f;
unsigned long lastMicros = 0;
float motor_speed = 0.0f;
float position = 0.0f;
uint32_t angle_timer = 0;
int8_t angle_limits[2] = D_ANGLE_LIMITS;
float angle_control_range[2] = D_SPEED_LIMITS;
bool motor_state = true;
uint8_t micro_stepping_value = 0;
float us_speed = 0;
// PID controllers
PIDController positionPID =
    {
        .kp = D_REG_KPV,
        .ki = D_REG_TIV,
        .kd = D_REG_TDV,

        .integral = 0.0f,
        .previousError = 0.0f,

        .outputMin = -10.0f, // max desired tilt [°]
        .outputMax = 10.0f};

PIDController tiltPID =
    {
        .kp = D_REG_KPT,
        .ki = D_REG_TIT,
        .kd = D_REG_TDT,

        .integral = 0.0f,
        .previousError = 0.0f,

        .outputMin = -4 * M_PI, // Motor speed [rad/s]
        .outputMax = 4 * M_PI};

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
  chopconf.bits.mres = 0b0101;            // Microstepping
  micro_stepping_value = 8;
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

  // REGULATOR (outer loop, speed regulation)

  float position_error = (position_target - position);

  // Generate angle theta based on position Error
  float theta_target =
      PID_update(&positionPID, position_error, dt);

  // Defines the error from speed (and saved offset)
  float thetaError = theta_target + angleX_offset - angleX;

  // Generate motor speed from inner loop
  float motor_speed =
      PID_update(&tiltPID, thetaError, dt);

  // Convert rad/s to ustp/stp
  us_speed = rad_s_to_ustp_s(motor_speed, micro_stepping_value);

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

  // Chariot speed
  speed = motor_speed * D_WHEEL_RADIUS;
  // Compute position
  position += speed * dt;
  // run motors (standalone speed)
  TMC_runspeed(0x00, (int32_t)us_speed);
  TMC_runspeed(0x01, -(int32_t)us_speed);

  // Send angle and speed to clients
  if (millis() - angle_timer > D_ANGLE_TIMEOUT)
  {
    Serial.print("speed: ");
    Serial.println(speed);
    Serial.print("pos: ");
    Serial.println(position);
    angle_timer = millis();
    send_speed(position);
    send_angle(angleX);
  }
  delay(1);
}

//-------------- FUNCTION IMPLEMENTATIONS ---------------

float rad_s_to_ustp_s(float rad_s, uint8_t micro_stepping_value)
{
  const float steps_per_rev = 200.0f; // typical stepper motor (1.8°)

  float rev_per_sec = rad_s / (2.0f * M_PI);
  float steps_per_sec = rev_per_sec * steps_per_rev;

  return steps_per_sec * micro_stepping_value;
}
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
/// @brief Sends the current angle to the clients
/// @param theta
void send_angle(float theta)
{
  // Create JSON message, send it
  JSONVar json;
  json["id"] = "angle";
  json["value"] = theta;
  String msg = JSON.stringify(json);
  notify_clients(msg);
}

/// @brief Sends speed to clients
/// @param speed
void send_speed(float speed)
{
  // Create JSON message, send it
  JSONVar json;
  json["id"] = "speed";
  json["value"] = speed;
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
        // Save angle offset
        angleX_offset = angleX;
        // Transmit saved angle to server
        JSONVar json;
        json["id"] = "calibration_angle";
        json["value"] = angleX_offset;
        String msg = JSON.stringify(json);
        notify_clients(msg);
      }
      else if (msg_id == "pid_values")
      {
        // Safety check
        if (msg["balance"].hasOwnProperty("kp"))
        {
          tiltPID.kp = (double)msg["balance"]["kp"];
          tiltPID.ki = (double)msg["balance"]["ti"];
          tiltPID.kd = (double)msg["balance"]["td"];
        }

        if (msg["velocity"].hasOwnProperty("kp"))
        {
          positionPID.kp = (double)msg["velocity"]["kp"];
          positionPID.ki = (double)msg["velocity"]["ti"];
          positionPID.kd = (double)msg["velocity"]["td"];
        }
      }
      else if (msg_id == "pid_request")
      {
        JSONVar json;
        json["id"] = "pid_values";
        json["balance"]["kp"] = tiltPID.kp;
        json["balance"]["td"] = tiltPID.kd;
        json["balance"]["ti"] = tiltPID.ki;
        json["velocity"]["kp"] = positionPID.kp;
        json["velocity"]["td"] = positionPID.kd;
        json["velocity"]["ti"] = positionPID.ki;
        String msg = JSON.stringify(json);
        notify_clients(msg);
      }
      else if (msg_id == "joystick-left")
      {
        // Adjust speed target from joystick
        position_target = position + map_f((double)msg["y"], -100, 100, angle_control_range[0], angle_control_range[1]);
      }
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

/// @brief Simple map function.
/// @param value
/// @param min1
/// @param max1
/// @param min2
/// @param max2
/// @return
float map_f(float value, float min1, float max1, float min2, float max2)
{
  if (max1 == min1)
  {
    // avoid division by zero
    return min2;
  }

  float t = (value - min1) / (max1 - min1);
  return min2 + t * (max2 - min2);
}