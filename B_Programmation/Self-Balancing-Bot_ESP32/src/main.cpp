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

// BATTERY MANAGEMENT
#define D_PIN_BATTERY_LVL 35
#define D_R1_VAL 51000.0                                // First resistor of the divider bridge
#define D_R2_VAL 10000.0                                // Second resistor
#define D_BATT_RATIO ((D_R1_VAL + D_R2_VAL) / D_R2_VAL) // Resistors ratio (Vbat/Vadc)
#define D_BATT_MIN_V 14.70
#define D_BATT_MAX_V 16.00
// REGULATOR VALUES
// POSITION
#define D_REG_KPV 0.0
#define D_REG_TDV 0.0
#define D_REG_TIV 0.0

#define D_REG_KPT 0.0
#define D_REG_TDT 0.0
#define D_REG_TIT 0.0

#define D_REG_RESTTILT 0

#define D_ANGLE_TIMEOUT 500       // Timeout between angle sents
#define D_ANGLE_LIMITS {-20, 20}  // Limit angle (degrees)
#define D_SPEED_LIMITS {-0.1, .1} // Speed limits (when moving) [m/s]

#define D_WHEEL_RADIUS 0.055 // [m]
// WiFi credentials
#define D_SSID "self-balancing-bot"
#define D_PASSWORD "123456789"

// File management (data saving)
#define D_MAX_RECORDING_SIZE 2000 // (Samples)

#define D_BATTERY_TIMEOUT 5000 // Update battery every x[ms]
#define D_INNER_LOOP_FREQ 100.0
#define D_INNER_LOOP_PERI 1.0 / D_INNER_LOOP_FREQ
#define D_OUTER_LOOP_FREQ 20.0
#define D_OUTER_LOOP_PERI 1.0 / D_OUTER_LOOP_FREQ

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

typedef struct
{
  float time;         // [s]
  float theta;        // [°]
  float theta_target; // [°]
  float position;     // [m]
  float speed;        // [m/s]
  float speed_target; // [m/s]
} S_DATA;

//-------------- FUNCTION PROTOTYPES ---------------
void init_littlefs();
void init_wifi();

void notify_clients(String msg);
void handle_websocket_message(void *arg, uint8_t *data, size_t len);
void on_event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void init_websocket();

float PID_update(PIDController *pid, float error, float dt);

void send_angle(float s);
void send_target_angle(float target_angle);
void send_motor_state(bool state);

bool is_angle_withing_range();
float map_f(float value, float min1, float max1, float min2, float max2);
float rad_s_to_ustp_s(float rad_s, uint16_t micro_stepping_value);
void send_recording();
float read_battery();
void send_battery(float battery_voltage);

//-------------- GLOBAL VARIABLES ---------------
AsyncWebServer server(80); // Web server
AsyncWebSocket ws("/ws");  // WebSocket endpoint

// Mpu unit
Adafruit_MPU6050 mpu;

// Motor drivers
TMC2226 tmc2226_left(0x00);
TMC2226 tmc2226_right(0x01);

float position = 0.0f;
float position_target = 0.0f;
float position_control = 0.0f;

float angleX = 0.0f;
float angleX_offset = 0.0f;

unsigned long lastMicros = 0;

float us_speed = 0;
float motor_speed = 0.0f;

float outer_loop_cnt = 0.0f;
float inner_loop_cnt = 0.0f;

float speed = 0.0f;
float speed_target = 0.0f;

float theta_target = 0.0f;

uint32_t angle_timer = 0;

float prev_gyro = 0.0f;

int8_t angle_limits[2] = D_ANGLE_LIMITS;
float speed_control_range[2] = D_SPEED_LIMITS;

bool motor_state = true;

S_DATA recording_data[D_MAX_RECORDING_SIZE];
bool recording = false;
uint16_t recording_size = 0;

uint16_t micro_stepping_value = 0;

float battery_timer = 0;

// PID controllers
PIDController speed_PID =
    {
        .kp = D_REG_KPV,
        .ki = D_REG_TIV,
        .kd = D_REG_TDV,

        .integral = 0.0f,
        .previousError = 0.0f,

        .outputMin = -10.0f, // max desired tilt [°]
        .outputMax = 10.0f};

PIDController tilt_PID =
    {
        .kp = D_REG_KPT,
        .ki = D_REG_TIT,
        .kd = D_REG_TDT,

        .integral = 0.0f,
        .previousError = 0.0f,

        .outputMin = -5 * M_PI, // Motor speed [rad/s]
        .outputMax = 5 * M_PI};

//-------------- SETUP FUNCTION ---------------
void setup()
{
  // Begin serial transmission
  Serial.begin(D_UART_BAUDRATE); // Open serial port
  Serial.println("Serial1 initialized");

  // Initializes TMC
  tmc2226_left.init();
  tmc2226_right.init();

  // Set chopper config
  TMC2226::CHOPCONF chopconf;

  chopconf.value = D_TMC_REGDFV_CHOPCONF; // DEFAULT VALUE
  chopconf.bits.mres = 0b0000;            // Microstepping
  micro_stepping_value = 256;

  tmc2226_left.write_to_register(TMC2226::E_REG_CHOPCONF, chopconf.bytes);
  tmc2226_right.write_to_register(TMC2226::E_REG_CHOPCONF, chopconf.bytes);

  // Initialize battery pin
  pinMode(D_PIN_BATTERY_LVL, INPUT);

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
  // Time reading
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  // OUTER LOOP
  outer_loop_cnt += dt;
  if (outer_loop_cnt > D_OUTER_LOOP_PERI)
  {
    // Update target based on joystick input (position_control)
    float speed_error = -(speed_target - speed);
    // Generate angle theta based on position Error
    theta_target = PID_update(&speed_PID, speed_error, outer_loop_cnt);
    outer_loop_cnt = 0;
  }

  // INNER LOOP
  inner_loop_cnt += dt;

  if (inner_loop_cnt > D_INNER_LOOP_PERI)
  {
    // ANGLE READING
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Accelerometer angle
    float accelAngleX =
        atan2(accel.acceleration.x,
              sqrt(accel.acceleration.y * accel.acceleration.y +
                   accel.acceleration.z * accel.acceleration.z)) *
        180.0f / PI;

    // Gyro rate (Adafruit returns rad/s)
    float gyroRateX = gyro.gyro.x * 180.0f / PI;
    gyroRateX = 0.9f * prev_gyro + 0.1f * gyroRateX;

    // Complementary filter
    angleX = 0.98f * (angleX + gyroRateX * inner_loop_cnt) + 0.02f * accelAngleX;

    // Defines the error from speed (and saved offset)
    float thetaError = theta_target + angleX_offset - angleX;

    // Generate motor speed from inner loop
    float motor_speed =
        PID_update(&tilt_PID, thetaError, inner_loop_cnt);

    // Convert rad/s to ustp/stp
    us_speed = rad_s_to_ustp_s(motor_speed, micro_stepping_value);

    // Chariot speed
    speed = motor_speed * D_WHEEL_RADIUS;

    // Compute position
    position += speed * inner_loop_cnt;

    // run motors (standalone speed)
    tmc2226_left.run_speed((int32_t)us_speed);
    tmc2226_right.run_speed(-(int32_t)us_speed);

    // If currently recording
    if (recording && recording_size < D_MAX_RECORDING_SIZE)
    {
      recording_data[recording_size].time = millis();
      recording_data[recording_size].speed_target = speed_target;
      recording_data[recording_size].speed = speed;
      recording_data[recording_size].theta_target = theta_target;
      recording_data[recording_size].theta = angleX;
      recording_data[recording_size].position = position;

      recording_size++;
    }
    inner_loop_cnt = 0;
  }

  // If angle out of safe range.
  if (!is_angle_withing_range())
  {
    motor_speed = 0;
    if (motor_state)
    {
      motor_state = false;
      tmc2226_left.disable();
      tmc2226_right.disable();
      send_motor_state(false);
    }
  }

  // Send angle and speed to clients (if not recording)
  if (!recording)
  {
    if (millis() - angle_timer > D_ANGLE_TIMEOUT)
    {
      angle_timer = millis();
      send_target_angle(theta_target);
      send_angle(angleX);
    }
    if (millis() - battery_timer > D_BATTERY_TIMEOUT)
    {
      battery_timer = millis();
      send_battery(read_battery());
    }
  }
}

//-------------- FUNCTION IMPLEMENTATIONS ---------------

/// @brief Transforms rad/s to microstep/s
/// @param rad_s
/// @param micro_stepping_value
/// @return
float rad_s_to_ustp_s(float rad_s, uint16_t micro_stepping_value)
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
/// @brief Send entire recording to client
void send_recording()
{
  // Set timestamp to 0 (for the first one)
  float first_time = recording_data[0].time;
  // Send the recording transmit signal
  ws.textAll("{\"id\":\"save_start\"}");
  ws.textAll("Time [ms],Speed_target [m/s],Speed [m/s],Theta_Target [°], Theta [°], Position [m]");
  String chunk;

  for (int i = 0; i < recording_size; i++)
  {
    chunk += String(recording_data[i].time - first_time, 3) + "," +
             String(recording_data[i].speed_target, 3) + "," +
             String(recording_data[i].speed, 3) + "," +
             String(recording_data[i].theta_target, 3) + "," +
             String(recording_data[i].theta, 3) + "," +
             String(recording_data[i].position, 3) + "\n";

    // Send data chunck wise
    if (chunk.length() > 4096)
    {
      ws.textAll(chunk);

      chunk = "";
    }
  }

  if (chunk.length())
  {
    ws.textAll(chunk);
  }

  ws.textAll("save_end");
}

/// @brief Sends battery infos to clients
/// @param battery_voltage
void send_battery(float battery_voltage)
{
  // Create JSON message, send it
  JSONVar json;
  json["id"] = "battery";
  json["voltage"] = String(battery_voltage, 2);
  json["percentage"] = String(map_f(battery_voltage, D_BATT_MIN_V, D_BATT_MAX_V, 0, 100), 2);
  String msg = JSON.stringify(json);
  notify_clients(msg);
}

/// @brief Sends target angle to clients
/// @param speed
void send_target_angle(float target_angle)
{
  // Create JSON message, send it
  JSONVar json;
  json["id"] = "target_angle";
  json["value"] = target_angle;
  String msg = JSON.stringify(json);
  notify_clients(msg);
}
// PID //

/// Update PID based on a given discrete jump.
float PID_update(PIDController *pid, float error, float dt)
{
  // Derivative
  float derivative = (error - pid->previousError) / dt;

  // Compute output WITHOUT updating integral yet
  float output =
      pid->kp * error +
      pid->ki * pid->integral +
      pid->kd * derivative;

  // Check if output would saturate
  bool saturatedHigh = output > pid->outputMax;
  bool saturatedLow = output < pid->outputMin;

  // Anti-windup: only integrate if it helps leave saturation
  if (!(saturatedHigh && error > 0) &&
      !(saturatedLow && error < 0))
  {
    pid->integral += error * dt;
  }

  // Recompute output with updated integral
  output =
      pid->kp * error +
      pid->ki * pid->integral +
      pid->kd * derivative;

  // Saturate output
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
        tmc2226_left.disable();
        tmc2226_right.disable();
        send_motor_state(false);
      }
      else if (msg_id == "motors_on")
      {
        if (is_angle_withing_range())
        {
          tmc2226_left.enable();
          tmc2226_right.enable();
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
          tilt_PID.kp = (double)msg["balance"]["kp"];
          tilt_PID.ki = (double)msg["balance"]["ti"];
          tilt_PID.kd = (double)msg["balance"]["td"];
        }

        if (msg["velocity"].hasOwnProperty("kp"))
        {
          speed_PID.kp = (double)msg["velocity"]["kp"];
          speed_PID.ki = (double)msg["velocity"]["ti"];
          speed_PID.kd = (double)msg["velocity"]["td"];
        }
      }
      else if (msg_id == "pid_request")
      {
        JSONVar json;
        json["id"] = "pid_values";
        json["balance"]["kp"] = tilt_PID.kp;
        json["balance"]["td"] = tilt_PID.kd;
        json["balance"]["ti"] = tilt_PID.ki;
        json["velocity"]["kp"] = speed_PID.kp;
        json["velocity"]["td"] = speed_PID.kd;
        json["velocity"]["ti"] = speed_PID.ki;
        String msg = JSON.stringify(json);
        notify_clients(msg);
      }
      else if (msg_id == "joystick-left")
      {
        // Adjust speed target from joystick
        speed_target = map_f((double)msg["y"], -100, 100, speed_control_range[0], speed_control_range[1]);
      }
      else if (msg_id == "record_start")
      {
        recording_size = 0;
        recording = true;
      }
      else if (msg_id == "record_stop")
      {
        send_recording();
        recording = false;
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

/// @brief Maps a value from one range to another and clamps the output.
/// @param value Input value
/// @param min1 Input range minimum
/// @param max1 Input range maximum
/// @param min2 Output range minimum
/// @param max2 Output range maximum
/// @return Mapped and clamped value
float map_f(float value, float min1, float max1, float min2, float max2)
{
  if (max1 == min1)
  {
    // Avoid division by zero
    return min2;
  }

  float t = (value - min1) / (max1 - min1);
  float result = min2 + t * (max2 - min2);

  // Clamp output
  if (min2 < max2)
  {
    if (result < min2)
      result = min2;
    if (result > max2)
      result = max2;
  }
  else
  {
    // Handle reversed output ranges
    if (result < max2)
      result = max2;
    if (result > min2)
      result = min2;
  }

  return result;
}

/// @brief Read battery level in mV
/// @return
float read_battery()
{
  uint32_t voltage = analogReadMilliVolts(D_PIN_BATTERY_LVL);
  float battery_voltage = voltage * D_BATT_RATIO / 1000; // [V]
  return battery_voltage;
}
