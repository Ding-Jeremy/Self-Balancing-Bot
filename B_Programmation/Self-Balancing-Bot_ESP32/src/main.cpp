/*
 * File        : main.cpp
 * Author      : Ding Jérémy
 * Date        : May 2026
 * Target      : ESP32-WROOM-32
 *
 * Description :
 * Firmware for the self-balancing robot developed as part of the
 * Bachelor's thesis "Self-Balancing Robot".
 *
 * Features:
 *  - Dual-loop PID controller (velocity + balance)
 *  - MPU6050 attitude estimation using a complementary filter
 *  - TMC2226 stepper motor control
 *  - Wi-Fi Access Point with WebSocket communication
 *  - Battery monitoring
 *  - Data recording for control analysis
 */
#include <Arduino.h>

//-------------- INCLUDES ---------------
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include "LittleFS.h"
#include <SPI.h>
#include <TMC2226.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//-------------- DEFINES ---------------

#define D_UART_BAUDRATE 9600

// BATTERY MANAGEMENT
#define D_PIN_BATTERY_LVL 35
#define D_BATT_RATIO 6.022 // Resistors ratio (Vbat/Vadc)

#define D_BATT_MIN_V 14.70
#define D_BATT_MAX_V 16.00

// ---------------- PID Controller Parameters ----------------
// Velocity controller
#define D_REG_KPV 20
#define D_REG_TDV 0.0
#define D_REG_TIV 0.0

// Balance controller
#define D_REG_KPT 1.53
#define D_REG_TDT 0.38
#define D_REG_TIT 0.0

// Rest Angle
#define D_BALANCE_ANGLE 2.1 // [°]
#define D_COMPLEMENTARY_FILTER_ALPHA 0.9f

#define D_DATASEND_PERIOD 500            // Data transmission period [ms]
#define D_ANGLE_LIMITS {-30, 30}         // Limit angles [°]
#define D_SPEED_CTRL_RANGE {-0.7, 0.7}   // Speed control lean angles [m/s]
#define D_SPEED_DELTA_CTRL_RANGE {-2, 2} // Max pivot speed [rad/s]

#define D_WHEEL_RADIUS 0.055            // [m]
#define D_MOTOR_MAX_SPEED (5.0f * M_PI) // [rad/s]

// WiFi credentials
#define D_SSID "self-balancing-bot"
#define D_PASSWORD "123456789"

// File management (data saving)
#define D_MAX_RECORDING_SIZE 2000 // (Samples)
#define D_RECORDING_PERIOD 20     // ms
#define D_RECORDING_CHUNK_SIZE 4095

#define D_BATTERY_SEND_PERIOD 5000 // Update battery every x [ms]

// Regulation loops frequencies
#define D_INNER_LOOP_FREQ 500.0
#define D_INNER_LOOP_PERI (1.0f / D_INNER_LOOP_FREQ)
#define D_OUTER_LOOP_FREQ 500.0
#define D_OUTER_LOOP_PERI (1.0f / D_OUTER_LOOP_FREQ)

// Structures
struct PIDController
{
  float kp;
  float ki;
  float kd;

  float integral;
  float previous_error;

  float output_min;
  float output_max;
};

struct RecordingData
{
  float time;         // [ms]
  float acceleration; // [rad/s^2]
  float theta;        // [°]
  float theta_target; // [°]
  float speed;        // [m/s]
  float speed_target; // [m/s]
  float position;     // [mm]
};

//-------------- FUNCTION PROTOTYPES ---------------
void init_littlefs();
void init_wifi();

void notify_clients(const String &msg);
void handle_websocket_message(void *arg, uint8_t *data, size_t len);
void on_event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void init_websocket();

float pid_update(PIDController *pid, float error, float dt);

void send_angle(float angle);
void send_target_angle(float target_angle);
void send_motor_state(bool state);
void send_battery(float battery_voltage);
void send_recording();

float read_battery();
bool is_angle_within_range();
float calculate_accelerometer_angle(const sensors_event_t &acceleration);
float map_f(float value, float min1, float max1, float min2, float max2);

//-------------- GLOBAL VARIABLES ---------------
AsyncWebServer server(80); // Web server
AsyncWebSocket ws("/ws");  // WebSocket endpoint

// IMU (MPU6050)
Adafruit_MPU6050 mpu;

// Motor drivers
TMC2226 tmc2226_left(0x00);
TMC2226 tmc2226_right(0x01);

float theta_measured = 0.0f;
float theta_offset = 0.0f;
float theta_target = 0.0f;

unsigned long last_micros = 0;

float motor_speed = 0.0f;

float outer_loop_cnt = 0.0f;
float inner_loop_cnt = 0.0f;

float acceleration = 0.0f;

float speed = 0.0f;
float speed_target = 0.0f;
float pivot_speed = 0.0f;

float position = 0.0f;

int8_t angle_limits[2] = D_ANGLE_LIMITS;
float speed_control_range[2] = D_SPEED_CTRL_RANGE;
float speed_delta_control_range[2] = D_SPEED_DELTA_CTRL_RANGE;

bool motor_state = false;

RecordingData recording_data[D_MAX_RECORDING_SIZE];
bool recording = false;
uint16_t recording_size = 0;
uint32_t recording_last_time = 0;

uint32_t angle_timer = 0;
uint32_t battery_timer = 0;

uint32_t move_wait_time = 1500; // ms
bool waiting = false;
uint32_t wait_start_time = 0;

bool bidirectional = true;
float current_dir = 1;

bool step_enable = false;
float step_speed_value = 2.0f; // m/s
uint32_t step_start_time = 0;
uint32_t step_speed_period = 10000; // ms
uint32_t step_speed_ton = 3000;     // ms

bool trap_enable = false;
float trap_peak = .7f; // m/s
uint32_t trap_start_time = 0;
uint32_t trap_speed_period = 5000; // ms
uint32_t trap_speed_trise = 2000;  // ms

// PID controllers
PIDController velocity_pid =
    {
        .kp = D_REG_KPV,
        .ki = D_REG_TIV,
        .kd = D_REG_TDV,

        .integral = 0.0f,
        .previous_error = 0.0f,

        .output_min = -5.0f, // max desired tilt [°]
        .output_max = 5.0f};

PIDController tilt_pid =
    {
        .kp = D_REG_KPT,
        .ki = D_REG_TIT,
        .kd = D_REG_TDT,

        .integral = 0.0f,
        .previous_error = 0.0f,

        .output_min = -200, // Motor acceleration [rad/s^2]
        .output_max = 200};

//-------------- SETUP FUNCTION ---------------
void setup()
{
  // Begin serial transmission
  Serial.begin(D_UART_BAUDRATE); // Open serial port
  Serial.println("Serial1 initialized");

  // Initializes TMC
  tmc2226_left.init();
  tmc2226_right.init();
  // Start MPU
  if (!mpu.begin())
  {
    Serial.println("MPU6050 initialization failed; motors remain disabled");
    while (true)
    {
      delay(1000);
    }
  }

  // Initialize MPU settings
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

  sensors_event_t initial_acceleration, initial_gyro, initial_temperature;
  mpu.getEvent(&initial_acceleration, &initial_gyro, &initial_temperature);
  theta_measured = calculate_accelerometer_angle(initial_acceleration);

  // Initialize battery pin
  pinMode(D_PIN_BATTERY_LVL, INPUT);

  // Start Wi-Fi and server
  init_wifi();
  init_littlefs();
  init_websocket();

  // Serve root HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", "text/html"); });

  // Serve static files
  server.serveStatic("/", LittleFS, "/");

  server.begin();

  delay(50);
  last_micros = micros();

  // Check if the bot is in a correct position
  // If angle out of safe range.

  if (is_angle_within_range())
  {
    motor_state = true;
    tmc2226_left.enable();
    tmc2226_right.enable();
    send_motor_state(true);
  }
  else
  {
    send_motor_state(false);
  }
}

//-------------- MAIN LOOP ---------------
void loop()
{

  // Time reading
  unsigned long now = micros();
  float dt = (now - last_micros) * 1e-6f;
  last_micros = now;

  // Speed STEP
  if (step_enable)
  {
    uint32_t elapsed = millis() - step_start_time;

    if (elapsed < step_speed_ton)
      speed_target = step_speed_value;
    else
      speed_target = 0;

    if (elapsed >= step_speed_period)
      step_start_time = millis();
  }

  if (trap_enable)
  {
    // Waiting between moves
    if (waiting)
    {
      speed_target = 0.0f;

      if (millis() - wait_start_time >= move_wait_time)
      {
        waiting = false;
        trap_start_time = millis(); // Start next move
      }
    }
    else
    {
      uint32_t elapsed = millis() - trap_start_time;

      uint32_t t_rise = trap_speed_trise;
      uint32_t t_flat = trap_speed_period - 2 * t_rise;

      if (elapsed < t_rise)
      {
        // Ramp up
        speed_target = current_dir * trap_peak * ((float)elapsed / t_rise);
      }
      else if (elapsed < (t_rise + t_flat))
      {
        // Constant speed
        speed_target = current_dir * trap_peak;
      }
      else if (elapsed < trap_speed_period)
      {
        // Ramp down
        uint32_t t = elapsed - (t_rise + t_flat);
        speed_target = current_dir * trap_peak * (1.0f - (float)t / t_rise);
      }
      else
      {
        // Motion finished
        speed_target = 0.0f;

        if (bidirectional)
          current_dir = -current_dir;

        waiting = true;
        wait_start_time = millis();
      }
    }
  }

  // OUTER LOOP
  outer_loop_cnt += dt;
  if (outer_loop_cnt > D_OUTER_LOOP_PERI)
  {

    // Update target based on joystick input (position_control)
    float speed_error = speed_target - speed;
    // Generate angle theta based on position Error
    theta_target = pid_update(&velocity_pid, speed_error, outer_loop_cnt);
    outer_loop_cnt = 0;
  }

  // INNER LOOP
  inner_loop_cnt += dt;

  if (inner_loop_cnt > D_INNER_LOOP_PERI)
  {
    // Update position with current speed
    position += inner_loop_cnt * speed;
    // theta_target = speed_target;
    //  ANGLE READING
    sensors_event_t accel,
        gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Accelerometer angle
    float accel_angle_x = calculate_accelerometer_angle(accel);

    // Gyro rate (Adafruit returns rad/s)
    float gyro_rate_x = gyro.gyro.x * 180.0f / PI;

    // Complementary filter
    theta_measured = D_COMPLEMENTARY_FILTER_ALPHA * (theta_measured + gyro_rate_x * inner_loop_cnt) +
                     (1.0f - D_COMPLEMENTARY_FILTER_ALPHA) * accel_angle_x;
    // Defines the error from speed (and saved offset + Hard coded offset)
    float theta_error = theta_target - (theta_measured - D_BALANCE_ANGLE - theta_offset);

    // Generate motor speed from inner loop
    acceleration =
        pid_update(&tilt_pid, theta_error, inner_loop_cnt);

    // Invert acceleration
    acceleration *= -1;
    motor_speed += acceleration * inner_loop_cnt;
    motor_speed = motor_speed > D_MOTOR_MAX_SPEED ? D_MOTOR_MAX_SPEED : motor_speed < -D_MOTOR_MAX_SPEED ? -D_MOTOR_MAX_SPEED
                                                                                                         : motor_speed;
    // Chariot speed
    speed = motor_speed * D_WHEEL_RADIUS;

    // run motors (standalone speed)
    tmc2226_left.run_speed(-motor_speed + pivot_speed);
    tmc2226_right.run_speed(motor_speed + pivot_speed);

    inner_loop_cnt = 0;
  }

  // If angle out of safe range.
  if (!is_angle_within_range())
  {

    if (motor_state)
    {
      motor_state = false;
      tmc2226_left.disable();
      tmc2226_right.disable();
      send_motor_state(false);
    }
  }
  if (!motor_state)
  {
    motor_speed = 0;
    speed = 0;
  }

  // Send angle and speed to clients (if not recording)
  if (!recording)
  {
    if (millis() - angle_timer > D_DATASEND_PERIOD)
    {
      angle_timer = millis();
      send_target_angle(theta_target);
      send_angle(theta_measured);
    }
    if (millis() - battery_timer > D_BATTERY_SEND_PERIOD)
    {
      battery_timer = millis();
      send_battery(read_battery());
    }
  }
  else // Currently recording
  {
    uint32_t now = millis();

    if (recording_size < D_MAX_RECORDING_SIZE &&
        now - recording_last_time >= D_RECORDING_PERIOD)
    {
      recording_last_time = now;

      recording_data[recording_size].time = now;
      recording_data[recording_size].acceleration = acceleration;
      recording_data[recording_size].speed_target = speed_target;
      recording_data[recording_size].speed = speed;
      recording_data[recording_size].theta_target = theta_target;
      recording_data[recording_size].theta = theta_measured - D_BALANCE_ANGLE;
      recording_data[recording_size].position = position * 1000.0f;

      recording_size++;
    }
  }
}
//-------------- FUNCTION IMPLEMENTATIONS ---------------

/// @brief Checks whether the measured angle is inside the safe range.
/// @return True when the robot may safely enable its motors.
bool is_angle_within_range()
{
  return (theta_measured - D_BALANCE_ANGLE > angle_limits[0] && theta_measured - D_BALANCE_ANGLE < angle_limits[1]);
}

/// @brief Calculates the robot angle from an accelerometer sample.
float calculate_accelerometer_angle(const sensors_event_t &acceleration)
{
  return atan2(acceleration.acceleration.x,
               sqrt(acceleration.acceleration.y * acceleration.acceleration.y +
                    acceleration.acceleration.z * acceleration.acceleration.z)) *
         180.0f / PI;
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
  float first_time = recording_size > 0 ? recording_data[0].time : 0.0f;
  // Send the recording transmit signal
  ws.textAll("{\"id\":\"save_start\"}");
  ws.textAll("Time [ms],Acceleration [rad/s^2],Speed_target [m/s],Speed [m/s],Theta_Target [deg],Theta [deg],Position [mm]\n");
  String chunk;

  for (int i = 0; i < recording_size; i++)
  {
    chunk += String(recording_data[i].time - first_time, 3) + "," +
             String(recording_data[i].acceleration, 3) + "," +
             String(recording_data[i].speed_target, 3) + "," +
             String(recording_data[i].speed, 3) + "," +
             String(recording_data[i].theta_target, 3) + "," +
             String(recording_data[i].theta, 3) + "," +
             String(recording_data[i].position, 1) + "\n";

    // Send data chunk-wise
    if (chunk.length() > D_RECORDING_CHUNK_SIZE)
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

/// @brief Updates a PID controller for one discrete time step.
float pid_update(PIDController *pid, float error, float dt)
{
  // Derivative
  float derivative = (error - pid->previous_error) / dt;

  // Compute output without updating the integral yet
  float output =
      pid->kp * error +
      pid->ki * pid->integral +
      pid->kd * derivative;

  // Check if output would saturate
  bool saturated_high = output > pid->output_max;
  bool saturated_low = output < pid->output_min;

  // Anti-windup: only integrate if it helps leave saturation
  if (!(saturated_high && error > 0) &&
      !(saturated_low && error < 0))
  {
    pid->integral += error * dt;
  }

  // Recompute output with updated integral
  output =
      pid->kp * error +
      pid->ki * pid->integral +
      pid->kd * derivative;

  // Saturate output
  if (output > pid->output_max)
    output = pid->output_max;
  else if (output < pid->output_min)
    output = pid->output_min;

  pid->previous_error = error;

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
void notify_clients(const String &msg)
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

  // Ignore partial, fragmented, or non-text messages.
  if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT)
  {
    return;
  }

  String message;
  message.reserve(len);
  for (size_t i = 0; i < len; i++)
  {
    message += static_cast<char>(data[i]);
  }

  JSONVar msg = JSON.parse(message);
  if (JSON.typeof(msg) != "object")
  {
    return;
  }

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
    if (is_angle_within_range())
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
    theta_offset = theta_measured;

    // Transmit saved angle to server
    JSONVar json;
    json["id"] = "calibration_angle";
    json["value"] = theta_offset;
    String msg = JSON.stringify(json);
    notify_clients(msg);
  }
  else if (msg_id == "pid_values")
  {
    // Safety check
    if (msg["balance"].hasOwnProperty("kp"))
    {
      tilt_pid.kp = (double)msg["balance"]["kp"];
      tilt_pid.ki = (double)msg["balance"]["ti"];
      tilt_pid.kd = (double)msg["balance"]["td"];
    }

    if (msg["velocity"].hasOwnProperty("kp"))
    {
      velocity_pid.kp = (double)msg["velocity"]["kp"];
      velocity_pid.ki = (double)msg["velocity"]["ti"];
      velocity_pid.kd = (double)msg["velocity"]["td"];
    }
  }
  else if (msg_id == "pid_request")
  {
    JSONVar json;
    json["id"] = "pid_values";
    json["balance"]["kp"] = tilt_pid.kp;
    json["balance"]["td"] = tilt_pid.kd;
    json["balance"]["ti"] = tilt_pid.ki;
    json["velocity"]["kp"] = velocity_pid.kp;
    json["velocity"]["td"] = velocity_pid.kd;
    json["velocity"]["ti"] = velocity_pid.ki;
    String msg = JSON.stringify(json);
    notify_clients(msg);
  }
  else if (msg_id == "joystick-left")
  {
    // Adjust speed target from joystick
    speed_target = map_f((double)msg["y"], -100, 100, speed_control_range[0], speed_control_range[1]);
  }
  else if (msg_id == "joystick-right")
  {
    pivot_speed = -map_f((double)msg["x"], -100, 100, speed_delta_control_range[0], speed_delta_control_range[1]);
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

/*
 * WebSocket event callback handler
 */
void on_event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    send_motor_state(motor_state);
    break;
  case WS_EVT_DISCONNECT:
    speed_target = 0.0f;
    pivot_speed = 0.0f;
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
