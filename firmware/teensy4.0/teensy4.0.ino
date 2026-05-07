// Teensy 4.0 firmware: listens for drive commands over serial
// and drives 4 wheels using the current BLD70B + relay wiring with
// encoder-based PID velocity control and mecanum kinematics.

#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include <Wire.h>
#include <MPU6050.h>

// Forward declaration to avoid Arduino sketch preprocessor prototype issues.
struct WheelPID;

// ---------- Motor pin + encoder config ----------
// Right bottom wheel (RB)
const int RB_PWM = 5;
const int RB_DIR = 4;
const int RB_ENC_A = 21;  // green
const int RB_ENC_B = 20;  // white

// Left bottom wheel (LB)
const int LB_PWM = 9;
const int LB_DIR = 8;
const int LB_ENC_A = 14;  // green
const int LB_ENC_B = 15;  // white

// Right upper wheel (RU)
const int RU_PWM = 2;
const int RU_DIR = 3;
const int RU_ENC_A = 23;  // green
const int RU_ENC_B = 22;  // white

// Left upper wheel (LU)
const int LU_PWM = 6;
const int LU_DIR = 7;
const int LU_ENC_A = 17;  // green
const int LU_ENC_B = 16;  // white

// Relay assumptions (based on tests):
// - Relay inputs are ACTIVE-HIGH (HIGH = relay energized).
// - Right wheels are on NC contacts: FORWARD = relay OFF (LOW).
// - Left wheels are on NO contacts: FORWARD = relay ON (HIGH).

// ---------- Robot + encoder config (matches URDF dimensions) ----------
// From robot.urdf.xacro:
//   wheel_radius = 0.05 m
//   wheelbase    = 0.30 m  (front-back distance between wheel centers)
//   track_width  = 0.395 m (left-right distance between wheel centers)
const float WHEEL_RADIUS_M   = 0.05f;
const float HALF_LENGTH_M    = 0.15f;     // wheelbase / 2
const float HALF_WIDTH_M     = 0.1975f;   // track_width / 2
const float MAX_WHEEL_RAD_S  = 1.2f;      // very low speed for safe bringup

// Conservative chassis speed limits for safety (tune up later if needed)
const float MAX_VX_MPS       = 0.06f;  // forward/backward (m/s), safety-limited
const float MAX_VY_MPS       = 0.06f;  // lateral (m/s), safety-limited
const float MAX_WZ_RAD_S     = 0.20f;  // yaw rate (rad/s), safety-limited

// Hard PWM cap as an extra physical safety layer.
const uint8_t MAX_PWM_SAFE   = 60;     // out of 255, still safe but helps overcome static friction

// Per-side command sign calibration for mecanum (used later).
const float LEFT_CMD_SIGN    = 1.0f;
const float RIGHT_CMD_SIGN   = 1.0f;

// Calibration mode: ignore full mecanum kinematics and just apply a fixed, low
// open-loop PWM based ONLY on cmd_vx to make all four wheels spin the same way.
// This is purely for safe direction bringup. Set to true only when re-doing wheel direction bringup.
const bool USE_SIMPLE_FORWARD_CAL = false;
const uint8_t OPEN_LOOP_PWM = 40;      // low fixed PWM in calibration mode

// TEMP hardware sanity test:
// true  -> ignore ROS commands and test ONE relay + ONE motor output directly
// false -> normal behavior
const bool USE_RELAY_SANITY_TEST = false;
const uint8_t RELAY_TEST_PWM = 40;     // safe, low PWM
const uint32_t RELAY_TOGGLE_MS = 2000; // toggle RU_DIR every 2 seconds

// Bring-up safety: when true, firmware will not drive any wheel PWM/dir.
// This enables serial protocol + telemetry + watchdog validation without motion.
// Turn this OFF in later stages once encoders and direction are verified.
const bool DISABLE_MOTORS = false;

// Per-wheel relay state that corresponds to FORWARD motion.
// Derived from the wiring notes:
// - Left wheels on NO: forward typically relay ON
// - Right wheels on NC: forward typically relay OFF
const bool LU_FORWARD_RELAY_ON = true;
const bool LB_FORWARD_RELAY_ON = true;
const bool RU_FORWARD_RELAY_ON = true;
const bool RB_FORWARD_RELAY_ON = true;

// Encoder resolution: 600 P/R * 4 edges = 1200 counts/motor rev,
// with gearbox ≈ 6120 counts per wheel revolution.
const float ENCODER_TICKS_PER_REV = 6120.0f;  // counts per wheel rev

// Encoder sign calibration:
// Define which direction should make the corresponding count *increase*.
// Based on hand-test observations:
// - RB decreases when that wheel is rotated in the "forward" direction -> sign -1
// - RU decreases when that wheel is rotated in the "forward" direction -> sign -1
// - LB and LU increase when rotated in the "forward" direction -> sign +1
const int ENC_SIGN_RB = -1;
const int ENC_SIGN_LB = +1;
const int ENC_SIGN_RU = -1;
const int ENC_SIGN_LU = +1;

// Control loop
const float CONTROL_DT_S = 0.01f;  // 10 ms (100 Hz)

// ---------- Command + timing ----------
float cmd_vx = 0.0f;   // m/s (forward +, backward -)
float cmd_vy = 0.0f;   // m/s (left +, right -)
float cmd_wz = 0.0f;   // rad/s (CCW +)

// Watchdog: stop if no command for this many ms
const unsigned long CMD_TIMEOUT_MS = 1000;  // 1 second safety timeout
unsigned long last_cmd_time_ms = 0;

// ---------- ASCII command parsing state ----------
// Expect lines like: V:vx,vy,vz\n  (vx, vy in m/s, vz in rad/s)
char cmd_line_buf[64];
uint8_t cmd_line_len = 0;

// ---------- Encoder state ----------
volatile long rb_counts = 0;
volatile long lb_counts = 0;
volatile long ru_counts = 0;
volatile long lu_counts = 0;

// Previous counts for velocity estimation (updated in control loop)
long rb_counts_prev = 0;
long lb_counts_prev = 0;
long ru_counts_prev = 0;
long lu_counts_prev = 0;

// Measured wheel angular velocities (rad/s)
float rb_omega_meas = 0.0f;
float lb_omega_meas = 0.0f;
float ru_omega_meas = 0.0f;
float lu_omega_meas = 0.0f;

// Simple quadrature decoding: trigger on channel A, use B to decide direction
void isr_rb_enc() {
  bool a = digitalRead(RB_ENC_A);
  bool b = digitalRead(RB_ENC_B);
  if (a == b) {
    rb_counts++;
  } else {
    rb_counts--;
  }
}

void isr_lb_enc() {
  bool a = digitalRead(LB_ENC_A);
  bool b = digitalRead(LB_ENC_B);
  if (a == b) {
    lb_counts++;
  } else {
    lb_counts--;
  }
}

void isr_ru_enc() {
  bool a = digitalRead(RU_ENC_A);
  bool b = digitalRead(RU_ENC_B);
  if (a == b) {
    ru_counts++;
  } else {
    ru_counts--;
  }
}

void isr_lu_enc() {
  bool a = digitalRead(LU_ENC_A);
  bool b = digitalRead(LU_ENC_B);
  if (a == b) {
    lu_counts++;
  } else {
    lu_counts--;
  }
}

// ---------- IMU (MPU6050) state ----------
MPU6050 imu;
bool imu_ok = false;

float imu_yaw_deg   = 0.0f;
float imu_pitch_deg = 0.0f;
float imu_roll_deg  = 0.0f;

// Yaw axis mapping for MPU6050 mounting:
// 0 -> gyro X, 1 -> gyro Y, 2 -> gyro Z.
// If yaw stays almost constant during physical rotation, select the
// axis that changes for that motion. IMU_YAW_SIGN flips direction.
const int IMU_YAW_GYRO_AXIS = 2;
const float IMU_YAW_SIGN = 1.0f;

// Gyro bias on selected yaw axis (raw counts) measured at rest in setup.
int16_t imu_yaw_axis_offset = 0;

// Simple IMU update: read raw accel/gyro and compute pitch/roll from accel.
// Telemetry I:yaw,pitch,roll is in degrees:
// - pitch/roll: atan2 from accel (reasonable when not accelerating hard).
// - yaw: integrated gyro Z (drifts over time; no magnetometer — EKF on ROS can fuse better).
void updateImu(float dt) {
  if (!imu_ok) return;

  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  const float axg = ax / 16384.0f;
  const float ayg = ay / 16384.0f;
  const float azg = az / 16384.0f;

  imu_pitch_deg = atan2f(axg, sqrtf(ayg * ayg + azg * azg)) * 180.0f / PI;
  imu_roll_deg  = atan2f(ayg, sqrtf(axg * axg + azg * azg)) * 180.0f / PI;

  int16_t yaw_raw = gz;
  if (IMU_YAW_GYRO_AXIS == 0) {
    yaw_raw = gx;
  } else if (IMU_YAW_GYRO_AXIS == 1) {
    yaw_raw = gy;
  } else {
    yaw_raw = gz;
  }

  const float gyro_scale = 131.0f; // LSB/(°/s) for default ±250°/s
  float yaw_dps = IMU_YAW_SIGN * ((float)(yaw_raw - imu_yaw_axis_offset) / gyro_scale);
  imu_yaw_deg += yaw_dps * dt;
}

// ---------- PID controller per wheel ----------
struct WheelPID {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
};

WheelPID pid_rb = {0.4f, 0.8f, 0.0f, 0.0f, 0.0f};
WheelPID pid_lb = {0.4f, 0.8f, 0.0f, 0.0f, 0.0f};
WheelPID pid_ru = {0.4f, 0.8f, 0.0f, 0.0f, 0.0f};
WheelPID pid_lu = {0.4f, 0.8f, 0.0f, 0.0f, 0.0f};

void resetPids() {
  pid_rb.integral = 0.0f; pid_rb.prev_error = 0.0f;
  pid_lb.integral = 0.0f; pid_lb.prev_error = 0.0f;
  pid_ru.integral = 0.0f; pid_ru.prev_error = 0.0f;
  pid_lu.integral = 0.0f; pid_lu.prev_error = 0.0f;
}

float pidUpdate(WheelPID &pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  pid.integral += error * dt;

  // Anti-windup clamp
  const float INTEGRAL_MAX = 100.0f;
  if (pid.integral > INTEGRAL_MAX)  pid.integral = INTEGRAL_MAX;
  if (pid.integral < -INTEGRAL_MAX) pid.integral = -INTEGRAL_MAX;

  float derivative = (error - pid.prev_error) / dt;
  pid.prev_error = error;

  float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

  // Map output to a normalized control effort in [-1, 1]
  const float MAX_OUTPUT = MAX_WHEEL_RAD_S;  // treat this as "full effort"
  if (output >  MAX_OUTPUT) output =  MAX_OUTPUT;
  if (output < -MAX_OUTPUT) output = -MAX_OUTPUT;

  return output / MAX_OUTPUT;  // [-1, 1]
}

// ---------- Motor helpers ----------
void stopAll() {
  analogWrite(RB_PWM, 0);
  analogWrite(LB_PWM, 0);
  analogWrite(RU_PWM, 0);
  analogWrite(LU_PWM, 0);
}

// dir_sign: +1 = forward, -1 = backward (per robot convention vs. wiring)
// is_left: true for left wheels, false for right wheels
// wheel_rad_s: desired wheel angular velocity (rad/s, robot frame)
void driveWheel(int pwmPin, int dirPin, int dir_sign, bool is_left, float wheel_rad_s) {
  if (DISABLE_MOTORS) return;

  // Select PID and measured velocity based on wheel
  WheelPID *pid = nullptr;
  float omega_meas = 0.0f;

  if (pwmPin == RB_PWM) {
    pid = &pid_rb;
    omega_meas = rb_omega_meas;
  } else if (pwmPin == LB_PWM) {
    pid = &pid_lb;
    omega_meas = lb_omega_meas;
  } else if (pwmPin == RU_PWM) {
    pid = &pid_ru;
    omega_meas = ru_omega_meas;
  } else if (pwmPin == LU_PWM) {
    pid = &pid_lu;
    omega_meas = lu_omega_meas;
  }

  float control_effort = 0.0f;
  if (USE_SIMPLE_FORWARD_CAL) {
    // Fixed low effort for predictable and safe wheel-direction calibration.
    if (fabs(wheel_rad_s) < 1e-3f) {
      control_effort = 0.0f;
    } else {
      float base = (float)OPEN_LOOP_PWM / 255.0f;
      control_effort = (wheel_rad_s > 0.0f) ? base : -base;
    }
  } else if (pid) {
    control_effort = pidUpdate(*pid, wheel_rad_s, omega_meas, CONTROL_DT_S);  // [-1, 1]
  } else {
    // Map commanded speed directly to effort
    float limited = wheel_rad_s;
    if (limited >  MAX_WHEEL_RAD_S) limited =  MAX_WHEEL_RAD_S;
    if (limited < -MAX_WHEEL_RAD_S) limited = -MAX_WHEEL_RAD_S;
    control_effort = limited / MAX_WHEEL_RAD_S;
  }

  // Anti-chatter: In closed-loop PID, it is possible for the controller output
  // to oscillate around 0, which would cause the relay "dir" pin to toggle
  // rapidly. For bring-up, lock the sign of the effort to the *commanded*
  // setpoint sign so relays only change when the setpoint meaning changes.
  if (!USE_SIMPLE_FORWARD_CAL && fabs(wheel_rad_s) > 1e-4f) {
    float desired_sign = (wheel_rad_s * dir_sign) >= 0.0f ? 1.0f : -1.0f;
    control_effort = fabs(control_effort) * desired_sign;
  } else if (!USE_SIMPLE_FORWARD_CAL) {
    control_effort = 0.0f;
  }

  // Clamp effort
  if (control_effort >  1.0f) control_effort =  1.0f;
  if (control_effort < -1.0f) control_effort = -1.0f;

  // Map magnitude to PWM (0..255)
  float mag = fabs(control_effort);
  uint8_t pwm = (uint8_t)(mag * 255.0f);
  if (pwm > MAX_PWM_SAFE) pwm = MAX_PWM_SAFE;

  // Determine "forward" based on NC/NO layout
  bool forward = (control_effort * dir_sign) >= 0.0f; // positive after sign -> forward

  // Use per-wheel relay calibration so full mecanum mode matches the
  // simple forward calibration behavior validated earlier.
  bool relayOn = false;
  if (pwmPin == LU_PWM) {
    relayOn = forward ? LU_FORWARD_RELAY_ON : !LU_FORWARD_RELAY_ON;
  } else if (pwmPin == LB_PWM) {
    relayOn = forward ? LB_FORWARD_RELAY_ON : !LB_FORWARD_RELAY_ON;
  } else if (pwmPin == RU_PWM) {
    relayOn = forward ? RU_FORWARD_RELAY_ON : !RU_FORWARD_RELAY_ON;
  } else if (pwmPin == RB_PWM) {
    relayOn = forward ? RB_FORWARD_RELAY_ON : !RB_FORWARD_RELAY_ON;
  }

  digitalWrite(dirPin, relayOn ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

// ---------- Parse an ASCII command line ----------
void handleCommandLine(const char* line) {
  // Expect: V:vx,vy,vz
  if (line[0] != 'V' && line[0] != 'v') {
    return;
  }

  float vx, vy, vz;
  int parsed = sscanf(line, "V:%f,%f,%f", &vx, &vy, &vz);
  if (parsed != 3) {
    parsed = sscanf(line, "v:%f,%f,%f", &vx, &vy, &vz);
    if (parsed != 3) return;
  }

  // Clamp commanded chassis speeds for safety
  if (vx >  MAX_VX_MPS) vx =  MAX_VX_MPS;
  if (vx < -MAX_VX_MPS) vx = -MAX_VX_MPS;
  if (vy >  MAX_VY_MPS) vy =  MAX_VY_MPS;
  if (vy < -MAX_VY_MPS) vy = -MAX_VY_MPS;
  if (vz >  MAX_WZ_RAD_S) vz =  MAX_WZ_RAD_S;
  if (vz < -MAX_WZ_RAD_S) vz = -MAX_WZ_RAD_S;

  cmd_vx = vx;
  cmd_vy = vy;
  cmd_wz = vz;
  last_cmd_time_ms = millis();

  // Safety: if an explicit stop command is received, immediately de-energize motors.
  // This avoids relying solely on the comms-loss watchdog for fast stop behavior.
  if (fabs(cmd_vx) < 1e-6f && fabs(cmd_vy) < 1e-6f && fabs(cmd_wz) < 1e-6f) {
    stopAll();
    resetPids();
  }

  // Debug: confirm command parsing.
  // Printed as a separate line WITHOUT ';' so the ROS bridge parser (which
  // expects E:...;I:...) will ignore it safely (logs a warning only if used).
  // Example: D:0.020,0.000,0.000
  Serial.print("D:");
  Serial.print(cmd_vx, 3);
  Serial.print(",");
  Serial.print(cmd_vy, 3);
  Serial.print(",");
  Serial.print(cmd_wz, 3);
  Serial.print("\n");
}

// ---------- Serial parsing (non-blocking, line-based) ----------
void parseSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (cmd_line_len > 0) {
        cmd_line_buf[cmd_line_len] = '\0';
        handleCommandLine(cmd_line_buf);
        cmd_line_len = 0;
      }
    } else {
      if (cmd_line_len < sizeof(cmd_line_buf) - 1) {
        cmd_line_buf[cmd_line_len++] = c;
      } else {
        // Overflow: reset buffer
        cmd_line_len = 0;
      }
    }
  }
}

// ---------- Setup & loop ----------
elapsedMicros control_timer;
elapsedMillis telemetry_timer;
elapsedMillis relay_test_timer;
bool relay_test_state = false;

void setup() {
  Serial.begin(115200);

  pinMode(RB_PWM, OUTPUT); pinMode(RB_DIR, OUTPUT);
  pinMode(LB_PWM, OUTPUT); pinMode(LB_DIR, OUTPUT);
  pinMode(RU_PWM, OUTPUT); pinMode(RU_DIR, OUTPUT);
  pinMode(LU_PWM, OUTPUT); pinMode(LU_DIR, OUTPUT);

  // Encoder inputs + ISRs (channel A: CHANGE, matches encodertest / diag pattern)
  pinMode(RB_ENC_A, INPUT_PULLUP); pinMode(RB_ENC_B, INPUT_PULLUP);
  pinMode(LB_ENC_A, INPUT_PULLUP); pinMode(LB_ENC_B, INPUT_PULLUP);
  pinMode(RU_ENC_A, INPUT_PULLUP); pinMode(RU_ENC_B, INPUT_PULLUP);
  pinMode(LU_ENC_A, INPUT_PULLUP); pinMode(LU_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RB_ENC_A), isr_rb_enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LB_ENC_A), isr_lb_enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RU_ENC_A), isr_ru_enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LU_ENC_A), isr_lu_enc, CHANGE);

  Wire.begin();
  imu.initialize();
  imu_ok = imu.testConnection();
  if (imu_ok) {
    delay(50);
    int32_t yaw_sum = 0;
    const int n = 200;
    for (int i = 0; i < n; i++) {
      int16_t ax, ay, az, gx, gy, gz;
      imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      int16_t yaw_raw = gz;
      if (IMU_YAW_GYRO_AXIS == 0) {
        yaw_raw = gx;
      } else if (IMU_YAW_GYRO_AXIS == 1) {
        yaw_raw = gy;
      } else {
        yaw_raw = gz;
      }
      yaw_sum += yaw_raw;
      delay(2);
    }
    imu_yaw_axis_offset = (int16_t)(yaw_sum / n);
  }

  stopAll();
  last_cmd_time_ms = millis();
  control_timer = 0;
  telemetry_timer = 0;
  relay_test_timer = 0;
}

void loop() {
  if (USE_RELAY_SANITY_TEST) {
    // Hard-stop all wheels except RU wheel test.
    analogWrite(RB_PWM, 0);
    analogWrite(LB_PWM, 0);
    analogWrite(LU_PWM, 0);

    // Toggle RU direction relay every RELAY_TOGGLE_MS while applying low PWM.
    if (relay_test_timer >= RELAY_TOGGLE_MS) {
      relay_test_timer = 0;
      relay_test_state = !relay_test_state;
      digitalWrite(RU_DIR, relay_test_state ? HIGH : LOW);
    }
    analogWrite(RU_PWM, RELAY_TEST_PWM);
    return;
  }

  // Parse any incoming serial data
  parseSerial();

  // Watchdog: stop if no command recently
  if (millis() - last_cmd_time_ms > CMD_TIMEOUT_MS) {
    // Avoid noisy spam when already in a stop command.
    bool had_nonzero_cmd = (fabs(cmd_vx) > 1e-6f) || (fabs(cmd_vy) > 1e-6f) || (fabs(cmd_wz) > 1e-6f);
    cmd_vx = 0.0f;
    cmd_vy = 0.0f;
    cmd_wz = 0.0f;
    if (had_nonzero_cmd) {
      Serial.print("W:CMD_TIMEOUT_MS exceeded, stopping (t=");
      Serial.print(millis());
      Serial.print(")\n");
    }
    stopAll();
    resetPids();
  }

  // Run control at ~100 Hz
  if (control_timer >= 10000) { // 10,000 us = 10 ms
    control_timer = 0;

    // Update IMU at control rate
    updateImu(CONTROL_DT_S);

    // --- Estimate wheel velocities from encoder counts ---
    long rb_now = rb_counts;
    long lb_now = lb_counts;
    long ru_now = ru_counts;
    long lu_now = lu_counts;

    long rb_delta = rb_now - rb_counts_prev;
    long lb_delta = lb_now - lb_counts_prev;
    long ru_delta = ru_now - ru_counts_prev;
    long lu_delta = lu_now - lu_counts_prev;

    // Encoder wiring mapping fix:
    // Hand-test observations show the RU/LU encoder channels are swapped physically,
    // so swap RU and LU deltas here to make ru_omega_meas/lss telemetry match
    // the expected (rb,lb,ru,lu) identities.
    long tmp = ru_delta;
    ru_delta = lu_delta;
    lu_delta = tmp;

    // Apply per-wheel encoder sign so omega_meas matches the "forward" convention.
    rb_delta *= ENC_SIGN_RB;
    lb_delta *= ENC_SIGN_LB;
    ru_delta *= ENC_SIGN_RU;
    lu_delta *= ENC_SIGN_LU;

    rb_counts_prev = rb_now;
    lb_counts_prev = lb_now;
    ru_counts_prev = ru_now;
    lu_counts_prev = lu_now;

    float rev_per_tick = 1.0f / ENCODER_TICKS_PER_REV;
    float ticks_to_rad = 2.0f * 3.14159265359f * rev_per_tick;
    float inv_dt = 1.0f / CONTROL_DT_S;

    rb_omega_meas = (float)rb_delta * ticks_to_rad * inv_dt;
    lb_omega_meas = (float)lb_delta * ticks_to_rad * inv_dt;
    ru_omega_meas = (float)ru_delta * ticks_to_rad * inv_dt;
    lu_omega_meas = (float)lu_delta * ticks_to_rad * inv_dt;

    // During initial bringup, ignore full mecanum and use cmd_vx only.
    // This verifies that all wheels spin the same way for a forward command.
    if (USE_SIMPLE_FORWARD_CAL) {
      if (fabs(cmd_vx) < 1e-4f) {
        stopAll();
        resetPids();
      } else {
        // Direct relay+PWM control for deterministic wheel-direction calibration.
        bool forward = (cmd_vx > 0.0f);

        bool lu_on = forward ? LU_FORWARD_RELAY_ON : !LU_FORWARD_RELAY_ON;
        bool lb_on = forward ? LB_FORWARD_RELAY_ON : !LB_FORWARD_RELAY_ON;
        bool ru_on = forward ? RU_FORWARD_RELAY_ON : !RU_FORWARD_RELAY_ON;
        bool rb_on = forward ? RB_FORWARD_RELAY_ON : !RB_FORWARD_RELAY_ON;

        digitalWrite(LU_DIR, lu_on ? HIGH : LOW);
        digitalWrite(LB_DIR, lb_on ? HIGH : LOW);
        digitalWrite(RU_DIR, ru_on ? HIGH : LOW);
        digitalWrite(RB_DIR, rb_on ? HIGH : LOW);

        analogWrite(LU_PWM, OPEN_LOOP_PWM);
        analogWrite(LB_PWM, OPEN_LOOP_PWM);
        analogWrite(RU_PWM, OPEN_LOOP_PWM);
        analogWrite(RB_PWM, OPEN_LOOP_PWM);
      }
    } else {
      // Full mecanum inverse kinematics (standard layout)
      float L = HALF_LENGTH_M;
      float W = HALF_WIDTH_M;
      float R = WHEEL_RADIUS_M;

      float vx = cmd_vx;
      float vy = cmd_vy;
      float wz = cmd_wz;

      float factor = 1.0f / R;
      float w_fl = factor * (vx - vy - (L + W) * wz);
      float w_fr = factor * (vx + vy + (L + W) * wz);
      float w_rl = factor * (vx + vy - (L + W) * wz);
      float w_rr = factor * (vx - vy + (L + W) * wz);

      driveWheel(LU_PWM, LU_DIR, +1, true,  LEFT_CMD_SIGN  * w_fl);
      driveWheel(RU_PWM, RU_DIR, +1, false, RIGHT_CMD_SIGN * w_fr);
      driveWheel(LB_PWM, LB_DIR, +1, true,  LEFT_CMD_SIGN  * w_rl);
      driveWheel(RB_PWM, RB_DIR, +1, false, RIGHT_CMD_SIGN * w_rr);
    }
  }

  // Telemetry at ~20 Hz (every 50 ms)
  if (telemetry_timer >= 50) {
    telemetry_timer = 0;
    Serial.print("E:");
    Serial.print(rb_counts * ENC_SIGN_RB);
    Serial.print(',');
    Serial.print(lb_counts * ENC_SIGN_LB);
    Serial.print(',');
    // Telemetry wiring mapping fix: swap RU/LU counts.
    Serial.print(lu_counts * ENC_SIGN_RU);
    Serial.print(',');
    Serial.print(ru_counts * ENC_SIGN_LU);
    Serial.print(";I:");
    Serial.print(imu_yaw_deg, 2);
    Serial.print(',');
    Serial.print(imu_pitch_deg, 2);
    Serial.print(',');
    Serial.print(imu_roll_deg, 2);
    Serial.print('\n');
  }
}