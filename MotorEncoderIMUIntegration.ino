#include <WiFi.h>
#include "html510.h"
#include <websitekeys.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <math.h>

const char* ssid = "LEDcontrol";
HTML510Server html(80);

// -------------------------------------------------------------
// IMU SETUP
// -------------------------------------------------------------
#define SDA_PIN 4
#define SCL_PIN 5
#define BNO55_ADDRESS_A 0x28

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO55_ADDRESS_A, &Wire);

// -------------------------------------------------------------
// FLAGS
// -------------------------------------------------------------
int flag = 0; 
// 0 stop, 1 forward, 2 backward, 3 left pivot, 4 right pivot, 5 turn-to-target

// -------------------------------------------------------------
// YAW VARIABLES
// -------------------------------------------------------------
float yawOffset   = 0.0f;
bool  yawZeroed   = false;

float yawRaw      = 0.0f;
float yawRobot    = 0.0f;

// forward/back heading reference (what we “lock” to when you press Forward/Backward)
float headingRef  = 0.0f;

// target yaw from the slider (used ONLY for spin-in-place turn-to-target)
float targetYaw   = 0.0f;

// error for forward/backward (relative to headingRef)
float yawErr      = 0.0f;
float yawFiltered = 0.0f;
float yawF1       = 0.0f;
float yawF2       = 0.0f;

const float filterAlpha = 0.10f;

// Wrap angle to [-180,180]
float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// Quaternion → yaw (degrees)
float quaternionToYaw(float w, float x, float y, float z) {
    float siny_cosp = 2.0f * (w*z + x*y);
    float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
    return atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

// -------------------------------------------------------------
// PID
// -------------------------------------------------------------
float Kp = 3.0f;
float Ki = 0.0f;
float Kd = 0.0f;

float integral   = 0.0f;
float lastError  = 0.0f;
unsigned long lastTime = 0;

void resetPID() {
  integral  = 0.0f;
  lastError = 0.0f;
  lastTime  = 0;   // important so dt is valid on next use
}

void resetFilter() {
  yawF1 = 0.0f;
  yawF2 = 0.0f;
  yawFiltered = 0.0f;
}

// -------------------------------------------------------------
// MOTOR SETUP
// -------------------------------------------------------------
const int pwmPin  = 10;
const int pwmPin2 = 1;

const int forwardPin   = 18;
const int backwardPin  = 19;

const int forwardPin2  = 6;
const int backwardPin2 = 7;

const int res = 12;
uint32_t freq = 500;
uint16_t dutyPercent = 100;
const uint32_t maxDuty = 4095;

float lastLeftPWM  = 0.0f;
float lastRightPWM = 0.0f;

// PWM writer
static void applyDutyDirect(uint16_t leftPercent, uint16_t rightPercent) {

  leftPercent  = constrain(leftPercent,  0, 100);
  rightPercent = constrain(rightPercent, 0, 100);

  uint16_t leftDuty  = (maxDuty * leftPercent)  / 100;
  uint16_t rightDuty = (maxDuty * rightPercent) / 100;

  // NOTE: preserve your original mapping (right channel on pwmPin, left on pwmPin2)
  ledcWrite(pwmPin,  rightDuty);
  ledcWrite(pwmPin2, leftDuty);

  lastLeftPWM  = leftPercent;
  lastRightPWM = rightPercent;
}

static void applyDutyPercent(uint16_t percent) {
  dutyPercent = percent;
  applyDutyDirect(percent, percent);
}

static void applyFrequency(uint32_t hz) {
  freq = hz;
  ledcAttach(pwmPin,  freq, res);
  ledcAttach(pwmPin2, freq, res);
  applyDutyPercent(dutyPercent);
}

// -------------------------------------------------------------
// WEB HANDLERS
// -------------------------------------------------------------
extern const char Slider[];

void Homepage() { html.sendhtml(Slider); }

void Frequency()  { applyFrequency(html.getVal()); }
void DutyCycle()  { applyDutyPercent(html.getVal()); }

void KpHandler() { Kp = html.getVal()/100.0f; }
void KiHandler() { Ki = html.getVal()/100.0f; }
void KdHandler() { Kd = html.getVal()/100.0f; }

// Slider target yaw (used ONLY for turn-to-target)
void SetYawHandler() {
  targetYaw = wrap180(html.getVal());
  resetPID();
}

// -------------------------------------------------------------
// Motion commands
// -------------------------------------------------------------
void Forward() {
  flag = 1;
  resetPID();
  resetFilter();

  // lock heading reference to current yaw so robot drives straight along this heading
  headingRef = yawRobot;

  digitalWrite(forwardPin,  HIGH);
  digitalWrite(backwardPin, LOW);
  digitalWrite(forwardPin2, HIGH);
  digitalWrite(backwardPin2, LOW);
}

void Backward() {
  flag = 2;
  resetPID();
  resetFilter();

  // same headingRef so backwards is “straight back” along same line
  headingRef = yawRobot;

  digitalWrite(forwardPin,  LOW);
  digitalWrite(backwardPin, HIGH);
  digitalWrite(forwardPin2, LOW);
  digitalWrite(backwardPin2, HIGH);
}

void Left()  { flag = 3; resetPID(); resetFilter(); }
void Right() { flag = 4; resetPID(); resetFilter(); }

void TurnToYawHandler() {
  flag = 5;
  resetPID();
  resetFilter();
}

void StopMotor() {
  flag = 0;
  resetPID();
  resetFilter();
  applyDutyDirect(0, 0);
}

// Re-zero yaw (define current physical heading as 0°)
// Also reset headingRef and targetYaw so everything is consistent.
void RezeroHandler() {
  yawOffset  = yawRaw;
  yawRobot   = 0.0f;
  headingRef = 0.0f;
  targetYaw  = 0.0f;
  resetPID();
  resetFilter();
}

// Status string: yawRobot, targetYaw, lastLeftPWM, lastRightPWM
void StatusHandler() {
  char msg[100];
  snprintf(msg, sizeof(msg), "%.2f,%.2f,%.1f,%.1f",
           yawRobot, targetYaw, lastLeftPWM, lastRightPWM);
  html.sendplain(msg);
}

// -------------------------------------------------------------
// FORWARD/BACK PID (heading hold relative to headingRef)
// -------------------------------------------------------------
void correctYaw(float error) {

  unsigned long now = millis();
  if (lastTime == 0) {
    lastTime = now;
    return;
  }

  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  lastTime = now;

  integral += error * dt;
  integral = constrain(integral, -150.0f, 150.0f);

  float derivative = (error - lastError) / dt;
  lastError = error;

  float out = Kp*error + Ki*integral + Kd*derivative;

  int base  = dutyPercent;
  int left, right;

  if (flag == 1) { // forward
    left  = base + out;
    right = base - out;
  }
  else if (flag == 2) { // backward
    left  = base - out;
    right = base + out;
  }
  else {
    return;
  }

  applyDutyDirect(constrain(left,0,100), constrain(right,0,100));
}

// -------------------------------------------------------------
// SETUP
// -------------------------------------------------------------
void setup() {

  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bno.begin()) {
    Serial.println("BNO055 ERROR!");
    while (1);
  }

  bno.setExtCrystalUse(true);

  WiFi.softAP(ssid, "");
  html.begin(80);

  html.attachHandler("/", Homepage);

  html.attachHandler("/FREQ?value=", Frequency);
  html.attachHandler("/DUTY?value=", DutyCycle);

  html.attachHandler("/KP?value=", KpHandler);
  html.attachHandler("/KI?value=", KiHandler);
  html.attachHandler("/KD?value=", KdHandler);

  html.attachHandler("/SETYAW?value=", SetYawHandler);
  html.attachHandler("/TURNTO",        TurnToYawHandler);

  html.attachHandler("/FORWARD",   Forward);
  html.attachHandler("/BACKWARD",  Backward);
  html.attachHandler("/LEFT",      Left);
  html.attachHandler("/RIGHT",     Right);
  html.attachHandler("/STOPMOTOR", StopMotor);
  html.attachHandler("/ZERO",      RezeroHandler);

  html.attachHandler("/STATUS", StatusHandler);

  pinMode(forwardPin,   OUTPUT);
  pinMode(backwardPin,  OUTPUT);
  pinMode(forwardPin2,  OUTPUT);
  pinMode(backwardPin2, OUTPUT);

  ledcAttach(pwmPin,  freq, res);
  ledcAttach(pwmPin2, freq, res);

  applyDutyPercent(dutyPercent);

  Serial.println("=== ROBOT READY ===");
}

// -------------------------------------------------------------
// LOOP
// -------------------------------------------------------------
void loop() {

  html.serve();

  // -----------------------------------------
  // QUATERNION YAW
  // -----------------------------------------
  imu::Quaternion q = bno.getQuat();
  yawRaw = quaternionToYaw(q.w(), q.x(), q.y(), q.z());
  yawRaw = wrap180(yawRaw);

  // First time: define yawOffset so yawRobot starts near 0
  if (!yawZeroed) {
    yawOffset = yawRaw;
    yawZeroed = true;
  }

  // robot yaw relative to yawOffset
  yawRobot = wrap180(yawRaw - yawOffset);

  // -----------------------------------------
  // Forward/back heading error (relative to headingRef)
  // -----------------------------------------
  yawErr = wrap180(yawRobot - headingRef);

  // Filter error for smoother PID
  yawF1 = filterAlpha*yawErr + (1.0f - filterAlpha)*yawF1;
  yawF2 = filterAlpha*yawF1  + (1.0f - filterAlpha)*yawF2;
  yawFiltered = yawF2;

  // =========================================
  // FORWARD/BACK PID (HEADING HOLD)
  // =========================================
  if (flag == 1 || flag == 2) {
    correctYaw(yawFiltered);
  }

  // =========================================
  // LEFT / RIGHT PIVOT
  // =========================================
  else if (flag == 3) {
    digitalWrite(forwardPin,  HIGH);
    digitalWrite(backwardPin, LOW);
    digitalWrite(forwardPin2, LOW);
    digitalWrite(backwardPin2, HIGH);
    applyDutyDirect(dutyPercent, dutyPercent);
  }

  else if (flag == 4) {
    digitalWrite(forwardPin,  LOW);
    digitalWrite(backwardPin, HIGH);
    digitalWrite(forwardPin2, HIGH);
    digitalWrite(backwardPin2, LOW);
    applyDutyDirect(dutyPercent, dutyPercent);
  }

  // =========================================
  // TURN TO TARGET (spin in place to slider yaw)
  // =========================================
  else if (flag == 5) {

    static int settledCount = 0;

    // Error to slider target (same sign convention as yawErr for consistency)
    float turnErr = wrap180(yawRobot - targetYaw);
    float absErr  = fabsf(turnErr);

    // Simple settle detector
    if (absErr < 0.5f)   // slightly looser than 0.25° to be realistic
      settledCount++;
    else
      settledCount = 0;

    if (settledCount > 10) {

        // Stop motors
        applyDutyDirect(0, 0);

        // Lock new heading as reference for forward/back
        headingRef = yawRobot;

        resetPID();
        resetFilter();
        settledCount = 0;

        Serial.print("[TURN] Target reached near ");
        Serial.print(targetYaw);
        Serial.print(" deg; yawRobot = ");
        Serial.println(yawRobot);

        // Done turning; go back to idle
        flag = 0;
        return;
    }

    // Decide spin direction based on sign of turnErr
    if (turnErr > 0) {
        digitalWrite(forwardPin,  HIGH);
        digitalWrite(backwardPin, LOW);
        digitalWrite(forwardPin2, LOW);
        digitalWrite(backwardPin2, HIGH);
    } 
    else {
        digitalWrite(forwardPin,  LOW);
        digitalWrite(backwardPin, HIGH);
        digitalWrite(forwardPin2, HIGH);
        digitalWrite(backwardPin2, LOW);
    }

    // Scale spin speed with magnitude of error
    float base = Kp;
    float cmd  = base * absErr;

    int rot = (int)cmd;

    const int MIN_ROT = 25;
    const int MAX_ROT = 90;

    rot = constrain(rot, MIN_ROT, MAX_ROT);

    applyDutyDirect(rot, rot);
  }

  // =========================================
  // IDLE
  // =========================================
  else {
    // Ensure motors are off when no command
    applyDutyDirect(0, 0);
  }

  delay(20);
}


// #include <WiFi.h>
// #include "html510.h"
// #include <websitekeys.h>
// #include <Wire.h>
// #include <Adafruit_BNO055.h>
// #include <Adafruit_Sensor.h>
// #include <math.h>

// const char* ssid = "LEDcontrol";
// HTML510Server html(80);

// // -------------------------------------------------------------
// // IMU SETUP
// // -------------------------------------------------------------
// #define SDA_PIN 4
// #define SCL_PIN 5
// #define BNO55_ADDRESS_A 0x28

// Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO55_ADDRESS_A, &Wire);

// // -------------------------------------------------------------
// // FLAGS
// // -------------------------------------------------------------
// int flag = 0; 
// // 0 stop, 1 forward, 2 backward, 3 left pivot, 4 right pivot,
// // 5 turn-to-target, 6 drive-distance

// // -------------------------------------------------------------
// // YAW VARIABLES
// // -------------------------------------------------------------
// float yawOffset   = 0.0f;
// bool  yawZeroed   = false;

// float yawRaw      = 0.0f;
// float yawRobot    = 0.0f;

// // forward/back heading reference (what we “lock” to when you press Forward/Backward)
// float headingRef  = 0.0f;

// // target yaw from the slider (used ONLY for spin-in-place turn-to-target)
// float targetYaw   = 0.0f;

// // error for forward/backward (relative to headingRef)
// float yawErr      = 0.0f;
// float yawFiltered = 0.0f;
// float yawF1       = 0.0f;
// float yawF2       = 0.0f;

// const float filterAlpha = 0.10f;

// // Wrap angle to [-180,180]
// float wrap180(float a) {
//   while (a > 180.0f) a -= 360.0f;
//   while (a < -180.0f) a += 360.0f;
//   return a;
// }

// // Quaternion → yaw (degrees)
// float quaternionToYaw(float w, float x, float y, float z) {
//     float siny_cosp = 2.0f * (w*z + x*y);
//     float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
//     return atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
// }

// // -------------------------------------------------------------
// // OUTER-LOOP YAW PID (IMU-based heading hold)
// // -------------------------------------------------------------
// float Kp = 3.0f;
// float Ki = 0.0f;
// float Kd = 0.0f;

// float integral   = 0.0f;
// float lastError  = 0.0f;
// unsigned long lastTime = 0;

// // These are the **commanded wheel speeds** from the yaw PID
// // in "counts per second" units (inner loop uses them).
// float targetSpeedL = 0.0f; // cps
// float targetSpeedR = 0.0f; // cps

// // Max reference speed corresponding to 100% "dutyPercent" from UI
// // (tune this for your robot; this is a decent starting point)
// const float MAX_SPEED_CPS = 200.0f; // counts per second at full speed

// void resetPID() {
//   integral  = 0.0f;
//   lastError = 0.0f;
//   lastTime  = 0;   // important so dt is valid on next use
// }

// void resetFilter() {
//   yawF1 = 0.0f;
//   yawF2 = 0.0f;
//   yawFiltered = 0.0f;
// }

// // -------------------------------------------------------------
// // MOTOR SETUP
// // -------------------------------------------------------------
// const int pwmPin  = 10;
// const int pwmPin2 = 1;

// const int forwardPin   = 18;
// const int backwardPin  = 19;

// const int forwardPin2  = 6;
// const int backwardPin2 = 7;

// const int res = 12;
// uint32_t freq = 500;
// uint16_t dutyPercent = 100;
// const uint32_t maxDuty = 4095;

// float lastLeftPWM  = 0.0f;
// float lastRightPWM = 0.0f;

// // PWM writer
// static void applyDutyDirect(uint16_t leftPercent, uint16_t rightPercent) {

//   leftPercent  = constrain(leftPercent,  0, 100);
//   rightPercent = constrain(rightPercent, 0, 100);

//   uint16_t leftDuty  = (maxDuty * leftPercent)  / 100;
//   uint16_t rightDuty = (maxDuty * rightPercent) / 100;

//   // NOTE: preserve your original mapping (right channel on pwmPin, left on pwmPin2)
//   ledcWrite(pwmPin,  rightDuty);
//   ledcWrite(pwmPin2, leftDuty);

//   lastLeftPWM  = leftPercent;
//   lastRightPWM = rightPercent;
// }

// static void applyDutyPercent(uint16_t percent) {
//   dutyPercent = percent;
//   applyDutyDirect(percent, percent);
// }

// static void applyFrequency(uint32_t hz) {
//   freq = hz;
//   ledcAttach(pwmPin,  freq, res);
//   ledcAttach(pwmPin2, freq, res);
//   applyDutyPercent(dutyPercent);
// }

// // -------------------------------------------------------------
// // ENCODERS + ODOMETRY
// // -------------------------------------------------------------
// // Pins – change to match wiring
// const int encA_L = 32;
// const int encB_L = 33;
// const int encA_R = 25;
// const int encB_R = 26;

// volatile long countL = 0;
// volatile long countR = 0;

// // 11 PPR × 4x quadrature = 44 counts per rev (adjust if your encoder differs)
// const float COUNTS_PER_REV = 44.0f;

// // Wheel + robot geometry (meters) – tweak these for your robot
// const float wheelCirc = 0.22f;   // e.g., 70 mm dia => 0.22 m circumference (placeholder)
// const float wheelBase = 0.20f;   // distance between left/right wheels (placeholder)

// // Integrated distances
// float distL = 0.0f;
// float distR = 0.0f;

// // Robot pose from odometry (meters, radians)
// float posX = 0.0f;
// float posY = 0.0f;
// float thetaOdom = 0.0f;           // fused heading (enc + IMU), radians

// // For incremental updates
// long lastCountL = 0;
// long lastCountR = 0;

// // Fusion weight: closer to 1 = trust encoders more, closer to 0 = trust IMU more
// const float fusionAlpha = 0.90f;

// // Signed wheel speeds in counts per second (inner loop uses magnitudes)
// float wheelSpeedL_cps = 0.0f;
// float wheelSpeedR_cps = 0.0f;

// // Drive X meters command state
// float driveTarget_m = 0.0f;   // signed, >0 forward, <0 backward
// float driveStart_m  = 0.0f;   // starting average distance
// int   driveDir      = 1;      // +1 forward, -1 backward

// // --- Encoder ISRs ---
// void IRAM_ATTR encA_L_ISR() {
//   if (digitalRead(encA_L) == digitalRead(encB_L)) countL++;
//   else countL--;
// }
// void IRAM_ATTR encB_L_ISR() {
//   if (digitalRead(encA_L) != digitalRead(encB_L)) countL++;
//   else countL--;
// }

// void IRAM_ATTR encA_R_ISR() {
//   if (digitalRead(encA_R) == digitalRead(encB_R)) countR++;
//   else countR--;
// }
// void IRAM_ATTR encB_R_ISR() {
//   if (digitalRead(encA_R) != digitalRead(encB_R)) countR++;
//   else countR--;
// }

// // -------------------------------------------------------------
// // INNER-LOOP WHEEL VELOCITY PID (per wheel)
// // -------------------------------------------------------------
// // These control how tightly PWM follows the commanded wheel speed
// // (in counts/sec). Start small and tune up.
// float KvP = 0.5f;   // speed PID P-gain
// float KvI = 0.0f;   // usually small or zero initially
// float KvD = 0.0f;   // often zero

// float vIntL     = 0.0f;
// float vIntR     = 0.0f;
// float vLastErrL = 0.0f;
// float vLastErrR = 0.0f;

// void resetVelPID() {
//   vIntL = vIntR = 0.0f;
//   vLastErrL = vLastErrR = 0.0f;
//   targetSpeedL = 0.0f;
//   targetSpeedR = 0.0f;
// }

// // This converts targetSpeedL/R (cps) + measured speeds into PWM
// // It is only used in flags 1, 2, 6 (forward/back/drive-distance)
// void updateWheelVelocityPID(float dt) {
//   if (dt <= 0.0f) return;
//   if (!(flag == 1 || flag == 2 || flag == 6)) return;

//   // Magnitudes for speed control (direction set by pins)
//   float cmdL = fabsf(targetSpeedL);
//   float cmdR = fabsf(targetSpeedR);

//   float measL = fabsf(wheelSpeedL_cps);
//   float measR = fabsf(wheelSpeedR_cps);

//   // Compute errors
//   float errL = cmdL - measL;
//   float errR = cmdR - measR;

//   // Integrals
//   vIntL += errL * dt;
//   vIntR += errR * dt;

//   // Optional integral clamp
//   vIntL = constrain(vIntL, -200.0f, 200.0f);
//   vIntR = constrain(vIntR, -200.0f, 200.0f);

//   // Derivatives
//   float dErrL = (errL - vLastErrL) / dt;
//   float dErrR = (errR - vLastErrR) / dt;
//   vLastErrL = errL;
//   vLastErrR = errR;

//   // Base PWM from requested speed magnitude
//   float basePWML = (cmdL / MAX_SPEED_CPS) * 100.0f;
//   float basePWMR = (cmdR / MAX_SPEED_CPS) * 100.0f;

//   // PID corrections
//   float pwmL = basePWML + (KvP * errL + KvI * vIntL + KvD * dErrL);
//   float pwmR = basePWMR + (KvP * errR + KvI * vIntR + KvD * dErrR);

//   // Clamp to [0,100]
//   pwmL = constrain(pwmL, 0.0f, 100.0f);
//   pwmR = constrain(pwmR, 0.0f, 100.0f);

//   applyDutyDirect((uint16_t)pwmL, (uint16_t)pwmR);
// }

// // -------------------------------------------------------------
// // WEB HANDLERS
// // -------------------------------------------------------------
// extern const char Slider[];

// void Homepage() { html.sendhtml(Slider); }

// void Frequency()  { applyFrequency(html.getVal()); }
// void DutyCycle()  { applyDutyPercent(html.getVal()); }

// void KpHandler() { Kp = html.getVal()/100.0f; }
// void KiHandler() { Ki = html.getVal()/100.0f; }
// void KdHandler() { Kd = html.getVal()/100.0f; }

// // Slider target yaw (used ONLY for turn-to-target)
// void SetYawHandler() {
//   targetYaw = wrap180(html.getVal());
//   resetPID();
// }

// // Rotate exactly ±90 using existing turn-to-target logic
// void Rot90Handler() {
//   int delta = html.getVal(); // expected +90 or -90
//   float d = (float)delta;
//   targetYaw = wrap180(yawRobot + d);
//   resetPID();
//   resetFilter();
//   resetVelPID();
//   flag = 5;   // reuse turn-to-target state
// }

// // Drive X meters handler (value in centimeters)
// void DriveMetersHandler() {
//   int cm = html.getVal();        // integer centimeters
//   driveTarget_m = cm / 100.0f;   // convert to meters
//   driveDir = (driveTarget_m >= 0.0f) ? +1 : -1;

//   // starting average distance (m)
//   float avgDist = 0.5f * (distL + distR);
//   driveStart_m = avgDist;

//   // heading lock: go straight relative to current yaw
//   headingRef = yawRobot;

//   // set direction pins based on driveDir
//   if (driveDir >= 0) {
//     digitalWrite(forwardPin,  HIGH);
//     digitalWrite(backwardPin, LOW);
//     digitalWrite(forwardPin2, HIGH);
//     digitalWrite(backwardPin2, LOW);
//   } else {
//     digitalWrite(forwardPin,  LOW);
//     digitalWrite(backwardPin, HIGH);
//     digitalWrite(forwardPin2, LOW);
//     digitalWrite(backwardPin2, HIGH);
//   }

//   flag = 6;      // drive distance mode
//   resetPID();
//   resetFilter();
//   resetVelPID();
// }

// // -------------------------------------------------------------
// // Motion commands
// // -------------------------------------------------------------
// void Forward() {
//   flag = 1;
//   resetPID();
//   resetFilter();
//   resetVelPID();

//   // lock heading reference to current yaw so robot drives straight along this heading
//   headingRef = yawRobot;

//   digitalWrite(forwardPin,  HIGH);
//   digitalWrite(backwardPin, LOW);
//   digitalWrite(forwardPin2, HIGH);
//   digitalWrite(backwardPin2, LOW);
// }

// void Backward() {
//   flag = 2;
//   resetPID();
//   resetFilter();
//   resetVelPID();

//   // same headingRef so backwards is “straight back” along same line
//   headingRef = yawRobot;

//   digitalWrite(forwardPin,  LOW);
//   digitalWrite(backwardPin, HIGH);
//   digitalWrite(forwardPin2, LOW);
//   digitalWrite(backwardPin2, HIGH);
// }

// void Left()  { flag = 3; resetPID(); resetFilter(); resetVelPID(); }
// void Right() { flag = 4; resetPID(); resetFilter(); resetVelPID(); }

// void TurnToYawHandler() {
//   flag = 5;
//   resetPID();
//   resetFilter();
//   resetVelPID();
// }

// void StopMotor() {
//   flag = 0;
//   resetPID();
//   resetFilter();
//   resetVelPID();
//   applyDutyDirect(0, 0);
// }

// // Re-zero yaw (define current physical heading as 0°)
// // Also reset headingRef and targetYaw so everything is consistent.
// void RezeroHandler() {
//   yawOffset  = yawRaw;
//   yawRobot   = 0.0f;
//   headingRef = 0.0f;
//   targetYaw  = 0.0f;
//   resetPID();
//   resetFilter();
//   resetVelPID();

//   // also reset odometry
//   posX = posY = 0.0f;
//   distL = distR = 0.0f;
//   thetaOdom = 0.0f;
//   lastCountL = countL;
//   lastCountR = countR;
// }

// // Status string: yawRobot, targetYaw, lastLeftPWM, lastRightPWM, posX, posY, avgDist
// void StatusHandler() {
//   char msg[120];
//   float avgDist = 0.5f * (distL + distR);
//   snprintf(msg, sizeof(msg), "%.2f,%.2f,%.1f,%.1f,%.3f,%.3f,%.3f",
//            yawRobot, targetYaw, lastLeftPWM, lastRightPWM,
//            posX, posY, avgDist);
//   html.sendplain(msg);
// }

// // -------------------------------------------------------------
// // OUTER-LOOP YAW PID (sets wheel speed commands, NOT PWM)
// // -------------------------------------------------------------
// void correctYaw(float error) {

//   unsigned long now = millis();
//   if (lastTime == 0) {
//     lastTime = now;
//     return;
//   }

//   float dt = (now - lastTime) / 1000.0f;
//   if (dt <= 0) dt = 0.001f;
//   lastTime = now;

//   integral += error * dt;
//   integral = constrain(integral, -150.0f, 150.0f);

//   float derivative = (error - lastError) / dt;
//   lastError = error;

//   float out = Kp*error + Ki*integral + Kd*derivative; // yaw correction in speed units (cps-ish)

//   // Convert UI dutyPercent to a **base speed** in counts/sec
//   float baseMag = (dutyPercent / 100.0f) * MAX_SPEED_CPS;

//   float leftCmd  = 0.0f;
//   float rightCmd = 0.0f;

//   if (flag == 1) { 
//     // forward: same as your prior mapping (base + out, base - out)
//     float base = +baseMag;
//     leftCmd  = base + out;
//     rightCmd = base - out;
//   }
//   else if (flag == 2) { 
//     // backward: mirror your prior sign logic
//     float base = -baseMag;
//     leftCmd  = base - out;
//     rightCmd = base + out;
//   }
//   else if (flag == 6) { 
//     // drive distance (use driveDir)
//     float base = (driveDir >= 0) ? +baseMag : -baseMag;
//     if (driveDir >= 0) {
//       leftCmd  = base + out;
//       rightCmd = base - out;
//     } else {
//       leftCmd  = base - out;
//       rightCmd = base + out;
//     }
//   }
//   else {
//     // other flags do not use yaw-based wheel speeds
//     return;
//   }

//   // Store as target speeds (inner velocity PID will convert to PWM)
//   targetSpeedL = leftCmd;
//   targetSpeedR = rightCmd;
// }

// // -------------------------------------------------------------
// // SETUP
// // -------------------------------------------------------------
// void setup() {

//   Serial.begin(115200);
//   delay(300);

//   Wire.begin(SDA_PIN, SCL_PIN);

//   if (!bno.begin()) {
//     Serial.println("BNO055 ERROR!");
//     while (1);
//   }

//   bno.setExtCrystalUse(true);

//   WiFi.softAP(ssid, "");
//   html.begin(80);

//   html.attachHandler("/", Homepage);

//   html.attachHandler("/FREQ?value=",   Frequency);
//   html.attachHandler("/DUTY?value=",   DutyCycle);

//   html.attachHandler("/KP?value=",     KpHandler);
//   html.attachHandler("/KI?value=",     KiHandler);
//   html.attachHandler("/KD?value=",     KdHandler);

//   html.attachHandler("/SETYAW?value=", SetYawHandler);
//   html.attachHandler("/TURNTO",        TurnToYawHandler);

//   html.attachHandler("/FORWARD",       Forward);
//   html.attachHandler("/BACKWARD",      Backward);
//   html.attachHandler("/LEFT",          Left);
//   html.attachHandler("/RIGHT",         Right);
//   html.attachHandler("/STOPMOTOR",     StopMotor);
//   html.attachHandler("/ZERO",          RezeroHandler);

//   // NEW handlers
//   html.attachHandler("/ROT90?value=",  Rot90Handler);
//   html.attachHandler("/DRIVEM?value=", DriveMetersHandler);

//   html.attachHandler("/STATUS",        StatusHandler);

//   pinMode(forwardPin,   OUTPUT);
//   pinMode(backwardPin,  OUTPUT);
//   pinMode(forwardPin2,  OUTPUT);
//   pinMode(backwardPin2, OUTPUT);

//   ledcAttach(pwmPin,  freq, res);
//   ledcAttach(pwmPin2, freq, res);

//   applyDutyPercent(dutyPercent);

//   // ----- Encoder pins -----
//   pinMode(encA_L, INPUT);
//   pinMode(encB_L, INPUT);
//   pinMode(encA_R, INPUT);
//   pinMode(encB_R, INPUT);

//   attachInterrupt(digitalPinToInterrupt(encA_L), encA_L_ISR, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encB_L), encB_L_ISR, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encA_R), encA_R_ISR, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encB_R), encB_R_ISR, CHANGE);

//   lastCountL = countL;
//   lastCountR = countR;

//   Serial.println("=== ROBOT READY (IMU + wheel velocity PID) ===");
// }

// // -------------------------------------------------------------
// // LOOP
// // -------------------------------------------------------------
// void loop() {

//   html.serve();

//   // -----------------------------------------
//   // QUATERNION YAW (IMU)
//   // -----------------------------------------
//   imu::Quaternion q = bno.getQuat();
//   yawRaw = quaternionToYaw(q.w(), q.x(), q.y(), q.z());
//   yawRaw = wrap180(yawRaw);

//   // First time: define yawOffset so yawRobot starts near 0
//   if (!yawZeroed) {
//     yawOffset = yawRaw;
//     yawZeroed = true;
//   }

//   // robot yaw relative to yawOffset (deg)
//   yawRobot = wrap180(yawRaw - yawOffset);

//   // -----------------------------------------
//   // ENCODER + ODOMETRY UPDATE (fusion)
//   // -----------------------------------------
//   static unsigned long lastVelTime = 0;
//   unsigned long now = millis();
//   float dtVel = 0.0f;
//   if (lastVelTime == 0) {
//     lastVelTime = now;
//   } else {
//     dtVel = (now - lastVelTime) / 1000.0f;
//     if (dtVel <= 0.0f) dtVel = 0.001f;
//     lastVelTime = now;
//   }

//   long cL = countL;
//   long cR = countR;

//   long dCountL = cL - lastCountL;
//   long dCountR = cR - lastCountR;
//   lastCountL = cL;
//   lastCountR = cR;

//   // Signed wheel speeds (counts/sec) for velocity PID
//   if (dtVel > 0.0f) {
//     wheelSpeedL_cps = (float)dCountL / dtVel;
//     wheelSpeedR_cps = (float)dCountR / dtVel;
//   }

//   // convert counts to distance [m]
//   float dL = (dCountL / COUNTS_PER_REV) * wheelCirc;
//   float dR = (dCountR / COUNTS_PER_REV) * wheelCirc;

//   distL += dL;
//   distR += dR;

//   float dCenter   = 0.5f * (dL + dR);
//   float dThetaEnc = (dR - dL) / wheelBase;   // radians

//   // IMU yaw in radians
//   float imuTheta = yawRobot * (M_PI / 180.0f);

//   // Complementary-style fusion on heading
//   thetaOdom = fusionAlpha * (thetaOdom + dThetaEnc) + (1.0f - fusionAlpha) * imuTheta;

//   // integrate pose using fused heading
//   posX += dCenter * cosf(thetaOdom);
//   posY += dCenter * sinf(thetaOdom);

//   // -----------------------------------------
//   // Forward/back heading error (relative to headingRef)
//   // -----------------------------------------
//   yawErr = wrap180(yawRobot - headingRef);

//   // Filter error for smoother PID
//   yawF1 = filterAlpha*yawErr + (1.0f - filterAlpha)*yawF1;
//   yawF2 = filterAlpha*yawF1  + (1.0f - filterAlpha)*yawF2;
//   yawFiltered = yawF2;

//   // =========================================
//   // FORWARD/BACK PID (HEADING HOLD) + DRIVE DIST
//   // =========================================
//   if (flag == 1 || flag == 2) {
//     // Outer loop sets targetSpeedL/R
//     correctYaw(yawFiltered);
//     // Inner loop uses encoders to realize those speeds
//     updateWheelVelocityPID(dtVel);
//   }
//   else if (flag == 6) {
//     // Drive X meters
//     float avgDist = 0.5f * (distL + distR);
//     float traveled = avgDist - driveStart_m;

//     if (fabsf(traveled) >= fabsf(driveTarget_m)) {
//       // reached target
//       applyDutyDirect(0, 0);
//       flag = 0;
//       resetPID();
//       resetFilter();
//       resetVelPID();
//       Serial.print("[DRIVE] Reached distance: ");
//       Serial.println(traveled);
//     } else {
//       // continue driving straight using yaw PID + wheel velocity PID
//       correctYaw(yawFiltered);
//       updateWheelVelocityPID(dtVel);
//     }
//   }

//   // =========================================
//   // LEFT / RIGHT PIVOT (still open-loop PWM)
// // =========================================
//   else if (flag == 3) {
//     digitalWrite(forwardPin,  HIGH);
//     digitalWrite(backwardPin, LOW);
//     digitalWrite(forwardPin2, LOW);
//     digitalWrite(backwardPin2, HIGH);
//     applyDutyDirect(dutyPercent, dutyPercent);
//   }

//   else if (flag == 4) {
//     digitalWrite(forwardPin,  LOW);
//     digitalWrite(backwardPin, HIGH);
//     digitalWrite(forwardPin2, HIGH);
//     digitalWrite(backwardPin2, LOW);
//     applyDutyDirect(dutyPercent, dutyPercent);
//   }

//   // =========================================
//   // TURN TO TARGET (spin in place to slider/ROT90 yaw) – open-loop PWM
//   // =========================================
//   else if (flag == 5) {

//     static int settledCount = 0;

//     // Error to slider/rot90 target
//     float turnErr = wrap180(yawRobot - targetYaw);
//     float absErr  = fabsf(turnErr);

//     // Simple settle detector
//     if (absErr < 0.5f)   // slightly looser than 0.25° to be realistic
//       settledCount++;
//     else
//       settledCount = 0;

//     if (settledCount > 10) {

//         // Stop motors
//         applyDutyDirect(0, 0);

//         // Lock new heading as reference for forward/back
//         headingRef = yawRobot;

//         resetPID();
//         resetFilter();
//         resetVelPID();
//         settledCount = 0;

//         Serial.print("[TURN] Target reached near ");
//         Serial.print(targetYaw);
//         Serial.print(" deg; yawRobot = ");
//         Serial.println(yawRobot);

//         // Done turning; go back to idle
//         flag = 0;
//         return;
//     }

//     // Decide spin direction based on sign of turnErr
//     if (turnErr > 0) {
//         digitalWrite(forwardPin,  HIGH);
//         digitalWrite(backwardPin, LOW);
//         digitalWrite(forwardPin2, LOW);
//         digitalWrite(backwardPin2, HIGH);
//     } 
//     else {
//         digitalWrite(forwardPin,  LOW);
//         digitalWrite(backwardPin, HIGH);
//         digitalWrite(forwardPin2, HIGH);
//         digitalWrite(backwardPin2, LOW);
//     }

//     // Scale spin speed with magnitude of error
//     float base = Kp;
//     float cmd  = base * absErr;

//     int rot = (int)cmd;

//     const int MIN_ROT = 25;
//     const int MAX_ROT = 90;

//     rot = constrain(rot, MIN_ROT, MAX_ROT);

//     applyDutyDirect(rot, rot);
//   }

//   // =========================================
//   // IDLE
//   // =========================================
//   else {
//     // Ensure motors are off when no command
//     applyDutyDirect(0, 0);
//   }

//   delay(20);
// }


