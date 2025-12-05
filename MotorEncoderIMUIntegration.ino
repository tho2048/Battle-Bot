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

