#include <WiFi.h>
#include <websitekeys.h>
#include "html510.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_VL53L1X.h"
#include <math.h>

// ================= WIFI =================
const char* ssid = "LEDcontrol";
HTML510Server html(80);

// ================= IMU =================
#define SDA_PIN 36
#define SCL_PIN 37
Adafruit_BNO055 bno(55, 0x28, &Wire);

// ================= TOF =================
Adafruit_VL53L1X tofLeft, tofFront, tofRight;

#define XSHUT_LEFT   12
#define XSHUT_FRONT  13
#define XSHUT_RIGHT  14

int16_t distLeft = -1;
int16_t distFront = -1;
int16_t distRight = -1;

// ================= MOTORS =================
const int pwmL = 15;
const int pwmR = 40;
const int fwdL = 6;
const int revL = 7;
const int fwdR = 41;
const int revR = 42;
const int PWM_RES  = 12;
const int MAX_DUTY = 4095;

// ================= SPEED =================
int basePWM = 25;
#define TURN_PWM 20

// ================= WALL FOLLOW =================
#define WALL_TARGET_MM 100
#define WALL_KP        0.2
#define MAX_BIAS       10
#define WALL_DEADBAND  15

// ================= YAW =================
float yawRaw = 0;
float yawOffset = 0;
float yawRobot = 0;
float targetYaw = 0;
float headingHold = 0;
bool yawZeroed = false;

#define YAW_KP  0.8
#define YAW_TOL 3.0

float lastLPWM = 0;
float lastRPWM = 0;

// ================= STATE =================
// 0 stop, 1 fwd, 2 back, 3 left, 4 right, 5 turn
bool autoMode = false;
int motion = 0;

// ================= UTILS =================
float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

float quatYaw(float w, float x, float y, float z) {
  return atan2f(
    2 * (w * z + x * y),
    1 - 2 * (y * y + z * z)
  ) * 180 / M_PI;
}

void setPWM(int l, int r) {
  l = constrain(l, 0, 100);
  r = constrain(r, 0, 100);
  ledcWrite(pwmL, (MAX_DUTY * l) / 100);
  ledcWrite(pwmR, (MAX_DUTY * r) / 100);
  lastLPWM = l;
  lastRPWM = r;
}

void stopMotors() {
  setPWM(0, 0);
  digitalWrite(fwdL, LOW);
  digitalWrite(revL, LOW);
  digitalWrite(fwdR, LOW);
  digitalWrite(revR, LOW);
}

// ================= TOF =================
void updateTOF() {
  if (tofLeft.dataReady()) {
    distLeft = tofLeft.distance();
    tofLeft.clearInterrupt();
  }

  if (tofFront.dataReady()) {
    distFront = tofFront.distance();
    tofFront.clearInterrupt();
  }

  // if (tofRight.dataReady()) {
  //   distRight = tofRight.distance();
  //   tofRight.clearInterrupt();
  // }
}

// ================= TURN (AUTO) =================
void startTurn(float delta) {
  targetYaw = wrap180(yawRobot + delta);
  motion = 5;
}

void handleTurn() {
  float err = wrap180(targetYaw - yawRobot);

  if (fabs(err) < YAW_TOL) {
    yawOffset = yawRaw;
    yawRobot = 0;
    stopMotors();
    motion = 1;
    return;
  }

  if (err > 0) {
    digitalWrite(fwdL, LOW);
    digitalWrite(revL, HIGH);
    digitalWrite(fwdR, HIGH);
    digitalWrite(revR, LOW);
  } else {
    digitalWrite(fwdL, HIGH);
    digitalWrite(revL, LOW);
    digitalWrite(fwdR, LOW);
    digitalWrite(revR, HIGH);
  }

  setPWM(TURN_PWM, TURN_PWM);
}

// ================= WALL FOLLOW =================
void wallFollow() {

  if (motion == 5) {
    handleTurn();
    return;
  }

  if (distLeft > 250) {
    startTurn(+5);
    return;
  }

  if (distFront > 150 && distLeft > 0) {

    float wallErr = WALL_TARGET_MM - distLeft;
    if (abs(wallErr) < WALL_DEADBAND) wallErr = 0;

    float bias =
      WALL_KP * wallErr +
      YAW_KP * (yawRobot - headingHold);

    bias = constrain(bias, -MAX_BIAS, MAX_BIAS);

    digitalWrite(fwdL, HIGH);
    digitalWrite(revL, LOW);
    digitalWrite(fwdR, HIGH);
    digitalWrite(revR, LOW);

    setPWM(basePWM - bias, basePWM + bias);
    motion = 1;
    return;
  }

  if (distFront < 150) {
    startTurn(-50);
  }
}

// ================= WEB =================
extern const char Slider[];

void Homepage() { html.sendhtml(Slider); }

void Forward() {
  autoMode = false;
  motion = 1;
  yawOffset = yawRaw;
  digitalWrite(fwdL, HIGH);
  digitalWrite(revL, LOW);
  digitalWrite(fwdR, HIGH);
  digitalWrite(revR, LOW);
  setPWM(25, 25);
}

void Backward() {
  autoMode = false;
  motion = 2;
  digitalWrite(fwdL, LOW);
  digitalWrite(revL, HIGH);
  digitalWrite(fwdR, LOW);
  digitalWrite(revR, HIGH);
  setPWM(basePWM, basePWM);
}

void Left() {
  autoMode = false;
  motion = 3;
  digitalWrite(fwdL, LOW);
  digitalWrite(revL, HIGH);
  digitalWrite(fwdR, HIGH);
  digitalWrite(revR, LOW);
  setPWM(basePWM, basePWM);
}

void Right() {
  autoMode = false;
  motion = 4;
  digitalWrite(fwdL, HIGH);
  digitalWrite(revL, LOW);
  digitalWrite(fwdR, LOW);
  digitalWrite(revR, HIGH);
  setPWM(basePWM, basePWM);
}

void StopMotor() {
  autoMode = false;
  motion = 0;
  stopMotors();
}

void AutoOn() {
  autoMode = true;
  motion = 1;
  yawOffset = yawRaw;
  yawRobot = 0;
}

void AutoOff() {
  autoMode = false;
  StopMotor();
}

void Zero() {
  yawOffset = yawRaw;
  yawRobot = 0;
}

void Status() {
  char msg[160];
  snprintf(
    msg, sizeof(msg),
    "%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%s",
    yawRobot, targetYaw, lastLPWM, lastRPWM,
    distLeft, distFront, distRight,
    autoMode ? "AUTO" : "MANUAL"
  );
  html.sendplain(msg);
}

// ================= SETUP =================
void setup() {

  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  WiFi.softAP(ssid);
  html.begin(80);

  html.attachHandler("/", Homepage);
  html.attachHandler("/FORWARD", Forward);
  html.attachHandler("/BACKWARD", Backward);
  html.attachHandler("/LEFT", Left);
  html.attachHandler("/RIGHT", Right);
  html.attachHandler("/STOPMOTOR", StopMotor);
  html.attachHandler("/AUTOON", AutoOn);
  html.attachHandler("/AUTOOFF", AutoOff);
  html.attachHandler("/ZERO", Zero);
  html.attachHandler("/STATUS", Status);

  bno.begin();
  bno.setExtCrystalUse(true);

  pinMode(fwdL, OUTPUT);
  pinMode(revL, OUTPUT);
  pinMode(fwdR, OUTPUT);
  pinMode(revR, OUTPUT);

  ledcAttach(pwmL, 500, PWM_RES);
  ledcAttach(pwmR, 500, PWM_RES);

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(100);
  tofLeft.begin(0x30, &Wire);
  tofLeft.startRanging();

  digitalWrite(XSHUT_FRONT, HIGH);
  delay(100);
  tofFront.begin(0x31, &Wire);
  tofFront.startRanging();

  // digitalWrite(XSHUT_RIGHT, HIGH);
  // delay(100);
  // tofRight.begin(0x32, &Wire);
  // tofRight.startRanging();

  Serial.println("=== ROBOT READY ===");
}

// ================= LOOP =================
void loop() {

  html.serve();
  updateTOF();

  imu::Quaternion q = bno.getQuat();
  yawRaw = wrap180(quatYaw(q.w(), q.x(), q.y(), q.z()));

  if (!yawZeroed) {
    yawOffset = yawRaw;
    yawZeroed = true;
  }

  yawRobot = wrap180(yawRaw - yawOffset);

  if (autoMode) wallFollow();
}