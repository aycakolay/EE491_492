#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <SoftwareSerial.h>

using namespace ControlTableItem;

// =================== TUNABLES ===================
const float rpm = 6.0;
const float DEG_MIN = 30.0f;
const float DEG_MAX = 300.0f;

// Timeout (ms) for waiting on MOVING register – safety net against infinite loop
const unsigned long MOVE_TIMEOUT_MS = 15000;

// ==================== Dynamixel IDs ====================
const uint8_t RX64_1_ID = 1; // Shoulder
const uint8_t RX64_2_ID = 2; // Elbow
const uint8_t AX18_3_ID = 3; // Wrist
const uint8_t AX18_4_ID = 4; // Gripper Right
const uint8_t AX18_5_ID = 5; // Gripper Left

// ==================== Buses and Pins =========================
SoftwareSerial soft_serial(7, 8); // Dynamixel shield RX TX
#define DXL_SERIAL Serial
#define DEBUG_SERIAL soft_serial
const int DXL_DIR_PIN = 2; // Dynamixel shield UART DIR

// ======================= OBJECTS =======================
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ==================== Protocol and baud ========================
const float DXL_PROTOCOL = 1.0f;
const uint32_t DXL_BAUD = 1000000; // 1 Mbps default

// ============= Configurations for stepper motor =============
const uint8_t STEP_PUL_PIN = 22;
const uint8_t STEP_DIR_PIN = 23;
const uint8_t STEP_EN_PIN = 24;

const float HOME_THETA1_DEG = 220.0f;  // Forward-facing
const float HOME_THETA2_DEG = 180.0f;  // Elbow up
const float HOME_THETA3_DEG = 50.0f;  // 55: horizontal, 145: vertical
const float HOME_THETA4_DEG = 240.0f;  // 240: open, 180: closed
const float HOME_THETA5_DEG = 150.0f;  // 150: open, 210: closed

// Steps per rev
const long STEPS_PER_REV = 3200; // Arrange this on driver switches

const unsigned int PULSE_US = 8;   // pulse width (µs)
const unsigned int STEP_INTERVAL_US = 3000; // constant step interval (µs), slightly slower

// ======================= HELPER FUNCTIONS =======================

// ================ Clamp angle to valid range ================
static inline float clampDeg(float deg) {
  if (deg < DEG_MIN) return DEG_MIN;
  if (deg > DEG_MAX) return DEG_MAX;
  return deg;
}

// ======================== Speed helpers =========================
static inline uint16_t rpmToRaw(float rpm) {
  if (rpm <= 0) return 0;
  float x = rpm / 0.111f;
  if (x > 1023) x = 1023;
  return (uint16_t)x;
}

static inline void setSpeedRPM(uint8_t id, float rpm) {
  dxl.writeControlTableItem(MOVING_SPEED, id, rpmToRaw(rpm));
}

// =============== Ensure servo is live and in position mode ================
static void ensureServoReady(uint8_t id) {
  dxl.ping(id);
  delay(100);

  dxl.torqueOff(id);
  delay(50);
  dxl.setOperatingMode(id, OP_POSITION);
  delay(50);

  // Set safe speed BEFORE enabling torque so it never snaps at max speed
  setSpeedRPM(id, rpm);
  delay(50);

  // Set goal to current position so torqueOn doesn't cause a jump
  float cur = dxl.getPresentPosition(id, UNIT_DEGREE);
  dxl.setGoalPosition(id, cur, UNIT_DEGREE);
  delay(50);

  dxl.torqueOn(id);
  delay(50);
}

// ================= Absolute move ================
static void moveAbsolute(uint8_t id, float goal_deg, float spd) {
  setSpeedRPM(id, spd);
  dxl.setGoalPosition(id, clampDeg(goal_deg), UNIT_DEGREE);
}

// ================= Relative move ================
static void moveRelative(uint8_t id, float delta_deg, float spd) {
  float current = dxl.getPresentPosition(id, UNIT_DEGREE);
  moveAbsolute(id, current + delta_deg, spd);
}

// ========== Wait until servo finishes moving ==========
static void waitUntilDone(uint8_t id) {
  delay(100); // short settling time so MOVING register sets to 1
  unsigned long t0 = millis();
  while (dxl.readControlTableItem(MOVING, id) == 1) {
    delay(50);
    if (millis() - t0 > MOVE_TIMEOUT_MS) break; // safety timeout
  }
}

// ========================= Stepper enable ==========================
static inline void stepperEnable(bool en) {
  digitalWrite(STEP_EN_PIN, en ? HIGH : LOW);
}

// ========================== Stepper Move Degrees =========================
static void stepperMoveDegrees(float delta_deg) {
  long steps = lround((delta_deg / 360.0) * STEPS_PER_REV);
  bool dir = (steps >= 0);
  steps = labs(steps);
  if (steps == 0) {
    return;
  }

  digitalWrite(STEP_DIR_PIN, dir ? LOW : HIGH);
  delayMicroseconds(200);

  stepperEnable(true);
  delayMicroseconds(200);

  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PUL_PIN, LOW);
    delayMicroseconds(PULSE_US);
    digitalWrite(STEP_PUL_PIN, HIGH);
    delayMicroseconds(STEP_INTERVAL_US);
  }

  stepperEnable(false);
}

// ====================== Home Position Sequence ========================
static void goHomePosition() {

  // Enable stepper throughout this routine.
  stepperEnable(true);

  // Stage 1: Move theta1 to home position, keep others at current positions.
  float hold_rx64_2 = dxl.getPresentPosition(RX64_2_ID, UNIT_DEGREE);
  float hold_ax18_3 = dxl.getPresentPosition(AX18_3_ID, UNIT_DEGREE);
  float hold_ax18_4 = dxl.getPresentPosition(AX18_4_ID, UNIT_DEGREE);
  float hold_ax18_5 = dxl.getPresentPosition(AX18_5_ID, UNIT_DEGREE);

  moveAbsolute(RX64_2_ID, hold_rx64_2, rpm);
  moveAbsolute(AX18_3_ID, hold_ax18_3, rpm);
  moveAbsolute(AX18_4_ID, hold_ax18_4, rpm);
  moveAbsolute(AX18_5_ID, hold_ax18_5, rpm);
  moveAbsolute(RX64_1_ID, HOME_THETA1_DEG, rpm);
  waitUntilDone(RX64_1_ID);

  // Stage 2: Move theta2 to home position, keep theta1 at home and others still.
  moveAbsolute(RX64_1_ID, HOME_THETA1_DEG, rpm);
  moveAbsolute(AX18_3_ID, hold_ax18_3, rpm);
  moveAbsolute(AX18_4_ID, hold_ax18_4, rpm);
  moveAbsolute(AX18_5_ID, hold_ax18_5, rpm);
  moveAbsolute(RX64_2_ID, HOME_THETA2_DEG, rpm);
  waitUntilDone(RX64_2_ID);

  // Stage 3: Move theta3 to home position, keep theta1/theta2 at home.
  moveAbsolute(RX64_1_ID, HOME_THETA1_DEG, rpm);
  moveAbsolute(RX64_2_ID, HOME_THETA2_DEG, rpm);
  moveAbsolute(AX18_4_ID, hold_ax18_4, rpm);
  moveAbsolute(AX18_5_ID, hold_ax18_5, rpm);
  moveAbsolute(AX18_3_ID, HOME_THETA3_DEG, rpm);
  waitUntilDone(AX18_3_ID);

  // Stage 4: Move theta4 to home position, keep all others still.
  moveAbsolute(RX64_1_ID, HOME_THETA1_DEG, rpm);
  moveAbsolute(RX64_2_ID, HOME_THETA2_DEG, rpm);
  moveAbsolute(AX18_3_ID, HOME_THETA3_DEG, rpm);
  moveAbsolute(AX18_4_ID, HOME_THETA4_DEG, rpm);
  moveAbsolute(AX18_5_ID, hold_ax18_5, rpm);
  waitUntilDone(AX18_4_ID);

  // Stage 5: Move theta5 to home position, keep all others still.
  moveAbsolute(RX64_1_ID, HOME_THETA1_DEG, rpm);
  moveAbsolute(RX64_2_ID, HOME_THETA2_DEG, rpm);
  moveAbsolute(AX18_3_ID, HOME_THETA3_DEG, rpm);
  moveAbsolute(AX18_4_ID, HOME_THETA4_DEG, rpm);
  moveAbsolute(AX18_5_ID, HOME_THETA5_DEG, rpm);
  waitUntilDone(AX18_5_ID);

  // Final hold: maintain all positions.
  moveAbsolute(RX64_1_ID, HOME_THETA1_DEG, rpm);
  moveAbsolute(RX64_2_ID, HOME_THETA2_DEG, rpm);
  moveAbsolute(AX18_3_ID, HOME_THETA3_DEG, rpm);
  moveAbsolute(AX18_4_ID, HOME_THETA4_DEG, rpm);
  moveAbsolute(AX18_5_ID, HOME_THETA5_DEG, rpm);

  // Keep stepper enabled after home as well.
  stepperEnable(true);

}

// ================================= SETUP =================================
void setup() {
  pinMode(STEP_PUL_PIN, OUTPUT);
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_EN_PIN,  OUTPUT);

  delay(100);

  digitalWrite(STEP_PUL_PIN, HIGH);
  digitalWrite(STEP_DIR_PIN, HIGH);
  stepperEnable(false);

  // Init Dynamixel bus
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);
  delay(300);  // let bus stabilise at 1 Mbps before first packet

  ensureServoReady(RX64_1_ID);
  ensureServoReady(RX64_2_ID);
  ensureServoReady(AX18_3_ID);
  ensureServoReady(AX18_4_ID);
  ensureServoReady(AX18_5_ID);

  delay(1000);
}

// ================================== LOOP ================================
void loop() {
  static bool ran = false;
  if (ran) { delay(1000); return; }
  ran = true;

  goHomePosition();

}