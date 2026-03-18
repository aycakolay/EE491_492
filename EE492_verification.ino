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

// -------- Target angles for verification (change these to test) --------
float TARGET_RX64_1 = 180.0f;  // ID 1 – Shoulder
float TARGET_RX64_2 = 200.0f;  // ID 2 – Elbow
float TARGET_AX18_3 = 160.0f;  // ID 3 – Wrist
float TARGET_AX18_4 =  60.0f;  // ID 4 – Gripper Right  (+angle)
float TARGET_AX18_5 = 240.0f;  // ID 5 – Gripper Left   (–angle, opposite of ID 4)

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
const float STEPPER_TEST_DEG = 180.0f;  // small test angle

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
    DEBUG_SERIAL.println(F("[STEP] 0 steps -> skip"));
    return;
  }

  DEBUG_SERIAL.print(F("[STEP] deg="));
  DEBUG_SERIAL.print(delta_deg, 2);
  DEBUG_SERIAL.print(F(" steps="));
  DEBUG_SERIAL.print(steps);
  DEBUG_SERIAL.print(F(" dir="));
  DEBUG_SERIAL.println(dir ? F("FWD") : F("REV"));

  // DIR- driven by MCU: LOW/HIGH select direction state for the opto input.
  digitalWrite(STEP_DIR_PIN, dir ? LOW : HIGH);
  delayMicroseconds(200);

  stepperEnable(true);
  DEBUG_SERIAL.println(F("[STEP] enable ON"));
  delayMicroseconds(200);

  for (long i = 0; i < steps; i++) {
    // PUL- active pulse for common-anode wiring is LOW.
    digitalWrite(STEP_PUL_PIN, LOW);
    delayMicroseconds(PULSE_US);
    digitalWrite(STEP_PUL_PIN, HIGH);
    delayMicroseconds(STEP_INTERVAL_US);
  }

  stepperEnable(false);
  DEBUG_SERIAL.println(F("[STEP] enable OFF"));
}

// ================================= SETUP =================================
void setup() {
  pinMode(STEP_PUL_PIN, OUTPUT);
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_EN_PIN,  OUTPUT);

  DEBUG_SERIAL.begin(115200);
  delay(100);

  // Common-anode idle state: keep -inputs HIGH (no current through optos).
  digitalWrite(STEP_PUL_PIN, HIGH);
  digitalWrite(STEP_DIR_PIN, HIGH);
  stepperEnable(false);
  DEBUG_SERIAL.println(F("[BOOT] JSS556 common-anode mode"));
  DEBUG_SERIAL.println(F("[BOOT] PUL-/DIR-/EN- -> pins 22/23/24, + -> 5V"));

  // Init Dynamixel bus
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);
  delay(300);  // let bus stabilise at 1 Mbps before first packet

  // ensureServoReady(RX64_1_ID);
  // ensureServoReady(RX64_2_ID);
  // ensureServoReady(AX18_3_ID);
  // ensureServoReady(AX18_4_ID);
  // ensureServoReady(AX18_5_ID);

  delay(1000);
}

// ================================== LOOP ================================
void loop() {
  static uint8_t cycles_done = 0;
  if (cycles_done >= 2) { delay(1000); return; }

  DEBUG_SERIAL.print(F("[TEST] Cycle "));
  DEBUG_SERIAL.print(cycles_done + 1);
  DEBUG_SERIAL.println(F("/2"));

  DEBUG_SERIAL.println(F("[TEST] Forward move"));
  stepperMoveDegrees(STEPPER_TEST_DEG);
  delay(500);

  DEBUG_SERIAL.println(F("[TEST] Return move"));
  stepperMoveDegrees(-STEPPER_TEST_DEG);
  delay(500);

  cycles_done++;
  if (cycles_done >= 2) {
    DEBUG_SERIAL.println(F("[TEST] Done"));
  }

}