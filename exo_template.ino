/*
 * ============================================================================
 *  exo_template.ino  -  HIP EXO CONTROL TEMPLATE
 *  ESP32 WROOM-32 (38-pin) + Waveshare SN65HVD230 CAN board
 *  + CubeMars AK70-9 V3 (KV60) motors, 1 or 2
 * ============================================================================
 *
 *  A starting point for every exo program. The parts that already work on
 *  the real motor (CAN, the motor protocol, safety stops, logging, the
 *  Serial Monitor commands) live in their own tabs, so a new experiment only
 *  means writing a controller in my_controller.h.
 *
 *  TABS (files in this folder)
 *    exo_template.ino  this file: setup(), loop(), and the order things
 *                      happen in each tick. Read it to see the flow.
 *    config.h          pins, motor IDs, directions, loop rate, safety limits
 *    my_controller.h   YOUR CODE. The only file most programs change.
 *    joints.h          joint[i] feedback and commands (deg and rad)
 *    safety.h          run/stop state, command limits, fault checks
 *    logger.h          CSV data over serial
 *    console.h         Serial Monitor commands
 *    ak70.h            CubeMars CAN protocol (from ak70_motor_test)
 *
 *  NEW PROGRAM: File -> Save As... in the Arduino IDE, then edit the copy's
 *  my_controller.h. (Save As copies the whole folder and renames the .ino.)
 *
 *  UPLOAD: Tools -> Board -> "ESP32 Dev Module", pick the port, Upload.
 *  Type `stop` before uploading if a motor is running.
 *
 *  SERIAL MONITOR: 921600 baud, line ending "Newline".
 *    Press Enter on an EMPTY line at any time = STOP (motors limp).
 *    Lines starting with "#" are messages; other lines are CSV data.
 *
 *  FIRST RUN (motor clamped to the bench, nothing on the output)
 *    1. Upload, open the Serial Monitor. Expect "# Motor 1 (right, CAN ID 1) found".
 *    2. `status`. Turn the shaft a quarter turn by hand, `status` again:
 *       pos should change by about 90 deg. Check "feedback ... Hz" is
 *       about 500 (see config.h, LOOP TIMING).
 *    3. `zero`, then `run`. The example controller holds the shaft like a
 *       soft spring. Push it by hand: it should push back. Press Enter to stop.
 *    4. `log on` for data (or use exo_logger.py to save it to a file).
 *
 *  CUBEMARS TOOL SETTINGS (per motor, once): CAN rate 1M, its own CAN ID,
 *  CAN feedback "periodic" at 500 Hz (or "query-reply").
 *
 *  WIRING: ESP32 3V3 -> CAN board 3V3, GND -> CAN board GND and motor supply
 *  (-), GPIO17 -> CTX, GPIO16 -> CRX, CANH -> motor CAN_H (white),
 *  CANL -> motor CAN_L (blue). 120 ohm termination at the bus ends.
 *
 *  See README.md for the full guide.
 * ============================================================================
 */

#include <Arduino.h>
#include "config.h"
#include "ak70.h"
#include "joints.h"
#include "safety.h"
#include "logger.h"
#include "console.h"
#include "my_controller.h"

// Catch impossible settings at compile time instead of on the bench.
static_assert(NUM_MOTORS >= 1 && NUM_MOTORS <= MAX_MOTORS, "NUM_MOTORS must be 1 or 2");
static_assert(CONTROL_HZ >= 50 && CONTROL_HZ <= 1000, "CONTROL_HZ must be 50..1000");
static_assert(LOG_HZ > 0 && LOG_HZ <= CONTROL_HZ && CONTROL_HZ % LOG_HZ == 0,
              "LOG_HZ must divide CONTROL_HZ evenly (e.g. 500, 250, 125, 100, 50)");

const uint32_t TICK_US      = 1000000UL / CONTROL_HZ;   // 2000 us at 500 Hz
const float    TICK_S       = 1.0f / CONTROL_HZ;
const uint32_t LOG_EVERY_N  = CONTROL_HZ / LOG_HZ;      // ticks per CSV row

uint32_t lastTickUs = 0;   // when the current tick was scheduled
uint32_t runTicks   = 0;   // ticks since `run` (gives the controller its t)
uint32_t logCounter = 0;

// ---------------------------------------------------------------------------
//  `run`: check everything, then hand control to my_controller.h
// ---------------------------------------------------------------------------
void startRun() {
  if (runState == STATE_RUN) { Serial.println("# already running"); return; }
  updateJointFeedback();          // make sure the checks see the newest data
  if (!readyToRun()) return;      // prints the reason

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (joint[i].fbHz > 0 && joint[i].fbHz < CONTROL_HZ / 2) {
      Serial.printf("# note: motor %d feedback is only %lu Hz (see `status`)\n",
                    i + 1, (unsigned long)joint[i].fbHz);
    }
    joint[i].setLimp();
  }
  resetSafetyForRun();
  runTicks = 0;
  controllerStart();
  runState = STATE_RUN;
  Serial.println("# RUN: controller active. Empty line (Enter) = stop.");
}

// ---------------------------------------------------------------------------
//  One control tick. Runs CONTROL_HZ times per second.
// ---------------------------------------------------------------------------
void controlTick() {
  uint32_t t0 = micros();

  // 1. Snapshot the feedback in the joint frame (deg and rad)
  updateJointFeedback();

  // 2. Fault checks: anything wrong -> all motors limp
  if (runState == STATE_RUN) {
    char why[96];
    if (checkFaults(why, sizeof(why))) goLimp(why);
  }

  // 3. Your controller, then the safety limits on whatever it asked for
  for (int i = 0; i < NUM_MOTORS; i++) joint[i].setLimp();   // default: limp
  if (runState == STATE_RUN) {
    controllerStep(runTicks * TICK_S, TICK_S);
    runTicks++;
    limitCommands();             // goes limp itself on a NaN command
  }
  if (runState != STATE_RUN) {   // also catches a stop during this tick
    for (int i = 0; i < NUM_MOTORS; i++) joint[i].setLimp();
  }

  // 4. One CAN frame per motor
  sendJointCommands();

  // 5. CSV row every LOG_EVERY_N ticks
  if (logOn && ++logCounter >= LOG_EVERY_N) {
    logCounter = 0;
    writeLogRow(millis(), STATE_NAME[runState]);
  }

  uint32_t dt = micros() - t0;
  if (dt > loopMaxTickUs) loopMaxTickUs = dt;
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.setTxBufferSize(4096);   // must come before begin(); lets logging run without waiting
  Serial.begin(SERIAL_BAUD);
  delay(300);
  Serial.println();
  Serial.println("# ===== Hip exo template =====");

  initJoints();
  if (!ak70::begin()) {
    Serial.println("# CAN start FAILED. Check CAN_TX_PIN / CAN_RX_PIN in config.h.");
    while (true) delay(1000);
  }
  Serial.printf("# CAN up: 1 Mbit/s, TX=GPIO%d, RX=GPIO%d | %d motor(s) | loop %d Hz | "
                "log %d Hz | serial %d baud\n",
                CAN_TX_PIN, CAN_RX_PIN, NUM_MOTORS, CONTROL_HZ, LOG_HZ, SERIAL_BAUD);
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.printf("#   motor %d: %s, CAN ID %u, dir %+d\n",
                  i + 1, joint[i].name, joint[i].canId, joint[i].dir);
  }

  controllerInit();
  printHelp();
  Serial.println("# Motors are LIMP. `zero` then `run` to start.");

  bootMs = millis();
  lastTickUs = micros();
}

void loop() {
  // As often as possible: take in CAN frames and typed commands
  pollCan();
  pollSerial();

  // Fixed-rate control tick
  uint32_t nowUs = micros();
  if (nowUs - lastTickUs >= TICK_US) {
    if (nowUs - lastTickUs >= 2 * TICK_US) loopLateTicks++;      // a whole tick behind
    lastTickUs += TICK_US;
    if (nowUs - lastTickUs > 5 * TICK_US) lastTickUs = nowUs;    // far behind: don't try to catch up
    controlTick();
  }

  // A few times per second: CAN health and hints
  static uint32_t lastSlowMs = 0, lastRateMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastSlowMs >= 100) {
    lastSlowMs = nowMs;
    ak70::busTick();
    printHints();
  }
  if (nowMs - lastRateMs >= 1000) {
    lastRateMs = nowMs;
    updateFeedbackRates();
  }
}
