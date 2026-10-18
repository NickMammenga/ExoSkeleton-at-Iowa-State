/*
 * ============================================================================
 *  AK70-9 V3 (KV60) Motor Test Console
 *  ESP32 WROOM-32 (38-pin) + Waveshare SN65HVD230 CAN board
 * ============================================================================
 *
 *  Drive one CubeMars AK70-9 V3 from the Arduino Serial Monitor and stream
 *  CSV data (position, velocity, current, torque estimate, temperature, error)
 *  for logging. Uses the ESP32's built-in CAN controller (TWAI), so no extra
 *  libraries are needed. Works with Arduino-ESP32 core 2.x and 3.x.
 *
 *  PROTOCOL (CubeMars "AK Series Product Manual" v3.2.0, in the project files)
 *  --------------------------------------------------------------------------
 *   - CAN 1 Mbps. Every command is a 29-bit EXTENDED frame.
 *   - CAN ID = (control_mode << 8) | motor_id
 *       1 = current loop      3 = velocity (ERPM)    5 = set origin
 *       6 = position-velocity loop                    8 = MIT (force control)
 *   - V3 switches between servo and MIT modes automatically based on the ID;
 *     no "enter motor mode" frame is needed.
 *   - Motor status frame, ID 0x2900 | motor_id (8 bytes): pos int16 (0.1 deg),
 *     speed int16 (10 ERPM), current int16 (0.01 A), temp int8 (C), error.
 *   - Output torque = 1.0621 Nm/A x current (AK70-9, manual table).
 *
 *  WIRING (same as the project's "CAN Setup Guide")
 *  ------------------------------------------------
 *   ESP32 3V3     -> Waveshare 3V3   (SN65HVD230 is a 3.3 V part, not 5 V)
 *   ESP32 GND     -> Waveshare GND, and also to the motor supply's NEGATIVE
 *   ESP32 GPIO17  -> Waveshare CTX   (change CAN_TX_PIN below if yours differs)
 *   ESP32 GPIO16  -> Waveshare CRX   (change CAN_RX_PIN below if yours differs)
 *   Waveshare CANH -> motor CAN_H (white)
 *   Waveshare CANL -> motor CAN_L (blue)
 *   Termination: with everything powered OFF, measure CANH to CANL.
 *     ~60 ohm  = both ends terminated (ideal)
 *     ~120 ohm = one end terminated (usually fine on a short bench bus)
 *     open     = no termination (will not work)
 *
 *  BEFORE FIRST RUN (CubeMars Tool via the R-Link)
 *  -----------------------------------------------
 *   - Read the motor's CAN ID -> set MOTOR_ID_DEFAULT (or type `id <n>`).
 *   - CAN bit rate = 1M.
 *   - CAN feedback: "periodic" at >= 50 Hz, or "query-reply". Both work.
 *   - Then stop commanding the motor from CubeMars Tool. The R-Link talks to
 *     the motor over its serial port, so it can stay plugged in, but two
 *     controllers commanding one motor is asking for surprises.
 *
 *  SERIAL MONITOR: 115200 baud, line ending "Newline" (or "Both NL & CR").
 *  Pressing Enter on an EMPTY line = STOP (motor goes limp).
 *  Info lines start with "#" so a CSV parser can skip them.
 *
 *  FIRST TEST SEQUENCE (motor clamped to the bench, nothing attached)
 *  -----------------------------------------------------------------
 *   1. Upload, open Serial Monitor. Expect "# Motor 1 found".
 *      `log on`, turn the output by hand: pos_deg should follow.
 *      A quarter turn by hand should read ~90 deg (checks position scaling).
 *   2. `origin`        current position becomes 0; soft limits arm.
 *   3. `cur 1`         gentle push (~1.1 Nm). Enter to stop.
 *   4. `vel 30`        30 deg/s. One output revolution should take ~12 s.
 *                      ~8 s or ~18 s means POLE_PAIRS is wrong (try 14).
 *   5. `pv 45`         move to +45 deg at limited speed. `pv 0` to return.
 *   6. `hold`          MIT spring-damper at the current position.
 *                      Push it by hand: should feel like a soft spring.
 *   7. `sine 20 0.5`   20 deg / 0.5 Hz sine about current position,
 *                      with `log on` for tracking data.
 *
 *  TROUBLESHOOTING
 *  ---------------
 *   - No "Motor found": type `status` (it prints a one-line diagnosis), then
 *     `scan` to find the motor's CAN ID. txErr >= 128 = nobody is ACKing, so
 *     the motor isn't on the bus (48 V, CANH/CANL, connector). `raw on`
 *     prints every frame the ESP32 receives.
 *   - Never wire the CAN board to the pins labeled TX/RX (GPIO1/3): those
 *     are the USB serial pins.
 *   - `hold`/`sine` immediately stop with "feedback timeout": the motor may
 *     not send status frames in MIT mode on your firmware. Run `raw on`,
 *     then `hold`, and look at what IDs come back.
 *   - MIT torque/velocity scaling wrong: P/V/T ranges below must match the
 *     firmware. They come from the manual's AK70-9 table.
 * ============================================================================
 */

#include <Arduino.h>
#include "driver/twai.h"

// =============================== USER SETTINGS ==============================
#define CAN_TX_PIN        17       // ESP32 -> transceiver CTX
#define CAN_RX_PIN        16       // ESP32 <- transceiver CRX
#define SERIAL_BAUD       115200   // raise to 921600 for logging > 100 Hz
#define MOTOR_ID_DEFAULT  1        // CAN ID set in CubeMars Tool
#define CONTROL_HZ        100      // command + log rate

// ---- AK70-9 KV60 constants ----
const float GEAR_RATIO = 9.0f;
const float POLE_PAIRS = 21.0f;   // CubeMars spec page; not in the manual. Verify with test step 4
const float KT_OUTPUT  = 1.0621f; // Nm/A at the output shaft (manual v3.2.0 table)

// ---- MIT ranges for AK70-9 (manual v3.2.0 table). Must match firmware. ----
const float P_MIN  = -12.56f, P_MAX  = 12.56f;  // rad
const float V_MIN  = -30.0f,  V_MAX  = 30.0f;   // rad/s
const float T_MIN  = -32.0f,  T_MAX  = 32.0f;   // Nm
const float KP_MIN = 0.0f,    KP_MAX = 500.0f;  // Nm/rad
const float KD_MIN = 0.0f,    KD_MAX = 5.0f;    // Nm*s/rad

// ---- Bench-test safety limits. Raise deliberately, not casually. ----
const float LIM_CURRENT_A    = 5.0f;    // ~5.3 Nm at output
const float LIM_SPEED_DPS    = 360.0f;  // output deg/s
const float LIM_ACCEL_DPS2   = 720.0f;  // output deg/s^2
const float LIM_TORQUE_NM    = 7.0f;    // MIT feedforward torque
const float LIM_KP           = 60.0f;
const float LIM_KD           = 3.0f;
const float LIM_POS_DEG      = 100.0f;  // +/- from origin (when limits armed)
const float LIM_MIT_JUMP_DEG = 30.0f;   // refuse MIT targets farther than this
const float LIM_SINE_AMP_DEG = 45.0f;
const float LIM_SINE_HZ      = 2.0f;
const uint32_t FB_TIMEOUT_MS = 250;     // no feedback this long while active -> stop
// ============================================================================

enum CanCmd : uint32_t {
  CMD_DUTY = 0, CMD_CURRENT = 1, CMD_BRAKE = 2, CMD_RPM = 3,
  CMD_POS = 4, CMD_ORIGIN = 5, CMD_POS_SPD = 6, CMD_MIT = 8
};
const uint32_t FN_STATUS       = 0x29;   // motor -> host: real-time status
const uint32_t FN_SERVO_START  = 0x2C;   // motor -> host: entered servo mode (FA FB FC FD)
const uint8_t  ERR_DISABLE_ACK = 0x77;   // error byte = 0x77 once after a "disable" frame

enum Mode { MODE_LIMP, MODE_CURRENT, MODE_VEL, MODE_POSVEL, MODE_MIT, MODE_SINE };
const char* MODE_NAMES[] = { "limp", "cur", "vel", "pv", "mit", "sine" };

const char* LOG_HEADER = "t_ms,mode,des_deg,pos_deg,vel_dps,cur_A,tau_est_Nm,temp_C,err";

struct Feedback {
  bool     valid   = false;
  uint32_t t_ms    = 0;
  float    pos_deg = 0, vel_dps = 0, cur_A = 0;
  int8_t   temp_C  = 0;
  uint8_t  err     = 0;
};

// ------------------------------- state --------------------------------------
Feedback fb;
uint8_t  motorId     = MOTOR_ID_DEFAULT;
Mode     mode        = MODE_LIMP;
bool     logOn       = false;
bool     rawOn       = false;
bool     limitsArmed = false;

float curCmdA = 0, velCmdDps = 0;
float pvPosDeg = 0, pvSpdDps = 60, pvAccDps2 = 180, pvDir = 1;
float mitP = 0, mitV = 0, mitT = 0;          // rad, rad/s, Nm
float gainKp = 8.0f, gainKd = 0.5f;
float sineAmpDeg = 0, sineHz = 0, sineCenterDeg = 0;
uint32_t sineT0 = 0;
float desDeg = NAN;                          // desired position, for the log

uint32_t txDrops = 0;
uint32_t bootMs = 0;
bool     seenIds[256] = {};   // motor IDs heard on the bus (status frames)
bool     scanning = false;
const uint32_t TICK_US = 1000000UL / CONTROL_HZ;
uint32_t lastTickUs = 0;

// ------------------------------ helpers -------------------------------------
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

float clampWarn(float x, float lo, float hi, const char* what) {
  float y = clampf(x, lo, hi);
  if (y != x) Serial.printf("# %s clamped to %.2f (bench limit)\n", what, y);
  return y;
}

float dpsToErpm(float dps)  { return dps / 6.0f * GEAR_RATIO * POLE_PAIRS; }
float erpmToDps(float erpm) { return erpm * 6.0f / (GEAR_RATIO * POLE_PAIRS); }

void putI32(uint8_t* b, int32_t v) {
  b[0] = (uint8_t)(v >> 24); b[1] = (uint8_t)(v >> 16);
  b[2] = (uint8_t)(v >> 8);  b[3] = (uint8_t)v;
}
void putI16(uint8_t* b, int16_t v) {
  b[0] = (uint8_t)(v >> 8); b[1] = (uint8_t)v;
}

const char* errName(uint8_t e) {
  switch (e) {
    case 0: return "none";
    case 1: return "motor over-temp";
    case 2: return "over-current";
    case 3: return "over-voltage";
    case 4: return "under-voltage";
    case 5: return "encoder fault";
    case 6: return "MOSFET over-temp";
    case 7: return "motor locked/stalled";
    default: return "unknown";
  }
}

bool feedbackFresh() { return fb.valid && (millis() - fb.t_ms) < FB_TIMEOUT_MS; }

// -------------------------------- CAN ---------------------------------------
bool canBegin() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g.tx_queue_len = 10;
  g.rx_queue_len = 32;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

void canSend(uint32_t cmd, const uint8_t* data, uint8_t len) {
  twai_message_t m = {};
  m.extd = 1;
  m.identifier = (cmd << 8) | motorId;
  m.data_length_code = len;
  memcpy(m.data, data, len);
  if (twai_transmit(&m, 0) != ESP_OK) txDrops++;   // never block the loop
}

// Servo mode: current loop, A
void sendCurrent(float amps) {
  uint8_t b[4];
  putI32(b, (int32_t)lroundf(amps * 1000.0f));
  canSend(CMD_CURRENT, b, 4);
}

// Servo mode: velocity loop, output deg/s -> ERPM
void sendVelocity(float dps) {
  uint8_t b[4];
  putI32(b, (int32_t)lroundf(dpsToErpm(dps)));
  canSend(CMD_RPM, b, 4);
}

// Servo mode: position-velocity loop. deg, deg/s, deg/s^2.
// Speed/accel are signed with the direction of travel, as in the manual's examples.
void sendPosVel(float deg, float dps, float dps2) {
  uint8_t b[8];
  putI32(b, (int32_t)lroundf(deg * 10000.0f));
  putI16(b + 4, (int16_t)clampf(roundf(dpsToErpm(dps) / 10.0f), -32768, 32767));
  putI16(b + 6, (int16_t)clampf(roundf(dpsToErpm(dps2) / 10.0f), -32768, 32767));
  canSend(CMD_POS_SPD, b, 8);
}

// Set origin: 0 = temporary (lost on power-off), 1 = permanent
void sendOrigin(bool permanent) {
  uint8_t b[1] = { (uint8_t)(permanent ? 1 : 0) };
  canSend(CMD_ORIGIN, b, 1);
}

uint16_t floatToUint(float x, float lo, float hi, int bits) {
  x = clampf(x, lo, hi);
  return (uint16_t)((x - lo) * (float)((1 << bits) - 1) / (hi - lo));
}

// MIT mode: rad, rad/s, Nm/rad, Nm*s/rad, Nm
void sendMit(float p, float v, float kp, float kd, float t) {
  uint16_t p_i  = floatToUint(p,  P_MIN,  P_MAX,  16);
  uint16_t v_i  = floatToUint(v,  V_MIN,  V_MAX,  12);
  uint16_t kp_i = floatToUint(kp, KP_MIN, KP_MAX, 12);
  uint16_t kd_i = floatToUint(kd, KD_MIN, KD_MAX, 12);
  uint16_t t_i  = floatToUint(t,  T_MIN,  T_MAX,  12);
  uint8_t b[8];
  b[0] = kp_i >> 4;
  b[1] = ((kp_i & 0xF) << 4) | (kd_i >> 8);
  b[2] = kd_i & 0xFF;
  b[3] = p_i >> 8;
  b[4] = p_i & 0xFF;
  b[5] = v_i >> 4;
  b[6] = ((v_i & 0xF) << 4) | (t_i >> 8);
  b[7] = t_i & 0xFF;
  canSend(CMD_MIT, b, 8);
}

void handleFrame(const twai_message_t& m) {
  if (rawOn) {
    Serial.printf("# RX %s 0x%08lX [%d]", m.extd ? "EXT" : "STD",
                  (unsigned long)m.identifier, m.data_length_code);
    for (int i = 0; i < m.data_length_code; i++) Serial.printf(" %02X", m.data[i]);
    Serial.println();
  }

  if (!m.extd) {
    static bool warned = false;
    if (!warned) {
      warned = true;
      Serial.println("# Note: received a STANDARD-ID frame. If it's from the motor, it may be "
                     "on legacy (V1/V2-style) MIT protocol. `raw on` to inspect.");
    }
    return;
  }
  uint8_t  fromId = (uint8_t)(m.identifier & 0xFF);
  uint32_t fn = m.identifier >> 8;

  // Remember every motor we hear, and point out one that isn't the selected ID
  if (fn == FN_STATUS && !seenIds[fromId]) {
    seenIds[fromId] = true;
    if (!scanning && fromId != motorId) {
      Serial.printf("# Heard a motor at ID %u (0x%02X). Type `id %u` to talk to it.\n",
                    fromId, fromId, fromId);
    }
  }
  if (scanning || fromId != motorId) return;

  if (fn == FN_SERVO_START) {
    if (rawOn) Serial.println("# (motor reports it entered servo mode)");
    return;
  }

  // Status frame: ID 0x2900 | motorId
  if (fn == FN_STATUS && m.data_length_code >= 8) {
    int16_t p = (int16_t)((m.data[0] << 8) | m.data[1]);
    int16_t s = (int16_t)((m.data[2] << 8) | m.data[3]);
    int16_t c = (int16_t)((m.data[4] << 8) | m.data[5]);
    bool first = !fb.valid;
    fb.valid   = true;
    fb.t_ms    = millis();
    fb.pos_deg = p * 0.1f;
    fb.vel_dps = erpmToDps(s * 10.0f);
    fb.cur_A   = c * 0.01f;
    fb.temp_C  = (int8_t)m.data[6];
    fb.err     = m.data[7];
    if (fb.err == ERR_DISABLE_ACK) {     // not a fault: someone sent "disable"
      fb.err = 0;
      Serial.println("# Motor acknowledged a disable command");
    }
    if (first) {
      Serial.printf("# Motor %u found: pos %.1f deg, temp %d C, err %u (%s)\n",
                    motorId, fb.pos_deg, fb.temp_C, fb.err, errName(fb.err));
    }
  }
}

// ------------------------------- modes --------------------------------------
void goLimp(const char* why) {
  mode = MODE_LIMP;
  desDeg = NAN;
  if (why) Serial.printf("# STOP (%s): motor limp\n", why);
}

bool insideLimits() {
  if (limitsArmed && fabsf(fb.pos_deg) > LIM_POS_DEG) {
    Serial.printf("# Motor at %.1f deg is outside soft limits (+/-%.0f). "
                  "Move it back by hand or `limits off`.\n", fb.pos_deg, LIM_POS_DEG);
    return false;
  }
  return true;
}

bool ready() {
  if (!feedbackFresh()) {
    Serial.println("# No recent feedback from the motor, not moving. Try `status`.");
    return false;
  }
  if (fb.err) {
    Serial.printf("# Motor reports error %u (%s). Clear it before commanding.\n",
                  fb.err, errName(fb.err));
    return false;
  }
  return insideLimits();
}

bool mitReady() {
  if (!ready()) return false;
  float pr = fb.pos_deg * DEG_TO_RAD;
  if (pr < P_MIN + 0.5f || pr > P_MAX - 0.5f) {
    Serial.println("# Position is outside the MIT range (about +/-720 deg). Run `origin` first.");
    return false;
  }
  return true;
}

bool targetOk(float deg) {
  if (limitsArmed && fabsf(deg) > LIM_POS_DEG) {
    Serial.printf("# Target %.1f deg is outside soft limits (+/-%.0f).\n", deg, LIM_POS_DEG);
    return false;
  }
  return true;
}

void controlTick() {
  uint32_t now = millis();

  // Safety checks while the motor is being driven
  if (mode != MODE_LIMP) {
    if (!feedbackFresh()) {
      goLimp("feedback timeout");
    } else if (fb.err) {
      char why[48];
      snprintf(why, sizeof(why), "motor error %u: %s", fb.err, errName(fb.err));
      goLimp(why);
    } else if (limitsArmed && fabsf(fb.pos_deg) > LIM_POS_DEG + 5.0f) {
      goLimp("soft position limit");
    }
  }

  switch (mode) {
    case MODE_LIMP:
      sendCurrent(0.0f);   // keeps the motor limp and keeps feedback coming
      break;
    case MODE_CURRENT:
      sendCurrent(curCmdA);
      break;
    case MODE_VEL:
      sendVelocity(velCmdDps);
      break;
    case MODE_POSVEL:
      sendPosVel(pvPosDeg, pvDir * pvSpdDps, pvDir * pvAccDps2);
      desDeg = pvPosDeg;
      break;
    case MODE_MIT:
      sendMit(mitP, mitV, gainKp, gainKd, mitT);
      desDeg = mitP * RAD_TO_DEG;
      break;
    case MODE_SINE: {
      float t    = (now - sineT0) / 1000.0f;
      float ramp = fminf(1.0f, t / 2.0f);          // fade amplitude in over 2 s
      float w    = TWO_PI * sineHz;
      float pDeg = sineCenterDeg + ramp * sineAmpDeg * sinf(w * t);
      float vDps = ramp * sineAmpDeg * w * cosf(w * t);
      sendMit(pDeg * DEG_TO_RAD, vDps * DEG_TO_RAD, gainKp, gainKd, 0.0f);
      desDeg = pDeg;
      break;
    }
  }

  if (logOn) {
    Serial.printf("%lu,%s,%.2f,%.2f,%.1f,%.2f,%.2f,%d,%u\n",
                  (unsigned long)now, MODE_NAMES[mode], desDeg, fb.pos_deg,
                  fb.vel_dps, fb.cur_A, fb.cur_A * KT_OUTPUT, fb.temp_C, fb.err);
  }
}

// --------------------------- bus health / hints -----------------------------
void busTick() {
  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) return;
  if (s.state == TWAI_STATE_BUS_OFF) {
    Serial.println("# CAN bus-off (wiring/termination/bit rate?). Recovering...");
    twai_initiate_recovery();
  } else if (s.state == TWAI_STATE_STOPPED) {   // recovery finished
    twai_start();
    Serial.println("# CAN restarted");
  }
}

void hints() {
  static uint32_t lastHint = 0;
  static bool lostWarned = false;
  uint32_t now = millis();
  if (!fb.valid) {
    if (now - bootMs > 1500 && now - lastHint > 4000) {
      lastHint = now;
      Serial.printf("# No reply from motor id %u yet. Type `status` for a diagnosis, "
                    "or `scan` to search for the motor's ID.\n", motorId);
    }
  } else if (!feedbackFresh()) {
    if (!lostWarned) {
      lostWarned = true;
      Serial.println("# Lost motor feedback.");
    }
  } else {
    lostWarned = false;
  }
}

void printDiagnosis(const twai_status_info_t& s);

void printStatus() {
  static const char* ST[] = { "STOPPED", "RUNNING", "BUS_OFF", "RECOVERING" };
  twai_status_info_t s = {};
  twai_get_status_info(&s);
  Serial.printf("# motor id %u | mode %s | limits %s (+/-%.0f deg) | gains kp %.1f kd %.2f\n",
                motorId, MODE_NAMES[mode], limitsArmed ? "ARMED" : "off", LIM_POS_DEG,
                gainKp, gainKd);
  if (fb.valid) {
    Serial.printf("# fb: pos %.2f deg  vel %.1f deg/s  cur %.2f A (~%.2f Nm)  temp %d C  "
                  "err %u (%s)  age %lu ms\n",
                  fb.pos_deg, fb.vel_dps, fb.cur_A, fb.cur_A * KT_OUTPUT, fb.temp_C,
                  fb.err, errName(fb.err), (unsigned long)(millis() - fb.t_ms));
  } else {
    Serial.println("# fb: none received yet");
  }
  Serial.printf("# CAN: %s  txErr %lu  rxErr %lu  txFailed %lu  busErr %lu  "
                "rxMissed %lu  txDropped %lu\n",
                s.state <= 3 ? ST[s.state] : "?", (unsigned long)s.tx_error_counter,
                (unsigned long)s.rx_error_counter, (unsigned long)s.tx_failed_count,
                (unsigned long)s.bus_error_count, (unsigned long)s.rx_missed_count,
                (unsigned long)txDrops);
  if (!feedbackFresh()) printDiagnosis(s);
}

// Plain-English read of the bus state when the selected motor isn't answering
void printDiagnosis(const twai_status_info_t& s) {
  if (s.state == TWAI_STATE_BUS_OFF || s.state == TWAI_STATE_RECOVERING) {
    Serial.println("# Diagnosis: the ESP32 can't hear its own CAN messages. The CAN board isn't "
                   "on CAN_TX_PIN/CAN_RX_PIN, CTX/CRX are swapped, or it has no 3.3 V.");
  } else if (s.tx_error_counter >= 96) {
    Serial.println("# Diagnosis: nobody is acknowledging our messages, so the motor isn't on "
                   "the bus. Check 48 V (blue light on), CANH->CAN_H (white), CANL->CAN_L "
                   "(blue), loose screw terminals, and termination.");
  } else {
    bool any = false;
    for (int i = 0; i < 256; i++) any = any || seenIds[i];
    if (any) {
      Serial.print("# Diagnosis: the bus works, and the motor(s) answering are at ID");
      for (int i = 0; i < 256; i++) if (seenIds[i]) Serial.printf(" %d", i);
      Serial.println(". Use `id <n>`.");
    } else {
      Serial.printf("# Diagnosis: messages are acknowledged (something is on the bus) but "
                    "nothing answers ID %u. Run `scan` to find the motor's ID.\n", motorId);
    }
  }
}

void drainRx() {
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);
}

// Send a zero-current frame to every ID and see who answers. The motor won't move.
void scanIds() {
  if (mode != MODE_LIMP) { Serial.println("# stop the motor first"); return; }
  Serial.println("# Scanning IDs 1-254 with zero-current frames (motor will not move)...");
  uint8_t saved = motorId;
  memset(seenIds, 0, sizeof(seenIds));
  scanning = true;
  for (int id = 1; id <= 254; id++) {
    motorId = (uint8_t)id;
    sendCurrent(0.0f);
    for (int k = 0; k < 5; k++) { drainRx(); delay(1); }
  }
  for (int k = 0; k < 50; k++) { drainRx(); delay(1); }   // late replies
  scanning = false;
  motorId = saved;

  int count = 0, lastFound = -1;
  for (int i = 0; i < 256; i++) if (seenIds[i]) { count++; lastFound = i; }
  if (count == 0) {
    Serial.println("# Scan found no motors.");
    twai_status_info_t s = {};
    twai_get_status_info(&s);
    printDiagnosis(s);
  } else if (count == 1) {
    motorId = (uint8_t)lastFound;
    fb = Feedback();
    limitsArmed = false;
    bootMs = millis();
    Serial.printf("# Found one motor at ID %d (0x%02X). Now talking to it.\n",
                  lastFound, lastFound);
    Serial.printf("# Tip: set MOTOR_ID_DEFAULT to %d so it's used at startup.\n", lastFound);
  } else {
    Serial.print("# Found motors at IDs:");
    for (int i = 0; i < 256; i++) if (seenIds[i]) Serial.printf(" %d", i);
    Serial.println(". Pick one with `id <n>`.");
  }
}

void printHelp() {
  Serial.println(
    "# ---------------------------- commands ----------------------------\n"
    "#  <Enter> | x | stop     motor limp (0 A)\n"
    "#  status                 feedback + CAN bus health (+ diagnosis)\n"
    "#  scan                   find the motor's CAN ID (motor won't move)\n"
    "#  id <n>                 talk to motor CAN ID n (stop first)\n"
    "#  origin [perm]          current position = 0 (perm = saved). Arms limits\n"
    "#  limits on|off          soft position limits (+/-100 deg from origin)\n"
    "#  cur <A>                current (torque) control, 1 A ~ 1.06 Nm, max 5 A\n"
    "#  vel <deg/s>            velocity control, max 360 deg/s\n"
    "#  pv <deg> [deg/s] [deg/s^2]  go to position, speed/accel limited (60, 180)\n"
    "#  hold                   MIT spring-damper at the current position\n"
    "#  gains <kp> <kd>        MIT gains, Nm/rad and Nm*s/rad (default 8, 0.5)\n"
    "#  mit <p_rad> <v_rad/s> <kp> <kd> <tau_Nm>   raw MIT command\n"
    "#  sine <amp_deg> <hz>    MIT sine about the current position\n"
    "#  log on|off             CSV stream (see header line)\n"
    "#  raw on|off             print every received CAN frame\n"
    "#  help                   this list\n"
    "# -----------------------------------------------------------------");
}

// ------------------------------ commands ------------------------------------
void handleLine(char* s) {
  for (char* p = s; *p; ++p) *p = (char)tolower((unsigned char)*p);

  char cmd[16] = "", w1[16] = "";
  int nw = sscanf(s, "%15s %15s", cmd, w1);
  if (nw <= 0) { goLimp("Enter"); return; }

  float a[5] = { NAN, NAN, NAN, NAN, NAN };
  int na = sscanf(s, "%*s %f %f %f %f %f", &a[0], &a[1], &a[2], &a[3], &a[4]);
  if (na < 0) na = 0;
  bool on = !strcmp(w1, "on") || !strcmp(w1, "1");

  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
    printHelp();

  } else if (!strcmp(cmd, "x") || !strcmp(cmd, "stop")) {
    goLimp("user");

  } else if (!strcmp(cmd, "status")) {
    printStatus();

  } else if (!strcmp(cmd, "scan")) {
    scanIds();

  } else if (!strcmp(cmd, "log")) {
    logOn = on;
    Serial.println(on ? LOG_HEADER : "# log off");

  } else if (!strcmp(cmd, "raw")) {
    rawOn = on;
    Serial.printf("# raw %s\n", on ? "on" : "off");

  } else if (!strcmp(cmd, "limits")) {
    limitsArmed = on;
    Serial.printf("# soft limits %s\n", on ? "ARMED" : "off");

  } else if (!strcmp(cmd, "id")) {
    if (na < 1 || a[0] < 0 || a[0] > 255) { Serial.println("# usage: id <0-255>"); return; }
    if (mode != MODE_LIMP) { Serial.println("# stop the motor first"); return; }
    motorId = (uint8_t)a[0];
    fb = Feedback();
    limitsArmed = false;
    bootMs = millis();
    Serial.printf("# now talking to motor id %u (limits disarmed)\n", motorId);

  } else if (!strcmp(cmd, "origin")) {
    if (mode != MODE_LIMP) { Serial.println("# stop the motor first"); return; }
    bool perm = !strcmp(w1, "perm");
    sendOrigin(perm);
    limitsArmed = true;
    Serial.printf("# origin set (%s). Soft limits ARMED at +/-%.0f deg\n",
                  perm ? "permanent" : "temporary", LIM_POS_DEG);

  } else if (!strcmp(cmd, "cur")) {
    if (na < 1) { Serial.println("# usage: cur <amps>"); return; }
    if (!ready()) return;
    curCmdA = clampWarn(a[0], -LIM_CURRENT_A, LIM_CURRENT_A, "current");
    desDeg = NAN;
    mode = MODE_CURRENT;
    Serial.printf("# current %.2f A (~%.1f Nm)\n", curCmdA, curCmdA * KT_OUTPUT);

  } else if (!strcmp(cmd, "vel")) {
    if (na < 1) { Serial.println("# usage: vel <deg/s>"); return; }
    if (!ready()) return;
    velCmdDps = clampWarn(a[0], -LIM_SPEED_DPS, LIM_SPEED_DPS, "speed");
    desDeg = NAN;
    mode = MODE_VEL;
    Serial.printf("# velocity %.1f deg/s (%.0f ERPM)\n", velCmdDps, dpsToErpm(velCmdDps));

  } else if (!strcmp(cmd, "pv")) {
    if (na < 1) { Serial.println("# usage: pv <deg> [deg/s] [deg/s^2]"); return; }
    if (!ready() || !targetOk(a[0])) return;
    pvPosDeg  = a[0];
    pvDir     = (pvPosDeg >= fb.pos_deg) ? 1.0f : -1.0f;
    pvSpdDps  =(na >= 2) ? clampWarn(fabsf(a[1]), 1, LIM_SPEED_DPS, "speed") : 60.0f;
    pvAccDps2 = (na >= 3) ? clampWarn(fabsf(a[2]), 1, LIM_ACCEL_DPS2, "accel") : 180.0f;
    mode = MODE_POSVEL;
    Serial.printf("# moving to %.1f deg at %.0f deg/s, %.0f deg/s^2\n",
                  pvPosDeg, pvSpdDps, pvAccDps2);

  } else if (!strcmp(cmd, "hold")) {
    if (!mitReady()) return;
    mitP = fb.pos_deg * DEG_TO_RAD;
    mitV = 0;
    mitT = 0;
    mode = MODE_MIT;
    Serial.printf("# MIT hold at %.1f deg (kp %.1f, kd %.2f)\n", fb.pos_deg, gainKp, gainKd);

  } else if (!strcmp(cmd, "gains")) {
    if (na < 2) { Serial.println("# usage: gains <kp> <kd>"); return; }
    gainKp = clampWarn(a[0], 0, LIM_KP, "kp");
    gainKd = clampWarn(a[1], 0, LIM_KD, "kd");
    Serial.printf("# gains kp %.2f kd %.2f (applies live in hold/mit/sine)\n", gainKp, gainKd);

  } else if (!strcmp(cmd, "mit")) {
    if (na < 5) { Serial.println("# usage: mit <p_rad> <v_rad/s> <kp> <kd> <tau_Nm>"); return; }
    if (!mitReady()) return;
    float pDeg = a[0] * RAD_TO_DEG;
    if (!targetOk(pDeg)) return;
    float kp = clampWarn(a[2], 0, LIM_KP, "kp");
    if (kp > 0 && fabsf(pDeg - fb.pos_deg) > LIM_MIT_JUMP_DEG) {
      Serial.printf("# Target is %.0f deg from the current position (max %.0f). "
                    "Use `pv` to get closer first.\n",
                    fabsf(pDeg - fb.pos_deg), LIM_MIT_JUMP_DEG);
      return;
    }
    float vmax = LIM_SPEED_DPS * DEG_TO_RAD;
    mitP   = a[0];
    mitV   = clampWarn(a[1], -vmax, vmax, "velocity");
    gainKp = kp;
    gainKd = clampWarn(a[3], 0, LIM_KD, "kd");
    mitT   = clampWarn(a[4], -LIM_TORQUE_NM, LIM_TORQUE_NM, "torque");
    mode = MODE_MIT;
    Serial.printf("# MIT p %.3f rad, v %.2f rad/s, kp %.1f, kd %.2f, tau %.2f Nm\n",
                  mitP, mitV, gainKp, gainKd, mitT);

  } else if (!strcmp(cmd, "sine")) {
    if (na < 2) { Serial.println("# usage: sine <amp_deg> <hz>"); return; }
    if (!mitReady()) return;
    float amp = clampWarn(fabsf(a[0]), 0, LIM_SINE_AMP_DEG, "amplitude");
    float hz  = clampWarn(fabsf(a[1]), 0.05f, LIM_SINE_HZ, "frequency");
    float peak = amp * TWO_PI * hz;
    if (peak > LIM_SPEED_DPS) {
      Serial.printf("# Peak speed %.0f deg/s exceeds limit %.0f. Lower amp or hz.\n",
                    peak, LIM_SPEED_DPS);
      return;
    }
    float c = fb.pos_deg;
    if (!targetOk(c + amp) || !targetOk(c - amp)) return;
    sineAmpDeg = amp; sineHz = hz; sineCenterDeg = c; sineT0 = millis();
    mode = MODE_SINE;
    Serial.printf("# sine %.1f deg @ %.2f Hz about %.1f deg (kp %.1f, kd %.2f)\n",
                  amp, hz, c, gainKp, gainKd);

  } else {
    Serial.printf("# unknown command '%s'. Type help\n", cmd);
  }
}

void pollSerial() {
  static char buf[96];
  static size_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = '\0';
      handleLine(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// ------------------------------ main ----------------------------------------
void setup() {
  Serial.setTxBufferSize(2048);   // so logging doesn't stall the loop
  Serial.begin(SERIAL_BAUD);
  delay(300);
  Serial.println("\n# ===== AK70-9 V3 motor test console =====");

  if (!canBegin()) {
    Serial.println("# CAN init FAILED. Check CAN_TX_PIN / CAN_RX_PIN.");
    while (true) delay(1000);
  }
  Serial.printf("# CAN up: 1 Mbps, TX=GPIO%d, RX=GPIO%d, motor id %u, %d Hz loop\n",
                CAN_TX_PIN, CAN_RX_PIN, motorId, CONTROL_HZ);
  printHelp();
  bootMs = millis();
  lastTickUs = micros();
}

void loop() {
  drainRx();
  pollSerial();

  uint32_t nowUs = micros();
  if (nowUs - lastTickUs >= TICK_US) {
    lastTickUs += TICK_US;
    if (nowUs - lastTickUs > 5 * TICK_US) lastTickUs = nowUs;   // don't try to catch up
    controlTick();
  }

  static uint32_t lastSlow = 0;
  if (millis() - lastSlow >= 100) {
    lastSlow = millis();
    busTick();
    hints();
  }
}
