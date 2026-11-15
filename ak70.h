/*
 * ============================================================================
 *  ak70.h  -  CubeMars AK70-9 V3 CAN DRIVER            (you shouldn't need to edit)
 * ============================================================================
 *
 *  Talks to AK-series V3 motors over the ESP32's built-in CAN controller
 *  (called "TWAI" in ESP32 docs). No extra libraries needed.
 *
 *  The frame packing and feedback decoding below are copied from
 *  ak70_motor_test.ino, which has been run on the real motor. The only change
 *  is that every function now takes the motor's CAN ID, so the same code
 *  drives 1 or 2 motors.
 *
 *  PROTOCOL SUMMARY (CubeMars "AK Series Product Manual" v3.2.0, project files)
 *   - CAN at 1 Mbit/s. Every frame uses a 29-bit EXTENDED ID.
 *   - Command ID  = (mode << 8) | motor_id. Modes used here:
 *        1 = current (torque)     3 = velocity (ERPM)     5 = set origin
 *        6 = position-velocity    8 = MIT (impedance: p, v, kp, kd, torque)
 *     V3 motors switch between these automatically from the ID. No
 *     "enter motor mode" frame is needed.
 *   - Feedback ID = 0x2900 | motor_id, 8 data bytes:
 *        position int16 (0.1 deg), speed int16 (10 ERPM),
 *        current int16 (0.01 A), temperature int8 (C), error code uint8.
 *     The motor sends it periodically (1-2000 Hz, set in CubeMars Tool) or
 *     as a reply to each command ("query-reply"). Both work here.
 *
 *  DO NOT add the old "FF FF FF FF FF FF FF FC/FD/FE" enter/exit/zero frames
 *  from V1/V2 examples. The v3.2.0 manual doesn't define them, and a V3 motor
 *  would read them as an MIT command with full stiffness, full speed, and
 *  about +32 Nm.
 *
 *  All positions/speeds here are in the MOTOR's frame (its own sign and
 *  origin, at the output shaft). joints.h converts to the joint frame.
 * ============================================================================
 */
#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "config.h"

namespace ak70 {

// ---- MIT (impedance) ranges for the AK70-9, manual v3.2.0 table ----
// These must match the motor firmware or MIT commands get scaled wrong.
// (Older code in this project used +/-60 rad/s for speed; that's the AK60-6.)
const float P_MIN  = -12.56f, P_MAX  = 12.56f;   // rad
const float V_MIN  = -30.0f,  V_MAX  = 30.0f;    // rad/s
const float T_MIN  = -32.0f,  T_MAX  = 32.0f;    // Nm
const float KP_MIN = 0.0f,    KP_MAX = 500.0f;   // Nm/rad
const float KD_MIN = 0.0f,    KD_MAX = 5.0f;     // Nm/(rad/s)

// ---- CAN command numbers (upper bits of the ID) ----
enum Command : uint32_t {
  CMD_CURRENT = 1,
  CMD_RPM     = 3,
  CMD_ORIGIN  = 5,
  CMD_POS_SPD = 6,
  CMD_MIT     = 8
};

// ---- Frames the motor sends to us ----
const uint32_t FN_STATUS       = 0x29;  // real-time feedback
const uint32_t FN_SERVO_START  = 0x2C;  // "entered servo mode" (data FA FB FC FD)
const uint8_t  ERR_DISABLE_ACK = 0x77;  // error byte after a "disable" frame; not a fault

// Raw feedback from one motor, in the motor's own frame.
struct Feedback {
  bool     valid   = false;  // true once at least one frame has arrived
  uint32_t t_ms    = 0;      // millis() when the last frame arrived
  float    pos_deg = 0;      // output shaft angle from the motor's origin
  float    vel_dps = 0;      // output shaft speed
  float    cur_A   = 0;      // phase current (sign = motor's direction)
  int8_t   temp_C  = 0;      // driver board temperature
  uint8_t  err     = 0;      // 0 = OK, see errorName()
};

uint32_t txDropped = 0;      // frames we couldn't queue (bus busy or no motor ACKing)

// ---------------------------------------------------------------------------
//  Small helpers
// ---------------------------------------------------------------------------
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// Output-shaft deg/s  <->  motor electrical RPM (ERPM).
// ERPM = output RPM x gear ratio x pole pairs, and 1 RPM = 6 deg/s.
inline float dpsToErpm(float dps)  { return dps / 6.0f * GEAR_RATIO * POLE_PAIRS; }
inline float erpmToDps(float erpm) { return erpm * 6.0f / (GEAR_RATIO * POLE_PAIRS); }

// Big-endian packing, as the manual's buffer_append_int32/int16.
inline void putI32(uint8_t* b, int32_t v) {
  b[0] = (uint8_t)(v >> 24); b[1] = (uint8_t)(v >> 16);
  b[2] = (uint8_t)(v >> 8);  b[3] = (uint8_t)v;
}
inline void putI16(uint8_t* b, int16_t v) {
  b[0] = (uint8_t)(v >> 8); b[1] = (uint8_t)v;
}

// Human-readable name for the motor's error code.
const char* errorName(uint8_t e) {
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

// ---------------------------------------------------------------------------
//  Bus setup and sending
// ---------------------------------------------------------------------------

// Start the CAN controller at 1 Mbit/s, accepting every frame.
bool begin() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g.tx_queue_len = 10;   // room for a few ticks' worth of commands
  g.rx_queue_len = 64;   // 2 motors x 500 Hz feedback, drained every loop
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

// Queue one frame. Never waits: if the queue is full the frame is dropped
// and counted, so a CAN problem can never freeze the control loop.
void send(uint8_t motorId, uint32_t cmd, const uint8_t* data, uint8_t len) {
  twai_message_t m = {};
  m.extd = 1;
  m.identifier = (cmd << 8) | motorId;
  m.data_length_code = len;
  memcpy(m.data, data, len);
  if (twai_transmit(&m, 0) != ESP_OK) txDropped++;
}

// Current loop (= torque). amps: + and - are the motor's own directions.
void sendCurrent(uint8_t id, float amps) {
  uint8_t b[4];
  putI32(b, (int32_t)lroundf(amps * 1000.0f));   // 1 unit = 1 mA
  send(id, CMD_CURRENT, b, 4);
}

// Velocity loop. dps = output shaft deg/s, sent as ERPM.
void sendVelocity(uint8_t id, float dps) {
  uint8_t b[4];
  putI32(b, (int32_t)lroundf(dpsToErpm(dps)));
  send(id, CMD_RPM, b, 4);
}

// Position-velocity loop: move to `deg` with speed and acceleration limits.
// Speed and accel carry the sign of the direction of travel, as in the
// manual's examples (joints.h works the sign out).
void sendPosVel(uint8_t id, float deg, float dps, float dps2) {
  uint8_t b[8];
  putI32(b, (int32_t)lroundf(deg * 10000.0f));                       // 0.0001 deg
  putI16(b + 4, (int16_t)clampf(roundf(dpsToErpm(dps) / 10.0f), -32768, 32767));   // 10 ERPM
  putI16(b + 6, (int16_t)clampf(roundf(dpsToErpm(dps2) / 10.0f), -32768, 32767));  // 10 ERPM/s
  send(id, CMD_POS_SPD, b, 8);
}

// Set origin: the motor's current position becomes 0.
// permanent = false: forgotten at power-off (what `zero` uses).
void sendOrigin(uint8_t id, bool permanent) {
  uint8_t b[1] = { (uint8_t)(permanent ? 1 : 0) };
  send(id, CMD_ORIGIN, b, 1);
}

// Scale a float into an unsigned integer of `bits` bits over [lo, hi].
uint16_t floatToUint(float x, float lo, float hi, int bits) {
  x = clampf(x, lo, hi);
  return (uint16_t)((x - lo) * (float)((1 << bits) - 1) / (hi - lo));
}

// MIT / impedance: the motor itself applies
//   torque = kp*(p - pos) + kd*(v - vel) + t
// p rad, v rad/s, kp Nm/rad, kd Nm/(rad/s), t Nm.
void sendMit(uint8_t id, float p, float v, float kp, float kd, float t) {
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
  send(id, CMD_MIT, b, 8);
}

// ---------------------------------------------------------------------------
//  Receiving
// ---------------------------------------------------------------------------

// If `m` is a status frame (0x29), fill `out`, set `fromId`, return true.
bool decodeStatus(const twai_message_t& m, uint8_t& fromId, Feedback& out) {
  if (!m.extd || (m.identifier >> 8) != FN_STATUS || m.data_length_code < 8) return false;
  fromId = (uint8_t)(m.identifier & 0xFF);
  int16_t p = (int16_t)((m.data[0] << 8) | m.data[1]);
  int16_t s = (int16_t)((m.data[2] << 8) | m.data[3]);
  int16_t c = (int16_t)((m.data[4] << 8) | m.data[5]);
  out.valid   = true;
  out.t_ms    = millis();
  out.pos_deg = p * 0.1f;
  out.vel_dps = erpmToDps(s * 10.0f);
  out.cur_A   = c * 0.01f;
  out.temp_C  = (int8_t)m.data[6];
  out.err     = m.data[7];
  if (out.err == ERR_DISABLE_ACK) out.err = 0;   // not a fault
  return true;
}

// Restart the CAN controller if errors shut it down ("bus-off"), which
// happens with bad wiring, missing termination, or a motor unplugged mid-run.
// Call a few times per second.
void busTick() {
  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) return;
  if (s.state == TWAI_STATE_BUS_OFF) {
    Serial.println("# CAN bus-off (wiring/termination?). Recovering...");
    twai_initiate_recovery();
  } else if (s.state == TWAI_STATE_STOPPED) {   // recovery finished
    twai_start();
    Serial.println("# CAN restarted");
  }
}

}  // namespace ak70
