/*
 * ============================================================================
 *  safety.h  -  RUN/STOP STATE, COMMAND LIMITS AND FAULT CHECKS   (don't edit)
 * ============================================================================
 *
 *  The template is always in one of two states:
 *
 *    LIMP  every motor gets 0 A and turns freely. The sketch boots here,
 *          and returns here on `stop`, an empty line (Enter), or any fault.
 *    RUN   your controllerStep() runs every tick. Entered only by `run`.
 *
 *  Every tick in RUN, in this order:
 *    1. checkFaults()    looks at the feedback. Any problem -> LIMP.
 *    2. controllerStep() (your code) sets each joint's command.
 *    3. limitCommands()  clamps those commands to config.h's limits.
 *    4. the commands are sent.
 *  Because step 3 comes after your code, nothing in my_controller.h can get
 *  around the limits. Change the limits in config.h instead.
 *
 *  A fault "latches": the motors stay limp until someone types `run` again,
 *  which re-checks everything first.
 *
 *  What this does NOT replace: a switch on the motor power supply within
 *  reach (the real e-stop), mechanical joint stops, and a spotter whenever
 *  the exo is on a person.
 * ============================================================================
 */
#pragma once
#include <Arduino.h>
#include "config.h"
#include "joints.h"

enum RunState : uint8_t { STATE_LIMP, STATE_RUN };
const char* const STATE_NAME[] = { "limp", "run" };

RunState runState = STATE_LIMP;
char lastStopReason[96] = "boot";

// ---- per-run bookkeeping ----
uint32_t overCurrentSince[MAX_MOTORS] = {};  // millis() when |current| first exceeded the trip level (0 = not over)
uint16_t clampWarned[MAX_MOTORS] = {};       // which "command was limited" notes were printed this run

// Bits for clampWarned
enum : uint16_t {
  W_TORQUE = 1 << 0, W_SPEED = 1 << 1, W_ACCEL = 1 << 2, W_KP = 1 << 3, W_KD = 1 << 4,
  W_POS_LIMIT = 1 << 5, W_IMP_TORQUE = 1 << 6, W_AT_LIMIT = 1 << 7
};

// Print a note the first time a given limit is hit during this run,
// instead of 500 times a second.
void noteOnce(int i, uint16_t bit, const char* what) {
  if (clampWarned[i] & bit) return;
  clampWarned[i] |= bit;
  Serial.printf("# Motor %d: %s (limit in config.h). Noted once per run.\n", i + 1, what);
}

// Go limp: every motor to 0 A, state LIMP, and say why.
void goLimp(const char* why) {
  bool wasRunning = (runState == STATE_RUN);
  runState = STATE_LIMP;
  for (int i = 0; i < NUM_MOTORS; i++) joint[i].setLimp();
  snprintf(lastStopReason, sizeof(lastStopReason), "%s", why ? why : "");
  if (wasRunning) Serial.printf("# STOP (%s): motors limp. Type `run` to start again.\n", why);
  else if (why)   Serial.printf("# Motors limp (%s)\n", why);
}

// Called by `run`. Clears the per-run bookkeeping.
void resetSafetyForRun() {
  for (int i = 0; i < MAX_MOTORS; i++) {
    overCurrentSince[i] = 0;
    clampWarned[i] = 0;
  }
}

static inline bool outsideLimits(float deg, float margin) {
  return deg < JOINT_MIN_DEG - margin || deg > JOINT_MAX_DEG + margin;
}

// Everything that has to be true before `run` is allowed.
// Prints the first problem it finds and returns false.
bool readyToRun() {
  if (REQUIRE_ZERO_BEFORE_RUN && !zeroed) {
    Serial.println("# Not running: type `zero` first (with the joints at their neutral pose), "
                   "so the soft position limits are active.");
    return false;
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    const Joint& j = joint[i];
    if (!j.ok) {
      Serial.printf("# Not running: no recent feedback from motor %d (%s, CAN ID %u). "
                    "Type `status`.\n", i + 1, j.name, j.canId);
      return false;
    }
    if (j.errorCode) {
      Serial.printf("# Not running: motor %d reports error %u (%s). Power-cycle it to clear.\n",
                    i + 1, j.errorCode, ak70::errorName(j.errorCode));
      return false;
    }
    if (j.tempC >= TRIP_TEMP_C) {
      Serial.printf("# Not running: motor %d is at %d C (limit %d C). Let it cool.\n",
                    i + 1, j.tempC, TRIP_TEMP_C);
      return false;
    }
    if (zeroed && outsideLimits(j.pos.deg, 0)) {
      Serial.printf("# Not running: motor %d is at %.1f deg, outside the soft limits "
                    "[%.0f, %.0f]. Move it back by hand.\n",
                    i + 1, j.pos.deg, JOINT_MIN_DEG, JOINT_MAX_DEG);
      return false;
    }
  }
  return true;
}

// Every tick while running, BEFORE the controller.
// Returns true and fills `why` if something is wrong.
bool checkFaults(char* why, size_t n) {
  uint32_t now = millis();
  for (int i = 0; i < NUM_MOTORS; i++) {
    const Joint& j = joint[i];
    if (!j.ok) {
      snprintf(why, n, "no feedback from motor %d for %lu ms", i + 1,
               (unsigned long)(now - j.fb.t_ms));
      return true;
    }
    if (j.errorCode) {
      snprintf(why, n, "motor %d error %u: %s", i + 1, j.errorCode,
               ak70::errorName(j.errorCode));
      return true;
    }
    if (j.tempC >= TRIP_TEMP_C) {
      snprintf(why, n, "motor %d temperature %d C", i + 1, j.tempC);
      return true;
    }
    if (fabsf(j.vel.deg) > TRIP_SPEED_DPS) {
      snprintf(why, n, "motor %d overspeed %.0f deg/s", i + 1, j.vel.deg);
      return true;
    }
    if (zeroed && outsideLimits(j.pos.deg, TRIP_POS_MARGIN_DEG)) {
      snprintf(why, n, "motor %d at %.1f deg, past the soft limit", i + 1, j.pos.deg);
      return true;
    }
    if (fabsf(j.currentA) > TRIP_CURRENT_A) {
      if (overCurrentSince[i] == 0) overCurrentSince[i] = now | 1;   // |1: never store 0
      else if (now - overCurrentSince[i] > TRIP_CURRENT_MS) {
        snprintf(why, n, "motor %d current %.1f A for over %lu ms", i + 1, j.currentA,
                 (unsigned long)TRIP_CURRENT_MS);
        return true;
      }
    } else {
      overCurrentSince[i] = 0;
    }
  }
  return false;
}

static inline bool badNumber(float x) { return isnan(x) || isinf(x); }

// Every tick while running, AFTER the controller: clamp every command to the
// limits in config.h. Returns false (and goes limp) if a command is unusable.
bool limitCommands() {
  const float limTorque = LIM_TORQUE_NM;
  const float limSpeed  = LIM_SPEED_DPS * DEG_TO_RAD;    // rad/s
  const float limAccel  = LIM_ACCEL_DPS2 * DEG_TO_RAD;   // rad/s^2
  const float posMin    = JOINT_MIN_DEG * DEG_TO_RAD;    // rad
  const float posMax    = JOINT_MAX_DEG * DEG_TO_RAD;

  for (int i = 0; i < NUM_MOTORS; i++) {
    Joint& j = joint[i];
    JointCommand& c = j.cmd;

    // A NaN or infinity means a bug in the controller (e.g. divide by zero).
    if (badNumber(c.torqueNm) || badNumber(c.posRad) || badNumber(c.velRad) ||
        badNumber(c.kp) || badNumber(c.kd) || badNumber(c.maxVelRad) || badNumber(c.maxAccRad)) {
      char why[64];
      snprintf(why, sizeof(why), "controller gave motor %d a NaN/inf command", i + 1);
      goLimp(why);
      return false;
    }

    // At (or past) a soft limit, nothing may push further out.
    bool atMax = zeroed && j.pos.deg >= JOINT_MAX_DEG;
    bool atMin = zeroed && j.pos.deg <= JOINT_MIN_DEG;

    switch (c.type) {
      case CMD_LIMP:
        break;

      case CMD_TORQUE: {
        float t = constrain(c.torqueNm, -limTorque, limTorque);
        if (t != c.torqueNm) noteOnce(i, W_TORQUE, "torque command limited");
        if ((atMax && t > 0) || (atMin && t < 0)) {
          t = 0;
          noteOnce(i, W_AT_LIMIT, "at a soft limit, outward torque removed");
        }
        c.torqueNm = t;
        break;
      }

      case CMD_VELOCITY: {
        float v = constrain(c.velRad, -limSpeed, limSpeed);
        if (v != c.velRad) noteOnce(i, W_SPEED, "speed command limited");
        if ((atMax && v > 0) || (atMin && v < 0)) {
          v = 0;
          noteOnce(i, W_AT_LIMIT, "at a soft limit, outward speed set to 0");
        }
        c.velRad = v;
        break;
      }

      case CMD_POSITION: {
        if (zeroed) {
          float p = constrain(c.posRad, posMin, posMax);
          if (p != c.posRad) noteOnce(i, W_POS_LIMIT, "position target moved inside the soft limits");
          c.posRad = p;
        }
        float v = constrain(fabsf(c.maxVelRad), 1.0f * DEG_TO_RAD, limSpeed);
        if (v != fabsf(c.maxVelRad)) noteOnce(i, W_SPEED, "position-move speed limited");
        float a = constrain(fabsf(c.maxAccRad), 1.0f * DEG_TO_RAD, limAccel);
        if (a != fabsf(c.maxAccRad)) noteOnce(i, W_ACCEL, "position-move acceleration limited");
        c.maxVelRad = v;
        c.maxAccRad = a;
        break;
      }

      case CMD_IMPEDANCE: {
        // Gains and individual terms
        float kp = constrain(c.kp, 0.0f, LIM_KP_NM_PER_RAD);
        if (kp != c.kp) noteOnce(i, W_KP, "impedance kp limited");
        float kd = constrain(c.kd, 0.0f, LIM_KD_NM_S_PER_RAD);
        if (kd != c.kd) noteOnce(i, W_KD, "impedance kd limited");
        float p = c.posRad;
        if (zeroed) {
          p = constrain(p, posMin, posMax);
          if (p != c.posRad) noteOnce(i, W_POS_LIMIT, "impedance target moved inside the soft limits");
        }
        float v = constrain(c.velRad, -limSpeed, limSpeed);
        if (v != c.velRad) noteOnce(i, W_SPEED, "impedance speed limited");

        // The motor computes kp*(p - pos) + kd*(v - vel) + tauFF itself, so
        // clamping tauFF alone isn't enough: a far-away target with a big kp
        // could ask for 30+ Nm. Predict the torque from the latest feedback
        // and, if the spring+damper part is too big, pull the target toward
        // where the joint actually is. The torque keeps its direction but is
        // capped at the limit.
        float ep = p - j.pos.rad;
        float ev = v - j.vel.rad;
        float sd = kp * ep + kd * ev;
        if (fabsf(sd) > limTorque) {
          float s = limTorque / fabsf(sd);
          ep *= s;
          ev *= s;
          sd *= s;
          noteOnce(i, W_IMP_TORQUE, "impedance torque limited (target pulled closer)");
        }
        // Feed-forward torque gets whatever room is left under the limit.
        float tff = constrain(c.torqueNm, -limTorque - sd, limTorque - sd);
        if ((atMax && tff > 0) || (atMin && tff < 0)) tff = 0;
        if (tff != c.torqueNm) noteOnce(i, W_TORQUE, "impedance feed-forward torque limited");

        c.kp = kp;
        c.kd = kd;
        c.posRad = j.pos.rad + ep;
        c.velRad = j.vel.rad + ev;
        c.torqueNm = tff;
        break;
      }
    }
  }
  return true;
}
