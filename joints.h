/*
 * ============================================================================
 *  joints.h  -  WHAT YOUR CONTROLLER SEES AND COMMANDS   (read it, don't edit it)
 * ============================================================================
 *
 *  Each motor is a "Joint". Every tick, before your controllerStep() runs,
 *  the template fills in each joint's feedback in the JOINT FRAME (angle
 *  from `zero`, direction from MOTOR_DIR in config.h), in both units:
 *
 *      joint[i].ok          true = fresh feedback from this motor
 *      joint[i].pos.deg     angle, degrees          joint[i].pos.rad   radians
 *      joint[i].vel.deg     speed, deg/s            joint[i].vel.rad   rad/s
 *      joint[i].torqueNm    measured torque (current x 1.0621 Nm/A)
 *      joint[i].currentA    measured current, A
 *      joint[i].tempC       driver board temperature, C
 *      joint[i].errorCode   0 = OK
 *
 *  Your controller then sets ONE command per joint per tick. Pick whichever
 *  units you like; each function exists in a Deg and a Rad version:
 *
 *    joint[i].setLimp();                  0 A: motor turns freely
 *    joint[i].setTorque(Nm);              torque at the output shaft
 *    joint[i].setVelocityDeg(deg/s);      motor's own speed loop
 *    joint[i].setVelocityRad(rad/s);
 *    joint[i].setPositionDeg(deg, maxSpeed_deg/s, maxAccel_deg/s^2);
 *    joint[i].setPositionRad(rad, maxSpeed_rad/s, maxAccel_rad/s^2);
 *        motor's own position loop; speed/accel optional (default 60, 180 deg)
 *    joint[i].setImpedanceDeg(pos_deg, vel_deg/s, kp_Nm/deg, kd_Nm/(deg/s), tauFF_Nm);
 *    joint[i].setImpedanceRad(pos_rad, vel_rad/s, kp_Nm/rad, kd_Nm/(rad/s), tauFF_Nm);
 *        virtual spring + damper computed INSIDE the motor (MIT mode):
 *        torque = kp*(pos - actual) + kd*(vel - actual speed) + tauFF
 *
 *  Any joint you don't give a command during a tick goes limp for that tick,
 *  so forgetting a joint is safe. All commands then pass through safety.h,
 *  which clamps them to the limits in config.h before they're sent.
 *
 *  Note on impedance gains in degrees: 1 Nm/deg = 57.3 Nm/rad. A "soft"
 *  spring of 5 Nm/rad is 0.087 Nm/deg.
 * ============================================================================
 */
#pragma once
#include <Arduino.h>
#include "config.h"
#include "ak70.h"

// One quantity in both units. For speeds, .deg is deg/s and .rad is rad/s.
struct DegRad {
  float deg = 0;
  float rad = 0;
};

enum CommandType : uint8_t {
  CMD_LIMP, CMD_TORQUE, CMD_VELOCITY, CMD_POSITION, CMD_IMPEDANCE
};

// A command, stored internally in radians (SI units).
struct JointCommand {
  CommandType type = CMD_LIMP;
  float torqueNm  = 0;   // torque mode: the torque; impedance: feed-forward torque
  float posRad    = 0;   // position and impedance: target angle
  float velRad    = 0;   // velocity and impedance: target speed
  float kp        = 0;   // impedance stiffness, Nm/rad
  float kd        = 0;   // impedance damping, Nm/(rad/s)
  float maxVelRad = 0;   // position mode: speed limit, rad/s
  float maxAccRad = 0;   // position mode: acceleration limit, rad/s^2
};

struct Joint {
  // ---------------- identity (from config.h) ----------------
  uint8_t     canId = 0;
  const char* name  = "";
  int         dir   = 1;     // +1 or -1, motor frame -> joint frame

  // ---------------- feedback, joint frame (read these) ----------------
  bool    ok        = false;
  DegRad  pos;
  DegRad  vel;
  float   torqueNm  = 0;
  float   currentA  = 0;
  int     tempC     = 0;
  uint8_t errorCode = 0;

  // ---------------- the command for this tick (set with the functions) -----
  JointCommand cmd;

  void setLimp() { cmd = JointCommand(); }

  void setTorque(float torqueNm) {
    cmd = JointCommand();
    cmd.type = CMD_TORQUE;
    cmd.torqueNm = torqueNm;
  }

  void setVelocityRad(float radPerSec) {
    cmd = JointCommand();
    cmd.type = CMD_VELOCITY;
    cmd.velRad = radPerSec;
  }
  void setVelocityDeg(float degPerSec) { setVelocityRad(degPerSec * DEG_TO_RAD); }

  void setPositionRad(float rad,
                      float maxSpeedRadPerSec = 60.0f * DEG_TO_RAD,
                      float maxAccelRadPerSec2 = 180.0f * DEG_TO_RAD) {
    cmd = JointCommand();
    cmd.type = CMD_POSITION;
    cmd.posRad = rad;
    cmd.maxVelRad = maxSpeedRadPerSec;
    cmd.maxAccRad = maxAccelRadPerSec2;
  }
  void setPositionDeg(float deg, float maxSpeedDegPerSec = 60.0f,
                      float maxAccelDegPerSec2 = 180.0f) {
    setPositionRad(deg * DEG_TO_RAD, maxSpeedDegPerSec * DEG_TO_RAD,
                   maxAccelDegPerSec2 * DEG_TO_RAD);
  }

  void setImpedanceRad(float posRad, float velRadPerSec, float kpNmPerRad,
                       float kdNmPerRadPerSec, float torqueFFNm = 0) {
    cmd = JointCommand();
    cmd.type = CMD_IMPEDANCE;
    cmd.posRad = posRad;
    cmd.velRad = velRadPerSec;
    cmd.kp = kpNmPerRad;
    cmd.kd = kdNmPerRadPerSec;
    cmd.torqueNm = torqueFFNm;
  }
  // Gains per DEGREE are converted to per RADIAN (x 57.3) for the motor.
  void setImpedanceDeg(float posDeg, float velDegPerSec, float kpNmPerDeg,
                       float kdNmPerDegPerSec, float torqueFFNm = 0) {
    setImpedanceRad(posDeg * DEG_TO_RAD, velDegPerSec * DEG_TO_RAD,
                    kpNmPerDeg * RAD_TO_DEG, kdNmPerDegPerSec * RAD_TO_DEG, torqueFFNm);
  }

  // ---------------- internals (used by the template) ----------------
  ak70::Feedback fb;              // raw feedback, motor frame
  bool     announced     = false; // printed "found" yet?
  uint32_t ignoreFbUntil = 0;     // millis(); skip stale frames right after `zero`
  uint32_t fbCount       = 0;     // feedback frames received (for the rate)
  uint32_t fbCountPrev   = 0;
  uint32_t fbHz          = 0;     // measured feedback rate
  float    pvTargetDeg   = NAN;   // position mode: last target (motor frame)
  float    pvSign        = 1;     // position mode: direction of travel
  float    logTauCmdNm   = NAN;   // what was sent, for the log (NAN = motor's own loop)
  float    logPosCmdDeg  = NAN;   // commanded angle, for the log (NAN = none)
};

// The joints. joint[0] = motor 1 ("m1"), joint[1] = motor 2 ("m2").
Joint joint[NUM_MOTORS];

// True once `zero` has been typed since power-on. Soft limits need it.
bool zeroed = false;

// Every motor ID we have heard on the bus (helps diagnose a wrong ID).
bool seenCanId[256] = {};

// Copy the per-motor settings from config.h into the joints.
void initJoints() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    joint[i].canId = MOTOR_CAN_ID[i];
    joint[i].name  = MOTOR_NAME[i];
    joint[i].dir   = (MOTOR_DIR[i] < 0) ? -1 : 1;
  }
}

// Which joint has this CAN ID? -1 if none.
int jointIndexForId(uint8_t id) {
  for (int i = 0; i < NUM_MOTORS; i++)
    if (joint[i].canId == id) return i;
  return -1;
}

// Read every CAN frame waiting in the receive queue and store motor feedback.
// Called as often as possible from loop(), so the queue never overflows.
void pollCan() {
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    if (!m.extd) {
      static bool warned = false;
      if (!warned) {
        warned = true;
        Serial.println("# Note: got a STANDARD-ID CAN frame. V3 motors use extended IDs; "
                       "a motor still on the old (V1/V2) protocol would send these.");
      }
      continue;
    }
    uint8_t id;
    ak70::Feedback f;
    if (!ak70::decodeStatus(m, id, f)) continue;   // not a status frame
    seenCanId[id] = true;

    int i = jointIndexForId(id);
    if (i < 0) continue;                           // a motor we're not using
    Joint& j = joint[i];
    if ((int32_t)(millis() - j.ignoreFbUntil) < 0) continue;

    j.fb = f;
    j.fbCount++;
    if (!j.announced) {
      j.announced = true;
      Serial.printf("# Motor %d (%s, CAN ID %u) found: pos %.1f deg, temp %d C, err %u (%s)\n",
                    i + 1, j.name, j.canId, f.pos_deg * j.dir, f.temp_C, f.err,
                    ak70::errorName(f.err));
    }
  }
}

// Convert the latest raw feedback into the joint frame. Called at the start
// of every tick, so your controller always sees one consistent snapshot.
void updateJointFeedback() {
  uint32_t now = millis();
  for (int i = 0; i < NUM_MOTORS; i++) {
    Joint& j = joint[i];
    float d = (float)j.dir;
    j.ok        = j.fb.valid && (now - j.fb.t_ms) < FB_TIMEOUT_MS;
    j.pos.deg   = d * j.fb.pos_deg;
    j.pos.rad   = j.pos.deg * DEG_TO_RAD;
    j.vel.deg   = d * j.fb.vel_dps;
    j.vel.rad   = j.vel.deg * DEG_TO_RAD;
    j.currentA  = d * j.fb.cur_A;
    j.torqueNm  = j.currentA * KT_NM_PER_A;
    j.tempC     = j.fb.temp_C;
    j.errorCode = j.fb.err;
  }
}

// Turn each joint's (already safety-checked) command into a CAN frame,
// converting from the joint frame back to the motor's frame.
// Every motor gets exactly one frame per tick, even when limp: in
// "query-reply" mode, that frame is what makes the motor send feedback.
void sendJointCommands() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    Joint& j = joint[i];
    const JointCommand& c = j.cmd;
    float d = (float)j.dir;
    j.logTauCmdNm  = NAN;
    j.logPosCmdDeg = NAN;

    switch (c.type) {
      case CMD_LIMP:
        ak70::sendCurrent(j.canId, 0.0f);
        j.logTauCmdNm = 0;
        break;

      case CMD_TORQUE:
        ak70::sendCurrent(j.canId, d * c.torqueNm / KT_NM_PER_A);
        j.logTauCmdNm = c.torqueNm;
        break;

      case CMD_VELOCITY:
        ak70::sendVelocity(j.canId, d * c.velRad * RAD_TO_DEG);
        break;

      case CMD_POSITION: {
        float targetMotorDeg = d * c.posRad * RAD_TO_DEG;
        // Work out the direction of travel only when the target changes,
        // so it doesn't flip back and forth once the joint arrives.
        if (isnan(j.pvTargetDeg) || fabsf(targetMotorDeg - j.pvTargetDeg) > 0.01f) {
          j.pvSign = (targetMotorDeg >= j.fb.pos_deg) ? 1.0f : -1.0f;
          j.pvTargetDeg = targetMotorDeg;
        }
        ak70::sendPosVel(j.canId, targetMotorDeg,
                         j.pvSign * c.maxVelRad * RAD_TO_DEG,
                         j.pvSign * c.maxAccRad * RAD_TO_DEG);
        j.logPosCmdDeg = c.posRad * RAD_TO_DEG;
        break;
      }

      case CMD_IMPEDANCE:
        ak70::sendMit(j.canId, d * c.posRad, d * c.velRad, c.kp, c.kd, d * c.torqueNm);
        j.logPosCmdDeg = c.posRad * RAD_TO_DEG;
        // Torque the motor should apply right now, from the latest feedback
        j.logTauCmdNm = c.kp * (c.posRad - j.pos.rad) + c.kd * (c.velRad - j.vel.rad)
                        + c.torqueNm;
        break;
    }
    if (c.type != CMD_POSITION) j.pvTargetDeg = NAN;
  }
}

// `zero`: the current pose becomes 0 deg on every motor. Uses the motor's
// TEMPORARY origin (forgotten at power-off), the same command as `origin`
// in ak70_motor_test. Only allowed while limp (the console checks).
void zeroJoints() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    Joint& j = joint[i];
    ak70::sendOrigin(j.canId, false);
    // Frames already on their way still carry the old position.
    // Ignore feedback for a moment so nothing acts on a stale angle.
    j.fb.valid = false;
    j.ignoreFbUntil = millis() + 50;
  }
  zeroed = true;
}

// Measured feedback rate per motor. Call once per second.
void updateFeedbackRates() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    joint[i].fbHz = joint[i].fbCount - joint[i].fbCountPrev;
    joint[i].fbCountPrev = joint[i].fbCount;
  }
}
