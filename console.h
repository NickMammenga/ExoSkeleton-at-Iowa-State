/*
 * ============================================================================
 *  console.h  -  SERIAL MONITOR COMMANDS                   (you shouldn't need to edit)
 * ============================================================================
 *
 *  Built-in commands (type in the Serial Monitor, line ending "Newline"):
 *
 *    <Enter>     an EMPTY line = stop (same as `stop`). Fastest way to stop.
 *    stop, x     all motors limp (0 A)
 *    run         start your controller (checks feedback, errors, limits first)
 *    zero        current pose = 0 deg on every motor; arms the soft limits
 *    log on|off  CSV data stream
 *    status      feedback, faults, CAN bus health, loop timing
 *    help, ?     list all commands, including the ones your controller added
 *
 *  Your controller can add its own commands with addCommand() in
 *  controllerInit(). The handler gets the words after the command as
 *  numbers: "on"/"yes"/"true" -> 1, "off"/"no"/"false" -> 0, anything that
 *  isn't a number -> NAN. Example, in my_controller.h:
 *
 *    addCommand("gain", "gain <x>   set the assist gain",
 *               [](float* args, int n) {
 *                 if (n >= 1 && !isnan(args[0])) assistGain = args[0];
 *               });
 *
 *  Typing `gain 2.5` then calls the handler with args[0] = 2.5, n = 1.
 * ============================================================================
 */
#pragma once
#include <Arduino.h>
#include "config.h"
#include "ak70.h"
#include "joints.h"
#include "safety.h"
#include "logger.h"

// Defined in exo_template.ino
void startRun();

// Loop timing, filled in by exo_template.ino and shown by `status`
uint32_t loopMaxTickUs = 0;   // longest tick (feedback + controller + safety + send + log)
uint32_t loopLateTicks = 0;   // ticks that started more than one period late
uint32_t bootMs        = 0;

// ---------------------------------------------------------------------------
//  Commands added by the controller
// ---------------------------------------------------------------------------
#define MAX_USER_COMMANDS 16
#define MAX_COMMAND_ARGS  6

typedef void (*CommandHandler)(float* args, int numArgs);

struct UserCommand {
  const char*    name;
  const char*    help;
  CommandHandler handler;
};
UserCommand userCommands[MAX_USER_COMMANDS];
int numUserCommands = 0;

// Register a command. `name` is what you type (lowercase, one word),
// `help` is the line shown by `help`.
void addCommand(const char* name, const char* help, CommandHandler handler) {
  if (numUserCommands >= MAX_USER_COMMANDS) {
    Serial.printf("# addCommand(\"%s\") ignored: max %d commands\n", name, MAX_USER_COMMANDS);
    return;
  }
  userCommands[numUserCommands++] = { name, help, handler };
}

// ---------------------------------------------------------------------------
//  help / status
// ---------------------------------------------------------------------------
void printHelp() {
  Serial.println(
    "# ------------------------------- commands -------------------------------\n"
    "#  <Enter> | stop | x   all motors limp (0 A)\n"
    "#  run                  start the controller\n"
    "#  zero                 current pose = 0 deg, arms soft limits (only when limp)\n"
    "#  log on|off           CSV data stream\n"
    "#  status               feedback, faults, CAN bus, loop timing\n"
    "#  help                 this list");
  if (numUserCommands > 0) {
    Serial.println("# ----- added by my_controller.h -----");
    for (int k = 0; k < numUserCommands; k++)
      Serial.printf("#  %s\n", userCommands[k].help);
  }
  Serial.println("# ------------------------------------------------------------------------");
}

// Plain-English guess at why a motor isn't answering.
void printDiagnosis(const twai_status_info_t& s) {
  if (s.state == TWAI_STATE_BUS_OFF || s.state == TWAI_STATE_RECOVERING) {
    Serial.println("# Diagnosis: the ESP32 can't hear its own CAN frames. The CAN board isn't "
                   "on CAN_TX_PIN/CAN_RX_PIN, CTX/CRX are swapped, or it has no 3.3 V.");
  } else if (s.tx_error_counter >= 96) {
    Serial.println("# Diagnosis: nobody is acknowledging our frames, so no motor is on the bus. "
                   "Check 48 V (blue light on), CANH->CAN_H (white), CANL->CAN_L (blue), "
                   "screw terminals, and termination.");
  } else {
    bool any = false;
    for (int id = 0; id < 256; id++) any = any || seenCanId[id];
    if (any) {
      Serial.print("# Diagnosis: the bus works. Motors heard at CAN ID");
      for (int id = 0; id < 256; id++) if (seenCanId[id]) Serial.printf(" %d", id);
      Serial.println(". Check MOTOR_CAN_ID in config.h.");
    } else {
      Serial.println("# Diagnosis: frames are acknowledged (a motor is on the bus) but nothing "
                     "answers the IDs in MOTOR_CAN_ID. Check the ID in CubeMars Tool, or run "
                     "ak70_motor_test's `scan`.");
    }
  }
}

void printStatus() {
  static const char* CAN_STATE[] = { "STOPPED", "RUNNING", "BUS_OFF", "RECOVERING" };
  uint32_t now = millis();

  Serial.printf("# state: %s", STATE_NAME[runState]);
  if (runState == STATE_LIMP) Serial.printf(" (last stop: %s)", lastStopReason);
  Serial.printf(" | zeroed: %s | soft limits [%.0f, %.0f] deg%s\n",
                zeroed ? "yes" : "NO", JOINT_MIN_DEG, JOINT_MAX_DEG,
                zeroed ? "" : " (inactive until `zero`)");

  bool anyMissing = false;
  for (int i = 0; i < NUM_MOTORS; i++) {
    const Joint& j = joint[i];
    Serial.printf("# m%d %s (CAN ID %u, dir %+d): ", i + 1, j.name, j.canId, j.dir);
    if (!j.fb.valid) {
      Serial.println("no feedback received");
      anyMissing = true;
      continue;
    }
    Serial.printf("pos %.2f deg (%.4f rad)  vel %.1f deg/s  torque %.2f Nm (%.2f A)  "
                  "temp %d C  err %u (%s)  feedback %lu Hz, age %lu ms\n",
                  j.pos.deg, j.pos.rad, j.vel.deg, j.torqueNm, j.currentA, j.tempC,
                  j.errorCode, ak70::errorName(j.errorCode), (unsigned long)j.fbHz,
                  (unsigned long)(now - j.fb.t_ms));
    if (!j.ok) anyMissing = true;
    if (j.ok && j.fbHz < CONTROL_HZ / 2) {
      Serial.printf("#    note: feedback is only %lu Hz, so the controller often sees "
                    "old data. Set CAN feedback to %d Hz (or query-reply) in CubeMars Tool.\n",
                    (unsigned long)j.fbHz, CONTROL_HZ);
    }
  }

  twai_status_info_t s = {};
  twai_get_status_info(&s);
  Serial.printf("# CAN: %s  txErr %lu  rxErr %lu  busErr %lu  rxMissed %lu  txDropped %lu\n",
                s.state <= 3 ? CAN_STATE[s.state] : "?", (unsigned long)s.tx_error_counter,
                (unsigned long)s.rx_error_counter, (unsigned long)s.bus_error_count,
                (unsigned long)s.rx_missed_count, (unsigned long)ak70::txDropped);
  Serial.printf("# loop: %d Hz, longest tick %lu us of %lu us, late ticks %lu | "
                "log %s at %d Hz, rows written %lu, skipped %lu\n",
                CONTROL_HZ, (unsigned long)loopMaxTickUs, (unsigned long)(1000000UL / CONTROL_HZ),
                (unsigned long)loopLateTicks, logOn ? "ON" : "off", LOG_HZ,
                (unsigned long)logRowsWritten, (unsigned long)logRowsSkipped);
  if (anyMissing) printDiagnosis(s);
}

// Unprompted hints when a motor never answers or stops answering.
// Called a few times per second.
void printHints() {
  static uint32_t lastHint = 0;
  static bool lostWarned[MAX_MOTORS] = {};
  uint32_t now = millis();
  for (int i = 0; i < NUM_MOTORS; i++) {
    const Joint& j = joint[i];
    if (!j.announced) {
      if (now - bootMs > 1500 && now - lastHint > 4000) {
        lastHint = now;
        Serial.printf("# No reply from motor %d (CAN ID %u) yet. Type `status` for a diagnosis.\n",
                      i + 1, j.canId);
      }
    } else if (j.fb.valid && now - j.fb.t_ms > FB_TIMEOUT_MS) {
      if (!lostWarned[i]) {
        lostWarned[i] = true;
        Serial.printf("# Lost feedback from motor %d.\n", i + 1);
      }
    } else {
      lostWarned[i] = false;
    }
  }
}

// ---------------------------------------------------------------------------
//  Parsing
// ---------------------------------------------------------------------------

// One word -> number. on/yes/true = 1, off/no/false = 0, not a number = NAN.
float parseArg(const char* w) {
  if (!strcmp(w, "on") || !strcmp(w, "yes") || !strcmp(w, "true"))  return 1.0f;
  if (!strcmp(w, "off") || !strcmp(w, "no") || !strcmp(w, "false")) return 0.0f;
  char* end;
  float v = strtof(w, &end);
  return (end != w && *end == '\0') ? v : NAN;
}

void handleLine(char* line) {
  // lowercase everything so "RUN" and "Run" work too
  for (char* p = line; *p; ++p) *p = (char)tolower((unsigned char)*p);

  // Split into words
  char* words[1 + MAX_COMMAND_ARGS];
  int nw = 0;
  for (char* tok = strtok(line, " \t"); tok && nw < 1 + MAX_COMMAND_ARGS; tok = strtok(nullptr, " \t"))
    words[nw++] = tok;

  // An empty line is STOP. Always.
  if (nw == 0) { goLimp("Enter"); return; }

  const char* cmd = words[0];
  float args[MAX_COMMAND_ARGS];
  int nargs = nw - 1;
  for (int k = 0; k < nargs; k++) args[k] = parseArg(words[k + 1]);

  if (!strcmp(cmd, "stop") || !strcmp(cmd, "x")) {
    goLimp("stop");

  } else if (!strcmp(cmd, "run")) {
    startRun();

  } else if (!strcmp(cmd, "zero")) {
    if (runState != STATE_LIMP) { Serial.println("# stop first, then `zero`"); return; }
    zeroJoints();
    Serial.printf("# zero set: this pose = 0 deg on every motor (until power-off). "
                  "Soft limits [%.0f, %.0f] deg are active.\n", JOINT_MIN_DEG, JOINT_MAX_DEG);

  } else if (!strcmp(cmd, "log")) {
    if (nargs < 1 || isnan(args[0])) { Serial.println("# usage: log on|off"); return; }
    logOn = args[0] != 0;
    if (logOn) printLogHeader();
    else Serial.println("# log off");

  } else if (!strcmp(cmd, "status")) {
    printStatus();

  } else if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
    printHelp();

  } else {
    for (int k = 0; k < numUserCommands; k++) {
      if (!strcmp(cmd, userCommands[k].name)) {
        userCommands[k].handler(args, nargs);
        return;
      }
    }
    Serial.printf("# unknown command '%s'. Type help\n", cmd);
  }
}

// Collect characters into a line; run handleLine() on each newline.
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
