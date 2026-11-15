/*
 * ============================================================================
 *  logger.h  -  CSV DATA OVER SERIAL                       (you shouldn't need to edit)
 * ============================================================================
 *
 *  `log on` prints a header line, then one CSV row every 1/LOG_HZ seconds.
 *  `log off` stops it. Lines starting with "#" are messages, not data, so a
 *  CSV reader (or exo_logger.py) can skip them.
 *
 *  Columns, for each motor N (m1, m2), all in the joint frame:
 *    t_ms              ESP32 time in milliseconds
 *    state             limp / run
 *    mN_pos_deg        angle from `zero`
 *    mN_vel_dps        speed, deg/s
 *    mN_tau_meas_Nm    measured torque (current x 1.0621 Nm/A)
 *    mN_tau_cmd_Nm     torque asked for after the safety limits. Torque and
 *                      limp: exact. Impedance: predicted from the latest
 *                      feedback. Velocity/position: nan (the motor's own
 *                      loop decides the torque).
 *    mN_pos_cmd_deg    commanded angle (position/impedance), else nan
 *    mN_cur_A          measured current
 *    mN_temp_C         driver board temperature
 *    mN_err            motor error code (0 = OK)
 *  then any columns added with logColumn() in my_controller.h.
 *
 *  These names match what exo_logger.py expects, so it records and plots
 *  this data without changes:
 *     python exo_logger.py --port COM5 --plot
 *
 *  If the serial link can't keep up (too many columns, LOG_HZ too high),
 *  rows are skipped rather than slowing the control loop. `status` shows
 *  how many were skipped.
 * ============================================================================
 */
#pragma once
#include <Arduino.h>
#include "config.h"
#include "joints.h"

#define MAX_LOG_COLUMNS 12

bool     logOn          = false;
uint32_t logRowsWritten = 0;
uint32_t logRowsSkipped = 0;

struct LogColumn {
  const char*  name;
  const float* value;
};
LogColumn logColumns[MAX_LOG_COLUMNS];
int numLogColumns = 0;

// Add your own column to the CSV. Call it in controllerInit().
// `value` must point to a float that stays alive (a global variable).
//   logColumn("assist_gain", &assistGain);
void logColumn(const char* name, const float* value) {
  if (numLogColumns >= MAX_LOG_COLUMNS) {
    Serial.printf("# logColumn(\"%s\") ignored: max %d custom columns\n", name, MAX_LOG_COLUMNS);
    return;
  }
  logColumns[numLogColumns].name  = name;
  logColumns[numLogColumns].value = value;
  numLogColumns++;
}

void printLogHeader() {
  Serial.print("t_ms,state");
  for (int i = 0; i < NUM_MOTORS; i++) {
    int n = i + 1;
    Serial.printf(",m%d_pos_deg,m%d_vel_dps,m%d_tau_meas_Nm,m%d_tau_cmd_Nm,"
                  "m%d_pos_cmd_deg,m%d_cur_A,m%d_temp_C,m%d_err",
                  n, n, n, n, n, n, n, n);
  }
  for (int k = 0; k < numLogColumns; k++) Serial.printf(",%s", logColumns[k].name);
  Serial.println();
}

// Build one row in memory, then send it only if the serial transmit buffer
// has room for all of it. Waiting for room would stall the control loop.
void writeLogRow(uint32_t t_ms, const char* state) {
  static char row[512];
  int len = snprintf(row, sizeof(row), "%lu,%s", (unsigned long)t_ms, state);
  for (int i = 0; i < NUM_MOTORS && len < (int)sizeof(row); i++) {
    const Joint& j = joint[i];
    len += snprintf(row + len, sizeof(row) - len, ",%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%d,%u",
                    j.pos.deg, j.vel.deg, j.torqueNm, j.logTauCmdNm, j.logPosCmdDeg,
                    j.currentA, j.tempC, j.errorCode);
  }
  for (int k = 0; k < numLogColumns && len < (int)sizeof(row); k++) {
    len += snprintf(row + len, sizeof(row) - len, ",%.5g", *logColumns[k].value);
  }
  if (len >= (int)sizeof(row) - 2) {   // shouldn't happen; never send a cut-off row
    logRowsSkipped++;
    return;
  }
  row[len++] = '\n';
  if (Serial.availableForWrite() >= len) {
    Serial.write((const uint8_t*)row, len);
    logRowsWritten++;
  } else {
    logRowsSkipped++;
  }
}
