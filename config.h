/*
 * ============================================================================
 *  config.h  -  SETTINGS FOR YOUR HARDWARE AND SAFETY LIMITS
 * ============================================================================
 *
 *  This is the only file besides my_controller.h that you should normally
 *  change. Everything that depends on how YOUR exo is wired and how careful
 *  you want to be lives here, so nobody has to hunt through the other files.
 *
 *  Units used throughout the template:
 *    angles   deg  (degrees)   or rad   (radians)
 *    speeds   deg/s            or rad/s
 *    torque   Nm  at the OUTPUT shaft (after the 9:1 gearbox)
 *    current  A   (motor phase current)
 *
 *  "Joint frame" means: angle measured from the pose where you typed `zero`,
 *  with the sign flipped for any motor whose MOTOR_DIR is -1, so that the
 *  same physical motion (e.g. hip flexion) is positive on both legs.
 * ============================================================================
 */
#pragma once
#include <Arduino.h>

// ============================================================================
//  WIRING
// ============================================================================
// ESP32 pins connected to the Waveshare SN65HVD230 CAN board.
// (Never use GPIO1/GPIO3, the pins labeled TX/RX: those are the USB serial.)
#define CAN_TX_PIN   17     // ESP32 -> CAN board "CTX"
#define CAN_RX_PIN   16     // ESP32 <- CAN board "CRX"

// Serial Monitor speed. Set the Serial Monitor (or exo_logger.py --baud)
// to the SAME number or you'll see garbage. 921600 is needed to log two
// motors at a high rate; 115200 is too slow.
#define SERIAL_BAUD  921600


// ============================================================================
//  MOTORS
// ============================================================================
// How many motors are connected RIGHT NOW (1 or 2).
// To add the second motor later:
//   1. In CubeMars Tool, give it its own CAN ID (e.g. 2) and the same CAN
//      settings as motor 1 (1 Mbps, feedback 500 Hz or query-reply).
//   2. Wire its CAN_H/CAN_L in parallel with motor 1 (same two wires).
//   3. Change NUM_MOTORS to 2, check MOTOR_CAN_ID[1] and MOTOR_DIR[1] below.
//   4. Upload. Nothing else in the code needs to change.
#define NUM_MOTORS   1

#define MAX_MOTORS   2      // size of the arrays below; don't change

// Per-motor settings. Index 0 = motor 1 ("m1" in the logs),
// index 1 = motor 2 ("m2"). Only the first NUM_MOTORS entries are used.

// CAN ID of each motor, as set in CubeMars Tool.
const uint8_t MOTOR_CAN_ID[MAX_MOTORS] = { 1, 2 };

// A name for each motor, shown in messages.
const char* const MOTOR_NAME[MAX_MOTORS] = { "right", "left" };

// Direction: +1 or -1. The two hip motors face opposite ways, so the same leg
// motion turns them in opposite directions. Pick the signs so that flexing
// the hip (leg forward) makes the angle go UP on both legs: type `status`,
// move the leg forward by hand, and flip the sign here if the angle went down.
const int MOTOR_DIR[MAX_MOTORS] = { +1, +1 };


// ============================================================================
//  LOOP TIMING
// ============================================================================
// How often the controller runs and commands are sent to every motor.
// 500 Hz is the fastest rate that leaves the CAN bus comfortably below half
// load with 2 motors (each tick = 1 command + 1 feedback frame per motor,
// about 31% of a 1 Mbit/s bus). 1000 Hz would push it to about 62%, where
// one noisy moment can start delaying frames.
// For your controller to see fresh data every tick, set each motor's CAN
// feedback in CubeMars Tool to "periodic" at 500 Hz (or "query-reply").
#define CONTROL_HZ   500

// How often a CSV row is printed when logging is on. Must divide CONTROL_HZ
// evenly (500, 250, 125, 100, 50 ...). 250 Hz with 2 motors uses about a
// third of the 921600 baud serial link, which leaves room for extra columns.
#define LOG_HZ       250


// ============================================================================
//  MOTOR MODEL: CubeMars AK70-9 V3, KV60
// ============================================================================
// Only change these if you change motor model.
const float GEAR_RATIO = 9.0f;
const float KT_NM_PER_A = 1.0621f;  // output torque per amp (manual v3.2.0 table)
// Pole pairs: only used to convert speed (velocity commands and the speed
// feedback). CubeMars' spec page says 21. If speeds look wrong by a factor
// of 1.5 (e.g. a 30 deg/s command takes 8 s or 18 s per revolution instead
// of 12 s), try 14.
const float POLE_PAIRS = 21.0f;


// ============================================================================
//  SAFETY LIMITS
// ============================================================================
// The safety layer (safety.h) runs AFTER your controller every tick, so
// these apply no matter what my_controller.h asks for. They are set for
// bench testing. Raise them deliberately, one at a time, and write down why.

// ---- Command limits: commands are clamped to these ----
const float LIM_TORQUE_NM      = 5.0f;    // max torque in any mode (about 4.7 A)
const float LIM_SPEED_DPS      = 360.0f;  // max commanded speed, deg/s
const float LIM_ACCEL_DPS2     = 720.0f;  // max accel for setPosition moves, deg/s^2
const float LIM_KP_NM_PER_RAD  = 60.0f;   // max impedance stiffness, Nm/rad
const float LIM_KD_NM_S_PER_RAD = 3.0f;   // max impedance damping, Nm/(rad/s)

// ---- Soft position limits, joint frame, degrees from `zero` ----
// Only active after `zero` has been typed (in this power-on session).
// Commands that would push past these are cut back; going further than
// TRIP_POS_MARGIN_DEG past them sends every motor LIMP.
// On the exo, set these from your mechanical design (for example
// -25 extension / +110 flexion).
const float JOINT_MIN_DEG = -100.0f;
const float JOINT_MAX_DEG =  100.0f;

// true = `run` is refused until `zero` has been typed in this session, so
// the soft limits are always active while your controller runs.
const bool REQUIRE_ZERO_BEFORE_RUN = true;

// ---- Faults: any of these sends ALL motors LIMP (0 A) ----
// You then have to type `run` again.
const uint32_t FB_TIMEOUT_MS      = 100;    // no feedback from a motor for this long
const float    TRIP_POS_MARGIN_DEG = 5.0f;  // this far past a soft limit
const float    TRIP_SPEED_DPS     = 540.0f; // measured speed above this
const float    TRIP_CURRENT_A     = 8.0f;   // measured current above this...
const uint32_t TRIP_CURRENT_MS    = 100;    // ...for longer than this
const int      TRIP_TEMP_C        = 75;     // driver board temperature
// (Any error code reported by the motor also trips.)
