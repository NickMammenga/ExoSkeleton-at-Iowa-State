/*
 * ============================================================================
 *  my_controller.h  -  YOUR PROGRAM.  This is the file you edit.
 * ============================================================================
 *
 *  Starting a new experiment:
 *    1. Arduino IDE: File -> Save As..., give it a new name. That copies the
 *       whole folder, so the template stays untouched.
 *    2. In the copy, replace the example below with your own controller.
 *    3. Change limits/IDs in config.h if needed. Leave the other files alone.
 *
 *  The template calls three functions here:
 *
 *    controllerInit()    once at power-on. Add your commands and log columns.
 *    controllerStart()   when someone types `run`. Reset timers, remember the
 *                        starting pose, clear integrators, etc.
 *    controllerStep(t, dt)
 *                        every tick (CONTROL_HZ = 500 times a second) while
 *                        running. t = seconds since `run`, dt = 0.002 s.
 *                        Read joint[i] feedback, set ONE command per joint.
 *                        Must be quick: the whole tick has 2000 us, and
 *                        `status` shows how long ticks are taking.
 *
 *  What you can use (details at the top of joints.h):
 *
 *    NUM_MOTORS                        1 or 2, from config.h
 *    joint[i].ok                       fresh feedback?
 *    joint[i].pos.deg / .pos.rad       angle from `zero`
 *    joint[i].vel.deg / .vel.rad       speed (deg/s, rad/s)
 *    joint[i].torqueNm, .currentA, .tempC, .errorCode
 *
 *    joint[i].setLimp()
 *    joint[i].setTorque(Nm)
 *    joint[i].setVelocityDeg(deg/s)            .setVelocityRad(rad/s)
 *    joint[i].setPositionDeg(deg [, deg/s, deg/s^2])
 *    joint[i].setPositionRad(rad [, rad/s, rad/s^2])
 *    joint[i].setImpedanceDeg(deg, deg/s, Nm/deg, Nm/(deg/s) [, Nm])
 *    joint[i].setImpedanceRad(rad, rad/s, Nm/rad, Nm/(rad/s) [, Nm])
 *
 *    addCommand("name", "help text", handler)   new Serial Monitor command
 *    logColumn("name", &floatVariable)          new CSV column
 *    DEG_TO_RAD, RAD_TO_DEG, PI, TWO_PI         unit conversion constants
 *
 *  Don't use delay() or anything that waits in controllerStep(): it would
 *  stall the motors' commands and the safety checks.
 *  Printing with Serial.printf() every tick is too much; print on events.
 *
 *  You never have to clamp torques or check limits yourself: safety.h does
 *  that after controllerStep() every tick. Clamping is a backstop, though,
 *  not a design: if you see "limited" notes, your controller is asking for
 *  more than the limits allow.
 * ============================================================================
 *
 *  EXAMPLE: a virtual spring and damper, computed here on the ESP32.
 *  When you type `run`, each joint is held where it is. Push it away and it
 *  pushes back, harder the further you go:
 *       torque = k * (hold angle - angle)  -  b * speed
 *  With k = 0.05 Nm/deg, a 20 deg push gives 1 Nm. Commands `k` and `b`
 *  change the gains while it runs. It's written in degrees; a radian version
 *  is in the comment inside controllerStep().
 * ============================================================================
 */
#pragma once
#include "joints.h"
#include "console.h"
#include "logger.h"

// ---------------------------------------------------------------------------
//  Your variables (global, so commands and log columns can reach them)
// ---------------------------------------------------------------------------
float springK = 0.05f;    // stiffness, Nm per degree
float damperB = 0.002f;   // damping, Nm per (deg/s)
float holdDeg[NUM_MOTORS];   // each joint's angle when `run` was typed

// ---------------------------------------------------------------------------
//  Runs once at power-on
// ---------------------------------------------------------------------------
void controllerInit() {
  // `k 0.1` sets the stiffness. The [] (...) { ... } is a small function
  // written in place; args[0] is the first number typed after the command.
  addCommand("k", "k <Nm/deg>           spring stiffness (example default 0.05)",
             [](float* args, int n) {
               if (n < 1 || isnan(args[0]) || args[0] < 0) {
                 Serial.println("# usage: k <Nm/deg>, e.g. k 0.05");
                 return;
               }
               springK = args[0];
               Serial.printf("# k = %.3f Nm/deg\n", springK);
             });

  addCommand("b", "b <Nm/(deg/s)>       damping (example default 0.002)",
             [](float* args, int n) {
               if (n < 1 || isnan(args[0]) || args[0] < 0) {
                 Serial.println("# usage: b <Nm/(deg/s)>, e.g. b 0.002");
                 return;
               }
               damperB = args[0];
               Serial.printf("# b = %.4f Nm/(deg/s)\n", damperB);
             });

  // Extra CSV columns, so the data records which gains were in use
  logColumn("k_Nm_per_deg", &springK);
  logColumn("b_Nm_per_dps", &damperB);
}

// ---------------------------------------------------------------------------
//  Runs when someone types `run` (after the safety checks pass)
// ---------------------------------------------------------------------------
void controllerStart() {
  for (int i = 0; i < NUM_MOTORS; i++) holdDeg[i] = joint[i].pos.deg;
  Serial.printf("# spring: k %.3f Nm/deg, b %.4f Nm/(deg/s), holding the current pose\n",
                springK, damperB);
}

// ---------------------------------------------------------------------------
//  Runs every tick while running.  t = seconds since `run`, dt = tick length
// ---------------------------------------------------------------------------
void controllerStep(float t, float dt) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    Joint& j = joint[i];

    float errorDeg = holdDeg[i] - j.pos.deg;                  // how far it was pushed
    float torque   = springK * errorDeg - damperB * j.vel.deg; // Nm
    j.setTorque(torque);

    // The same thing in radians would be:
    //   float kRad = springK * RAD_TO_DEG;   // Nm/rad
    //   float bRad = damperB * RAD_TO_DEG;   // Nm/(rad/s)
    //   j.setTorque(kRad * (holdDeg[i] * DEG_TO_RAD - j.pos.rad) - bRad * j.vel.rad);
    //
    // Or let the motor compute the spring itself (MIT/impedance mode):
    //   j.setImpedanceDeg(holdDeg[i], 0, springK, damperB);
  }
  (void)t;    // the example doesn't use these;
  (void)dt;   // they're here for your controllers
}
