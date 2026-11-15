# Hip exo control template

The starting point for every exo program. It runs on an ESP32 WROOM-32 (38-pin) with the Waveshare SN65HVD230 CAN board and drives one or two CubeMars AK70-9 V3 (KV60) motors.

The parts that already work on the real motor live in their own tabs: the CAN protocol (copied from `ak70_motor_test`), safety stops, logging, and the Serial Monitor commands. To write a new experiment, you only write a controller in **`my_controller.h`**.

---

## The files

All of these sit in one folder named `exo_template`. Opening `exo_template.ino` in the Arduino IDE shows each file as a tab.

| File | What's in it | Edit it? |
|---|---|---|
| `exo_template.ino` | `setup()`, `loop()`, and the order things happen each tick | Rarely |
| `config.h` | Pins, motor IDs and directions, loop rate, safety limits | Once per setup |
| **`my_controller.h`** | **Your controller** | **Every program** |
| `joints.h` | What the controller reads and commands (deg and rad) | No, but read the top |
| `safety.h` | Run/stop state, command limits, fault checks | No |
| `logger.h` | CSV data over serial | No |
| `console.h` | Serial Monitor commands | No |
| `ak70.h` | CubeMars CAN protocol | No |

When you click **Upload**, the IDE compiles every file in the folder into a single program and writes it to the ESP32. Each upload replaces the previous program entirely; the board never holds the files themselves.

## Workflow

**Install it once.** Put the `exo_template` folder in your Arduino sketchbook folder (usually `Documents/Arduino`). The folder name must match the `.ino` name.

**Start a new program.** Open `exo_template.ino`, choose **File → Save As…**, and give it a name like `hip_spring_test`. Save As copies the whole folder and renames the `.ino`. Edit the copy's `my_controller.h` and leave the template itself untouched.

**Upload.** Choose **Tools → Board → ESP32 Dev Module**, pick the port, and click **Upload**. Type `stop` first if a motor is running.

- If the code has a mistake, nothing is uploaded and the old program stays on the board. The error appears at the bottom of the IDE with the file name and line number.
- Uploading restarts the ESP32. It always boots with the motors limp, and you have to type `zero` again.

**Serial Monitor.** Set it to **921600 baud** with line ending **"Newline"**.

## One-time setup in CubeMars Tool (per motor)

- CAN rate: **1M**.
- Give each motor its own **CAN ID**, for example right = 1 and left = 2. These must match `MOTOR_CAN_ID` in `config.h`.
- CAN feedback: **periodic at 500 Hz**, or **query-reply**. With a slower feedback rate, the controller acts on stale data. `status` shows the rate the ESP32 is actually receiving.

## First run (motor clamped to the bench, nothing on the output)

1. Upload and open the Serial Monitor. You should see `# Motor 1 (right, CAN ID 1) found`.
2. Type `status`. Turn the shaft a quarter turn by hand and type `status` again. The position should change by about 90°, and the feedback rate should read about 500 Hz.
3. Type `zero`, then `run`. The example controller holds the shaft like a soft spring. Push it by hand and it pushes back. Press Enter to stop.
4. Type `log on` to see the data stream, then `log off`.

## Commands

| Command | Does |
|---|---|
| *Enter* on an empty line, `stop`, or `x` | All motors go **limp** (0 A). This is the fastest way to stop. |
| `run` | Starts your controller. It first checks feedback, error codes, temperature, that `zero` was done, and the soft limits. |
| `zero` | Makes the current pose 0° on every motor and arms the soft limits. Only works while limp. Lost at power-off. |
| `log on` / `log off` | CSV data stream |
| `status` | Feedback for each motor, why it last stopped, CAN bus health, loop timing, and log statistics |
| `help` | Lists all commands, including the ones your controller added |

The example controller adds `k <Nm/deg>` and `b <Nm/(deg/s)>` to change its gains while it runs.

## Writing a controller

`my_controller.h` has three functions:

```cpp
void controllerInit()  { }              // once at power-on: add commands and log columns
void controllerStart() { }              // when `run` is typed: reset timers, remember the pose
void controllerStep(float t, float dt)  // 500 times a second while running
{
  for (int i = 0; i < NUM_MOTORS; i++) {
    Joint& j = joint[i];
    // read:  j.pos.deg  j.pos.rad  j.vel.deg  j.vel.rad  j.torqueNm  j.currentA  j.tempC  j.ok
    // then set ONE command:
    j.setTorque(0.5f);
  }
}
```

In `controllerStep`, `t` is the number of seconds since `run` and `dt` is 0.002 s.

### Commands, in degrees or radians

| Command | Meaning |
|---|---|
| `setLimp()` | 0 A; the motor turns freely |
| `setTorque(Nm)` | Torque at the output shaft |
| `setVelocityDeg(deg/s)` / `setVelocityRad(rad/s)` | The motor's own speed loop |
| `setPositionDeg(deg, deg/s, deg/s²)` / `setPositionRad(...)` | The motor's own position loop. Speed and acceleration are optional (default 60°/s and 180°/s²). |
| `setImpedanceDeg(deg, deg/s, Nm/deg, Nm/(deg/s), Nm)` / `setImpedanceRad(rad, rad/s, Nm/rad, Nm/(rad/s), Nm)` | A spring and damper computed inside the motor (MIT mode): torque = kp·(target − angle) + kd·(target speed − speed) + feed-forward |

1 Nm/deg = 57.3 Nm/rad. A soft 5 Nm/rad spring is 0.087 Nm/deg.

### Rules

- **Set a command for every joint, every tick.** A joint you skip goes limp for that tick. That's safe, but probably not what you meant.
- **Never wait in `controllerStep`.** No `delay()`, no loops that wait on something, and no `Serial.printf` every tick. The whole tick has 2 ms, and `status` shows how long ticks are actually taking.
- **Use global variables** for anything that has to persist between ticks, or that a command or log column needs to reach.

### Adding a command or a log column (in `controllerInit`)

```cpp
float assistGain = 1.0f;   // global

addCommand("gain", "gain <x>   assist gain", [](float* args, int n) {
  if (n >= 1 && !isnan(args[0])) assistGain = args[0];
});
logColumn("assist_gain", &assistGain);
```

The handler gets each word after the command converted to a number. `on`, `yes` and `true` become 1; `off`, `no` and `false` become 0; anything else that isn't a number becomes `NAN`.

## Safety: what the template does for you

The safety layer runs **after** your controller every tick, so nothing in `my_controller.h` can get around it. All the limits are in `config.h`.

**Clamped** (with a one-time note per run):

- Torque is capped at 5 Nm in every mode.
- Speed is capped at 360°/s, and position-move acceleration at 720°/s².
- Impedance kp is capped at 60 Nm/rad and kd at 3 Nm/(rad/s).
- Impedance is also limited by *predicted* torque. A far-away target with a stiff spring gets pulled closer, so the motor never applies more than 5 Nm.
- Position targets are kept inside the soft limits, and at a limit, any torque or speed pushing further out is removed.

**Faults** (all motors go limp and stay limp until someone types `run` again):

- No feedback from a motor for more than 100 ms
- Any motor error code
- Driver board temperature of 75 °C or more
- Speed above 540°/s
- More than 5° past a soft limit
- Current above 8 A for more than 100 ms
- A NaN or infinite command, which means a bug in your controller

`run` refuses to start until `zero` has been typed. That way the soft limits (−100° to +100° by default) are always active while a controller runs.

**What it does not replace:** a switch on the motor power supply within reach (the real e-stop), mechanical joint stops, and a spotter whenever the exo is on a person. The limits are set for **bench testing**. Raise them one at a time, on purpose.

## Adding the second motor

1. In CubeMars Tool, give it CAN ID 2 and the same CAN settings as motor 1.
2. Wire its CAN_H and CAN_L in parallel with motor 1. Keep a 120 Ω terminator at each end of the bus.
3. In `config.h`, set `NUM_MOTORS 2`.
4. Check the directions: type `zero` and `status`, flex each hip forward by hand, and confirm both angles go **up**. If one goes down, set that motor's `MOTOR_DIR` to `-1`.
5. Upload. Controllers written with `for (int i = 0; i < NUM_MOTORS; i++)` work unchanged.

## Loop rate: 500 Hz

The controller runs, and each motor gets a command, **500 times a second**. With two motors, every tick puts four frames on the 1 Mbit/s CAN bus (one command and one feedback frame per motor). That's about 31% bus load, which leaves plenty of room. 1000 Hz would mean about 62%, where one noisy moment can start delaying frames.

Logging defaults to **250 rows per second** (`LOG_HZ`), which uses about a third of the serial link with two motors. If the link can't keep up, rows are skipped rather than slowing the motors, and `status` counts the skipped rows.

## Recording data

`exo_logger.py` (in `claude/ak70_free_spin/` in the project) works with this template unchanged. Its column names match.

```
python exo_logger.py --port COM5 --plot          # close the Serial Monitor first
```

Type commands as usual. A line starting with `!` adds a timestamped marker (`!start walking`). The data goes to `logs/run_<date>_<time>.csv`.

Log columns for each motor (m1, m2), all in the joint frame:

| Column | Meaning |
|---|---|
| `t_ms` | ESP32 time in ms |
| `state` | `limp` or `run` |
| `mN_pos_deg` | Angle from `zero` |
| `mN_vel_dps` | Speed in °/s |
| `mN_tau_meas_Nm` | Measured torque (current × 1.0621 Nm/A) |
| `mN_tau_cmd_Nm` | Commanded torque after safety. Torque and limp modes: exact. Impedance: predicted. Velocity/position modes: `nan`, because the motor's own loop decides the torque. |
| `mN_pos_cmd_deg` | Commanded angle in position or impedance mode, otherwise `nan` |
| `mN_cur_A`, `mN_temp_C`, `mN_err` | Current, driver temperature, CubeMars error code |

Any columns added with `logColumn()` come after these.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Garbage in the Serial Monitor | The baud rate isn't 921600. |
| Typing does nothing | The line ending isn't set to "Newline". |
| `No reply from motor 1` | Type `status` for a diagnosis: no power or wiring, CAN board pins, or a different CAN ID. To find an unknown ID, upload `ak70_motor_test` and use its `scan` command. |
| `run` says "type `zero` first" | That's intended: zero at the neutral pose so the soft limits mean something. |
| Stops with "no feedback … for 1xx ms" | The CAN feedback rate is too low or frames are being lost. Set 500 Hz in CubeMars Tool, then check `status` (rxMissed, txDropped). |
| `status` shows feedback well under 500 Hz | Same fix: raise CAN feedback to 500 Hz, or use query-reply. |
| Velocity is off by a factor of 1.5 | `POLE_PAIRS` in `config.h` (21 vs 14). Check: a 30°/s command should take 12 s per output revolution. |
| "limited" notes | Your controller asks for more than the limits allow. Change the controller, or raise the limit in `config.h` on purpose. |

**Never send the old `FF FF FF FF FF FF FF FC/FD/FE` "enter/exit/zero" frames** from V1/V2 examples to these V3 motors. A V3 motor reads them as an MIT command with full stiffness and about +32 Nm.

## How this was tested

- **Compiled** with `-Wall -Wextra` against the real ESP32 Arduino core 3.3.2 for "ESP32 Dev Module", with 1 and with 2 motors: no warnings. The program uses 23% of flash and 6% of RAM.
- **Run on a PC against a simulated AK70-9**, decoding frames straight from the v3.2.0 manual: 69 checks.
  - Covered: zero-current limp frames, the origin frame, the spring example's torque sign and balance, every command type in degrees and radians, the frame bytes for each, all clamps, every fault trip, the NaN guard, the 250 Hz log format, both feedback modes, and two motors with one reversed.
- **Not yet run on the real motor.** The CAN code is the same as `ak70_motor_test`, which does work on the motor. Still, do the first-run steps above on the bench before anything else.
