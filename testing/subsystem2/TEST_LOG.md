# Subsystem 2 — test log

**Subsystem:** Motion Control  ·  **Student ID:** 2303576  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | PWM slices 4+5 at 20 kHz (TOP 6249, DIV 1) and enabled | NFR9 | auto | `make TEST=1`, `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | Both motors braked while MOTORS_ENABLED is clear | FR11 | auto | as S2-U01 | `[TEST] S2-U02 PASS` |
| S2-U03 | Encoder edge interrupts enabled on GP4/5/16/17 | NFR7 | auto | as S2-U01 | `[TEST] S2-U03 PASS` |
| S2-U04 | Quadrature table: +4 per forward cycle, −4 reverse, skipped edge = error | FR9, NFR7 | auto | as S2-U01 | `[TEST] S2-U04 PASS` |
| S2-U05 | Steering mix: 0 straight, +50 slows right, −100 pivots left, clamps | FR13 | auto | as S2-U01 | `[TEST] S2-U05 PASS` |
| S2-U06 | PID: zero error → 0 %, sign follows error, limited to ±100 % | FR7 | auto | as S2-U01 | `[TEST] S2-U06 PASS` |
| S2-U07 | Stall detector: duty > 20 % and no pulse for 300 ms, not sooner | FR10 | auto | as S2-U01 | `[TEST] S2-U07 PASS` |
| S2-U08 | IF-07 snapshot at rest: E_OK, DISABLED, speed 0 | FR9, IF-07 | auto | as S2-U01, wheels still | `[TEST] S2-U08 PASS` |
| S2-U09 | Measured values filled in (`ENC_COUNTS_PER_REV`, `WHEEL_DIAMETER_MM`, `TRACK_WIDTH_MM`) | FR7, NFR8 | auto | as S2-U01 | `[TEST] S2-U09 PASS` (FAILS until measured) |
| S2-U10 | Manoeuvre segments: distance done-check, spin-left wheel directions | FR13 | auto | as S2-U01 | `[TEST] S2-U10 PASS` |
| S2-U11 | Wheel direction and brake vs coast | FR7, FR12 | manual | Wheels up. Bench kit `z_motion_control` `test_motor.uf2`. Watch each wheel | Both wheels turn forward on "forward"; brake stops faster than coast |
| S2-U12 | Encoder counts per wheel turn | FR9, NFR7 | manual | Bench kit `test_encoder.uf2`. Press `r`, turn one wheel 10 turns forward by hand; 3 times per wheel | Count goes up; 3 runs agree within ±2; err 0. Record count/10 as `ENC_COUNTS_PER_REV` |
| S2-U13 | Duty sweep (feed-forward + stall threshold data) | FR10, NFR7 | manual | Bench kit `test_speed.uf2`, wheels up then on the floor; save CSV in `logs/` | err 0 at every step; gives `MOTOR_START_DUTY_PCT`, `MOTOR_TOP_SPEED_MMPS` |
| S2-U14 | Speed loop holds cruise speed | NFR5, NFR6, FR8 | manual | `MOTION_DEBUG_PRINT_MS 100`, `MOTION_BRINGUP_IGNORE_STEERING 1`; base station START, wheels up, then floor | After 1 s each wheel within ±10 % of `CRUISE_SPEED_MMPS` |
| S2-U15 | Stop command ramps down; ERROR brakes at once | FR12, FR14, NFR11 | manual | Drive as S2-U14; send STOP; repeat and trigger an ERROR (e.g. S2-U16) | STOP: smooth stop then DISABLED. ERROR: brake at once, stop no slower than STOP |
| S2-U16 | Stall fault | FR10 | manual | Drive as S2-U14 wheels up, hold one wheel still | Console `s2: STALL` within ~0.3 s, motors braked, State Machine → ERROR with MOTOR_STALL |
| S2-U17 | Distance over 1 m | NFR8, FR9 | manual | Mark 1 m on the floor, drive it straight, read `distance_cm` in telemetry | 95–105 cm |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
