# Subsystem 1 — test log

**Subsystem:** WiFi Communication, Command and Telemetry + State Machine  ·  **Student ID:** 2303567  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | statechart context, actions and event codes fit their packed sizes | CR1 | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | power-on enters INITIALISING and starts the init timeout | FR1 | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | required subsystems ready → STANDBY, sensors only | FR1 | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | start rejected outside STANDBY and never replayed later | FR4 | auto | `subsystem1_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | start → NAVIGATING with sensors and motors enabled | FR4, FR11 | auto | `subsystem1_test.c` | `[TEST] S1-U05 PASS` |
| S1-U06 | stop → STANDBY from NAVIGATING and OBSTACLE AVOIDANCE | FR14 | auto | `subsystem1_test.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | WiFi loss while navigating is reported, car keeps going | FR6 | auto | `subsystem1_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | fatal fault → ERROR with motors off; IMU fault non-fatal | FR12, FR27 | auto | `subsystem1_test.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | ERROR retries initialisation 3 times then stays in ERROR | design §5 | auto | `subsystem1_test.c` | `[TEST] S1-U09 PASS` |
| S1-U10 | obstacle: hold, bypass, recovery, back to NAVIGATING | FR15–FR19 | auto | `subsystem1_test.c` | `[TEST] S1-U10 PASS` |
| S1-U11 | barcode turn is commanded at the next junction | FR13, FR33 | auto | `subsystem1_test.c` | `[TEST] S1-U11 PASS` |
| S1-U12 | barcode bindings commit only in STANDBY | FR4 | auto | `subsystem1_test.c` | `[TEST] S1-U12 PASS` |
| S1-U13 | State Machine task runs and publishes a valid status word | IF-03 | auto | `subsystem1_test.c` | `[TEST] S1-U13 PASS` |
| S1-U14 | default barcode bindings A–D, other symbols unbound | IF-05 | auto | `subsystem1_test.c` | `[TEST] S1-U14 PASS` |
| S1-U15 | state changes print on the console at power-on | FR1 | manual | 1. Flash `make NET=lwip MQTT=1` image with no broker running. 2. Watch console 40 s. | `sm: INITIALISING -> ERROR` after 30 s, retry after 2 s |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
