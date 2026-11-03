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
| S1-U15 | no broker: INITIALISING times out into ERROR and retries | FR1 | manual | 1. Flash `make NET=lwip MQTT=1` image with no broker running. 2. Watch console 40 s. | `sm: INITIALISING -> ERROR` after 30 s, retry after 2 s |
| S1-U16 | bit packer writes/reads exact widths, detects overflow | CR1 | auto | `subsystem1_test.c` | `[TEST] S1-U16 PASS` |
| S1-U17 | telemetry frame 4 bytes empty, 31 bytes with all blocks | FR2, CR1 | auto | `subsystem1_test.c` | `[TEST] S1-U17 PASS` |
| S1-U18 | heartbeat frame is 6 bytes | FR3, CR1 | auto | `subsystem1_test.c` | `[TEST] S1-U18 PASS` |
| S1-U19 | start/stop/ping accepted only in their exact 1-byte form | FR4 | auto | `subsystem1_test.c` | `[TEST] S1-U19 PASS` |
| S1-U20 | barcode BIND decoded; duplicate, out-of-range, short rejected | FR4 | auto | `subsystem1_test.c` | `[TEST] S1-U20 PASS` |
| S1-U21 | events batch back to back at their exact width | FR2, CR1 | auto | `subsystem1_test.c` | `[TEST] S1-U21 PASS` |
| S1-U22 | `net_post_event()` rejects bad type / oversized payload | IF-08 | auto | `subsystem1_test.c` | `[TEST] S1-U22 PASS` |
| S1-U23 | connects to WiFi + MQTT, reaches STANDBY, publishes status/tlm/hb | FR1–FR3, NFR1 | manual | 1. Broker running, `wifi_secrets.h` set. 2. Flash `make NET=lwip MQTT=1`. 3. `python tools/base_station.py monitor` for 30 s. | `status online`; `tlm` at 5 Hz, `hb` at 1 Hz; `STATE INITIALISING->STANDBY` |
| S1-U24 | start, stop, ping, bind acknowledged within 500 ms | FR4, NFR2 | manual | `base_station.py ping`, `start`, `stop`, `bind A=right`, in STANDBY and in NAVIGATING | ACKs as in docs/MQTT.md, ping round trip < 500 ms |
| S1-U25 | broker restart: car reconnects within 10 s and keeps its state | FR1, FR6, NFR2 | manual | 1. Start a run. 2. Stop the broker 20 s. 3. Restart it. | `offline` then `online` within 10 s of restart; state still NAVIGATING; `DROPPED` only if > 32 events |
| S1-U26 | malformed commands rejected without a state change | FR4 | manual | `base_station.py raw 21`, `raw 20 00`, `raw e0` | `CMD_ACK ... rejected`, state unchanged |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
