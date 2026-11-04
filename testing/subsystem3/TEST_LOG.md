# Subsystem 3 — test log

**Subsystem:** Barcode Decoding and IR Line Following  ·  **Student ID:** 2303564  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | example: runs in a task context | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | | | manual | 1. … 2. … | |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## Integrated implementation update 

`app_program/subsystem3/subsystem3_test.c` now defines the following pure-core
on-target cases. They were compiled for APP_TEST=1 but have not run on a Pico.
The original template's running-in-task check is replaced by real core checks.

| ID | Current test | Hardware execution |
|---|---|---|
| S3-U01 | Packed frame preserves ADC values and sequence | Pending |
| S3-U02 | Reversed calibration clips to black | Pending |
| S3-U03 | Zero contrast rejected | Pending |
| S3-U04 | Timestamp wrap and half-range bounds | Pending |

Host/application integration results: `BUDDY3_FLOW.md`.
76 core checks and 44 adapter checks passed. Every application C file compiled
and partially linked in all eight SMP/TEST/MQTT profiles. These are not hardware
results and do not establish operational readiness.

