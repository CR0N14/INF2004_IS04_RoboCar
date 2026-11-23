# Subsystem 3 — test log

**Subsystem:** Barcode Decoding and IR Line Following  ·  **Student ID:** 2303564  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Packed frame preserves ADC values and sequence | IR frame transport | auto | Build with `TEST=1`; reset the Pico | `[TEST] S3-U01 PASS` |
| S3-U02 | Reversed calibration clips to black | IR normalisation | auto | Build with `TEST=1`; reset the Pico | `[TEST] S3-U02 PASS` |
| S3-U03 | Zero contrast is rejected | Calibration validation | auto | Build with `TEST=1`; reset the Pico | `[TEST] S3-U03 PASS` |
| S3-U04 | Timestamp wrap and half-range bounds | Frame age validation | auto | Build with `TEST=1`; reset the Pico | `[TEST] S3-U04 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Execution

The on-target cases above are defined in
`app_program/subsystem3/subsystem3_test.c`. Hardware execution remains pending.

Reusable host tests are under `host/`. From the project root, run:

```sh
python3 testing/subsystem3/host/run_tests.py
```

The runner requires Python 3, Clang and clang-format. It formats the tested
sources before checking and compiling them. Generated executables and the
default transcript (`build/test.log`) stay in the ignored `build/` directory.
Use `--application` to include adapter/navigation checks and application
compilation; `--arm` additionally compiles freestanding Cortex-M0+ objects.
Host checks do not establish hardware or operational integration readiness.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
