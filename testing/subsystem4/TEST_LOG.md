# Subsystem 4 — test log

**Subsystem:** IMU-Based Motion and Terrain Monitoring  ·  **Student ID:** 2303557  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

**Method** is one of:
- **auto**: a `TEST_CHECK` in `subsystem4_test.c`, printed on the console.
- **host**: a unit test on a PC (`testing/subsystem4/host/`, run `make`).
- **manual**: a person observes the result.

### Host unit tests (`cd testing/subsystem4/host && make`)

The logic is driven by a rigid-car model of our car and humps (`hump_model.c`):
- wheelbase 8 / 8.45 cm, IMU 1.75 cm behind and 9.2 cm above the rear axle;
- big hump 2.3 × 19.75 cm and small hump 1.0 × 19.25 cm;
- the LSM303D's 50 Hz anti-alias filter (CTRL2 ABW = 11), 400 Hz sampling, 0.122 mg steps and the ±4 g clip.

| ID | What is tested | Requirement / interface | Method | Expected result |
|---|---|---|---|---|
| S4-H01 | integer square root | NFR20 (no FPU, no libm) | host | exact |
| S4-H02 | integer atan2, wrap, rounding | FR21 | host | error ≤ 0.01° |
| S4-H03 | axis mapping, accelerometer scale (8197 counts/g) with the `imu_params.h` zero-g offsets and per-axis scale, magnetometer 0.080 mG/LSB and offsets, ±2 gauss limit | FR21 | host | as stated (1 g within 0.5 mg) |
| S4-H04 | at-rest self-test: tilted board aligned to 1 g; moving, not-1 g, too-few rejected | FR20 | host | as stated |
| S4-H05 | static pitch 10°, roll −5° | FR21 | host | ±0.2° |
| S4-H06 | heading N/E/S/W level, and 45° pitched 10° rolled 8° | FR21 | host | ±1.5° |
| S4-H07 | turn rate −90°/s → Turning; at rest → Stationary, no events | FR21, FR24 | host | ±10 %, classes right |
| S4-H08 | hump height, no noise: both humps × both wheelbases × 10/20/30 cm/s, starting from rest | FR22, NFR15 | host | 1 hump, no aborted climb, within ±20 % or ±5 mm |
| S4-H09 | as H08 with 5 mg noise per sample, 2° mounting tilt, 3 mg residual offset, 6 seeds each (10 mg row printed as INFO) | FR22, NFR15, NFR16 | host | every measured hump within NFR15; ≤ 1 of 6 missed |
| S4-H10 | detection, 10 passes per hump at 12–30 cm/s, 5 mg noise | NFR16 | host | ≥ 9 of 10 |
| S4-H11 | 5 m+ of flat track with 0.1–0.15 g starts, stops, speed changes | NFR16, FR24, FR25 | host | 0 humps, no Climbing/Descending class, 0 collisions |
| S4-H12 | Accelerating / Moving straight reported on that run | FR24, NFR18 | host | both seen, ≤ 12 events |
| S4-H13 | Climbing → Descending → Moving straight over a hump | FR24, NFR18 | host | in order |
| S4-H14 | 2 g spike, motors on → collision + Sudden impact at once; 500 ms ignore; none with motors off | FR25, NFR17, NFR18 | host | 2 collisions, impact same cycle |
| S4-H15 | level-check fallback: spike then level → collision; spike then pitch rising → none | FR25 fallback | host | as stated |
| S4-H16 | tip-over: 65° for 0.8 s → 1 event; 70° roll for 0.4 s → none | FR26 | host | as stated |
| S4-H17 | highest peak resets at run start, last peak kept | FR23 | host | as stated |
| S4-H18 | hump height with an uncorrected zero-g offset (10 cm/s, worst case) | FR20, NFR15 | host | within NFR15 up to 10 mg |
| S4-H19 | hump kicks vs bumps for each anti-alias setting (INFO); at 50 Hz with the level check: no false collision over the humps, a hard bump still caught | FR25 | host | 0 false collisions, 5 ms bump caught |
| S4-H20 | second board's mounting: gravity 15° nose-down and 6° rolled, zero-g offsets 3 mg, 5 mg noise, level check on; both humps at 10/20/30 cm/s | FR20, FR22, NFR15 | host | level after calibration within ±0.1°; every hump within NFR15; no collision |
| S4-H21 | INFO: the same with the second board's fitted per-axis gains (S4-M01b: X 0.995, Y 0.969, Z 0.969) left uncorrected | FR22, NFR15 | host | shows why `IMU_ACC_SCALE_*_PERMIL` is needed (small hump missed at 10 cm/s) |

### On-target tests (`make TEST=1`, GY-511 wired, car still and level)

| ID | What is tested | Requirement / interface | Method | Expected result |
|---|---|---|---|---|
| S4-U01 | IMU task runs at 100 Hz | NFR14 | auto | 90–110 cycles in 1 s |
| S4-U02 | integer maths on the Cortex-M0+ | — | auto | PASS |
| S4-U03 | logic on target reads a 10° tilt | FR21 | auto | PASS |
| S4-U04 | self-test ends in a defined state | FR20 | auto | within 8 s |
| S4-U05 | GY-511 passes self-test, ready sent | FR20, IF-01 | auto | PASS (FAIL without the GY-511) |
| S4-U06 | `s4_get_tlm()` status matches the task | IF-07 | auto | PASS |
| S4-U07 | 2 s of reads with no failure | A6, FR27 | auto | PASS |
| S4-U08 | processing time per cycle | NFR20 | auto | < 1000 µs |
| S4-U09 | I2C time per cycle (elapsed, includes pre-emption) | A6 | auto | < 2000 µs |
| S4-U10 | no 10 ms cycle missed | NFR14 | auto | PASS |
| S4-U11 | level and Stationary on the bench | FR21, FR24 | auto | ≤ 2°, Stationary |

### Manual tests

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-M01 | Stage 2 bring-up: bus scan, WHO_AM_I, settings, raw data, FIFO burst check, tilt test | FR20, A6 | manual | `s4_stage2_lsm303d.uf2`, tilt sequence | one LSM303D at 0x1D or 0x1E (the same every round), WHO_AM_I `49`, settings match, ~4 samples/10 ms, 0 fails |
| S4-M01b | Stage 2 repeat on the second board (0x1E) for task C: tilts, six positions in the air, two slow anticlockwise turns | FR20, FR21 | manual | `s4_stage2_lsm303d.uf2`, production settings; away from metal, phones and magnets; keep logging until the second turn ends | each axis seen at +1 g and −1 g; 400+ `acc mg` lines; magnetometer readings well below ±2 gauss |
| S4-M11 | accelerometer diagnosis: seven set-ups (defaults after BOOT, ±2/4/16 g, 773/50 Hz filter, FIFO on/off, self-test) | FR20 | manual | `s4_accel_diag.uf2`; car flat 2 rounds, upside down 2, nose up 2 | a working set-up shows \|a\| ≈ 1000 mg with sd of a few mg, the sign flips upside down, and the self-test moves the axes by 70–1700 mg |
| S4-M02 | bench readout: signs and angles | FR21 | manual | `s4_bench_readout.uf2`; tilt nose up/down, left/right up against a protractor; face N/E/S/W against a phone compass; turn slowly | signs right, ±2°, heading ±10°, turn rate sign right |
| S4-M03 | GY-511 unplugged at boot / during a run | FR27 | manual | default build; unplug and re-plug | ready still sent; FAULT, car keeps going; recovers within 1 s of re-plugging |
| S4-M04 | hump height on the hump track, 10 passes per hump, pushed and driven | FR22, NFR15, NFR16 | manual | `s4_bench_readout.uf2` | ≥ 9/10 detected; each within ±20 % or ±5 mm |
| S4-M05 | collision threshold: hump kicks vs a gentle bump | FR25 | manual | bench readout `pk` and HUMP lines | threshold set between the two |
| S4-M06 | 5 m of flat track | NFR16 | manual | bench readout | no HUMP, no COLLISION |
| S4-M07 | A5 heading with the motors loaded; full-circle magnetometer calibration; magnetometer range | A5, FR21 | manual | car on the course floor, motors running; `tools/accfit.py` on the log | 90/180/360° turns within ±10°; every magnetometer axis ≤ about 1,600 mG ("S4-M07 range check ... PASS"). Above it: switch to ±4 gauss and repeat |
| S4-M08 | A6: 10 minutes of reads with both motors running | A6 | manual | needs Buddy 2's motor code | 0 failed reads |
| S4-M09 | tip-over by hand past 60° with motors flagged on | FR26 | manual | bench image | one TIP-OVER line after 0.5 s |
| S4-M10 | telemetry over MQTT (with Buddy 1) | IF-07, IF-08 | manual | MQTT build, base station | IMU fields and HUMP / CLASS / IMU_FAULT events arrive |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-12 | S4-M01b | uncommitted (S4 image of 2026-11-05) | Buddy 4 | PASS | `logs/2026-11-12_s4-m01b_stage2.txt`, fit: `logs/2026-11-12_s4-m01b_accfit.txt` | Second GY-511 at 0x1E in every line, production settings in all 81 read-backs, 0 failed reads, 811 lines. Every axis held near ±1 g. All 15 holds tilt the labelled way; (+X, −Y, −Z) is the only one of 24 mappings that fits. Fit (sensor axes): zero-g X +78.0, Y +98.9, Z +75.1 mg (±1.6); scale X 995, Y 969, Z 969 ‰ (±1.7); residual 1.6 mg RMS; \|a\| at rest after correction 999–1000 mg; board rests 13.8° nose-down. Z is 3.1 % low, not 7 %. Magnetometer: largest axis 504 mG, ±2 gauss kept; field not uniform (sphere fit 40 mG RMS), so offsets are provisional, from the two flat turns (anticlockwise, −714°). FIFO burst wrapped in 67 of 67 checks; 15 checks skipped (only 3 samples waiting). The capture started about 57 s after boot: t = 9–57 s missing. |
