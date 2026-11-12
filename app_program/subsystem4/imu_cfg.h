/*
 *  imu_cfg.h
 *  Subsystem 4 build switches.  Change a value here and rebuild; nothing
 *  else needs editing.  The defaults are the safe settings for every
 *  teammate's build.  Values that need the real car live in
 *  imu_params.h instead.
 *
 *  (The makefile cannot pass these in: "make CFLAGS+=-D..." replaces the
 *  makefile's own CFLAGS, cpu flags included.)
 */

#ifndef IMU_CFG_H
#define IMU_CFG_H

/* ---------------------------------------------------------------------
 *  Report-only mode, until the thresholds are measured on the car.
 *
 *    1 (default)  Collision and tip-over are reported (telemetry, motion
 *                 class, console) but no fault goes to the State Machine,
 *                 so the run does not stop.  A missing or failed GY-511
 *                 never sends SM_EVT_SELFTEST_FAIL: the task reports
 *                 ready, marks its data invalid and keeps retrying.
 *    0 (demo)     Design review behaviour: collision and tip-over send
 *                 SM_EVT_FAULT (the State Machine enters ERROR), and a
 *                 self-test that fails three times during INITIALISING
 *                 sends SM_EVT_SELFTEST_FAIL (FR20, FR25, FR26).
 *
 *  Losing the IMU during a run never stops the car in either mode (FR27).
 * --------------------------------------------------------------------- */
#define S4_CFG_REPORT_ONLY      1

/* ---------------------------------------------------------------------
 *  Console readout (USB serial).
 *
 *    S4_CONSOLE_OFF    nothing (default)
 *    S4_CONSOLE_BENCH  bench readout without MQTT: pitch, roll, heading,
 *                      turn rate, motion class and IMU status 4 times a
 *                      second, plus one line per hump, collision,
 *                      tip-over and IMU fault
 *    S4_CONSOLE_RAW    Stage 2 bring-up: bus scan and chip IDs every 5 s,
 *                      raw accelerometer and magnetometer twice a second,
 *                      running count of failed reads.  The IMU task does
 *                      nothing else in this mode (no events, no
 *                      self-test, no State Machine interaction).
 *    S4_CONSOLE_ACCDIAG  accelerometer diagnosis: cycles through several
 *                      accelerometer set-ups (datasheet defaults, after a
 *                      factory-trim reload, ours, self-test on) and prints
 *                      the statistics of each.  Nothing else runs.
 * --------------------------------------------------------------------- */
#define S4_CONSOLE_OFF          0
#define S4_CONSOLE_BENCH        1
#define S4_CONSOLE_RAW          2
#define S4_CONSOLE_ACCDIAG      3

#define S4_CFG_CONSOLE          S4_CONSOLE_OFF

/* ---------------------------------------------------------------------
 *  Collision fallback (design review FR25 plus the agreed fallback).
 *    0 (default)  a horizontal spike above IMU_COLLISION_MG outside a
 *                 hump is a collision at once
 *    1            the spike counts only if pitch stays level for
 *                 IMU_COLLISION_CONFIRM_MS afterwards; a hump kick
 *                 changes pitch, a collision does not.  Still inside
 *                 NFR17's 50 ms.
 * --------------------------------------------------------------------- */
#define S4_CFG_COLLISION_LEVEL_CHECK    0

/* ---------------------------------------------------------------------
 *  Bench only: act as if SM_FLG_MOTORS_ENABLED were set whenever
 *  SM_FLG_SENSORS_ENABLED is.  Without MQTT nothing can start a run, so
 *  this is the only way to exercise collision, tip-over and the run's
 *  highest peak on the bench.  Never set it in a car build.
 * --------------------------------------------------------------------- */
#define S4_CFG_BENCH_MOTORS     0

/* ---------------------------------------------------------------------
 *  I2C bus speed.  0 = 400 kHz (default), 1 = 100 kHz fallback for long
 *  or noisy wiring.  At 100 kHz a 10 ms cycle spends about 4 ms on the
 *  bus instead of 1 ms.
 * --------------------------------------------------------------------- */
#define S4_CFG_I2C_100KHZ       0

/* ---------------------------------------------------------------------
 *  Accelerometer FIFO read.
 *    1 (default)  all samples in one burst per cycle.  This needs the
 *                 output pointer to wrap from 2Dh back to 28h, which the
 *                 datasheet does not state; the Stage 2 log of 2026-11-04
 *                 shows it does (70 of 70 checks: the FIFO level fell by
 *                 4, or by 3 when a sample arrived during the read).
 *    0            one 6-byte read per sample (each read of OUT_*_A pops
 *                 the oldest sample, datasheet section 4.3); about 0.3 ms
 *                 more bus time per cycle (Stage 2: 1.3 ms median,
 *                 1.8 ms 95th percentile per cycle).
 * --------------------------------------------------------------------- */
#define S4_CFG_FIFO_BURST       1

#endif /* IMU_CFG_H */
