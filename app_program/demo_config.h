/* Explicit, bounded encoder-free demonstration profile. */
#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H
#ifndef ENCODER_FREE_DEMO
#define ENCODER_FREE_DEMO 0
#endif
#ifndef DEMO_IGNORE_IMU
#define DEMO_IGNORE_IMU 0
#endif
#if DEMO_IGNORE_IMU && !ENCODER_FREE_DEMO
#error "DEMO_IGNORE_IMU requires ENCODER_FREE_DEMO"
#endif
#ifndef DEMO_BASE_DUTY
#define DEMO_BASE_DUTY 14
#endif
#ifndef DEMO_BARCODE_BENCH
#define DEMO_BARCODE_BENCH 0
#endif
#if DEMO_BARCODE_BENCH && !ENCODER_FREE_DEMO
#error "Barcode bench requires demo build"
#endif
/* Timed barcode drive test: START drives straight at DEMO_BASE_DUTY for this
 * many ms, then the car posts its own STOP. Decodes are logged only and never
 * turn the car. 0 disables the test. */
#ifndef DEMO_BARCODE_DRIVE_MS
#define DEMO_BARCODE_DRIVE_MS 0u
#endif
#ifndef DEMO_BARCODE_CLEAR_MS
#define DEMO_BARCODE_CLEAR_MS 600u
#endif
/* IR sample period. 0 keeps the original tk_dly_tsk(10) loop, which the
 * 10 ms kernel tick stretches to 20 ms. Nonzero wakes the IR task from a
 * cyclic handler every DEMO_IR_PERIOD_MS (a multiple of the 10 ms tick). */
#ifndef DEMO_IR_PERIOD_MS
#define DEMO_IR_PERIOD_MS 10u
#endif
#if DEMO_BARCODE_DRIVE_MS && (DEMO_BARCODE_BENCH || !ENCODER_FREE_DEMO)
#error "Barcode drive test requires a demo build without the bench"
#endif
#ifndef DEMO_JUNCTION_TEST
#define DEMO_JUNCTION_TEST 0
#endif
#if DEMO_JUNCTION_TEST && (!ENCODER_FREE_DEMO || DEMO_BARCODE_BENCH || DEMO_BARCODE_DRIVE_MS)
#error "Junction test requires an exclusive driving demo profile"
#endif
#define DEMO_MAX_DUTY 30
#if DEMO_BASE_DUTY < 1 || DEMO_BASE_DUTY > DEMO_MAX_DUTY
#error "Demo forward duty must be between 1 and DEMO_MAX_DUTY"
#endif
#if DEMO_IR_PERIOD_MS && ((DEMO_IR_PERIOD_MS % 10u) != 0u)
#error "IR period must be zero or a multiple of the 10 ms kernel tick"
#endif
#define DEMO_LEFT_TRIM 0
#define DEMO_RIGHT_TRIM 0
#define DEMO_TURN_DUTY 20
#define DEMO_SEARCH_DUTY 18
#define DEMO_LINE_CORRECTION 50
#define DEMO_SEARCH_FIRST_MS 250u
#define DEMO_STALE_MS 60u
#define DEMO_LOSS_MS 100u
#define DEMO_FOUND_MS 100u
#define DEMO_JUNCTION_MS 100u
#define DEMO_CROSS_MAX_MS 1000u
#define DEMO_RECOVERY_MS 3000u
#define DEMO_PRETURN_MS 250u
#define DEMO_TURN_MIN_MS 300u
#define DEMO_UTURN_MIN_MS 750u
#define DEMO_TURN_MAX_MS 3000u
#define DEMO_BYPASS_SPIN_MS 450u
#define DEMO_BYPASS_SIDE_MS 1000u
#define DEMO_BYPASS_PASS_MS 1500u
#define DEMO_RUN_MAX_MS 120000u
/* User measurements, 2026-11-21; references for validation, not generated
 * sensor results. Buddy 4's estimator uses acceleration and its lever arm. */
#define DEMO_WHEELBASE_MM 80u
#define DEMO_MEASURED_TRACK_MM 264u
#define DEMO_MEASURED_WHEEL_MM 65u
#define DEMO_MEASURED_LINE_MM 18u
#define DEMO_PICKUP_TILT_DEG 45
#define DEMO_PICKUP_HOLD_MS 150u
#endif
