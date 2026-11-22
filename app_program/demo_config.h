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
#define DEMO_BASE_DUTY 20
#define DEMO_MAX_DUTY 30
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
