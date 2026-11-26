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
#define DEMO_TURN_DUTY DEMO_BASE_DUTY
#define DEMO_SEARCH_DUTY 10
#define DEMO_LINE_CORRECTION 40
#define DEMO_SEARCH_FIRST_MS 350u
#define DEMO_WIDE_SEARCH_DUTY 18
#define DEMO_WIDE_SEARCH_MS 3000u
#define DEMO_STALE_MS 60u
#define DEMO_LOSS_MS 20u
#define DEMO_ARC_MS 300u
#define DEMO_BARCODE_PRIORITY_MS 4000u
#define DEMO_ARC_STEER 33
#define DEMO_REACQUIRE_MS 20u
#define DEMO_FOUND_MS 100u
#define DEMO_JUNCTION_MS 100u
#define DEMO_CROSS_MAX_MS 1000u
#define DEMO_RECOVERY_MS (DEMO_SEARCH_FIRST_MS + DEMO_WIDE_SEARCH_MS)
#define DEMO_PRETURN_MS 250u
#define DEMO_TURN_MIN_MS 300u
#define DEMO_UTURN_MIN_MS 1600u
#define DEMO_TURN_MAX_MS 3000u
/* Obstacle bypass: heading-guided turns (S4 magnetometer, 0..35999 cdeg,
 * clockwise) and sonar-guided legs (S5 servo aimed at the obstacle side).
 * Every value below needs floor calibration; see OBSTACLE_BYPASS.md. */
#define DEMO_SERVO_LEFT_LOW 1          /* 1: servo angles below 90 point left */
#define DEMO_HEADING_SIGN 1            /* +1: heading increases turning right */
#define DEMO_SIDE_CLEAR_CM 40u         /* nearest return beside the obstacle  */
#define DEMO_SONAR_EDGE_CORR_CM 0u     /* per-edge beam-spread correction     */
#define DEMO_BYPASS_TURN_DUTY 16       /* slow spins keep heading samples dense */
#define DEMO_BYPASS_DUTY 16            /* straight legs                       */
#define DEMO_BYPASS_OUT_CDEG 4500      /* turn out, and turn back in, 45 deg  */
#define DEMO_BYPASS_IN_CDEG 4500
#define DEMO_BYPASS_ALIGN_CDEG 300     /* "back on the original heading"      */
#define DEMO_BYPASS_HOLD_GAIN 2        /* % duty per degree of heading error  */
#define DEMO_BYPASS_AIM_OUT_DEG 60u    /* servo offset towards obstacle, out leg */
#define DEMO_BYPASS_AIM_PASS_DEG 90u   /* servo offset towards obstacle, pass leg */
#define DEMO_BYPASS_SEEN_CM 40u        /* obstacle present at or below this   */
#define DEMO_BYPASS_GONE_CM 60u        /* obstacle passed at or above this    */
#define DEMO_BYPASS_GONE_READS 2u      /* consecutive far readings for a jump */
#define DEMO_BYPASS_MIN_READS 5u       /* readings before "never seen" counts */
#define DEMO_BYPASS_UNSEEN_MS 1500u    /* never seen this long: leg is clear  */
#define DEMO_BYPASS_MARGIN_MS 400u     /* past the edge: half the car width   */
#define DEMO_BYPASS_LENGTH_MS 800u     /* past the far end: the car length    */
#define DEMO_BYPASS_TURN_MAX_MS 3000u
#define DEMO_BYPASS_LEG_MAX_MS 5000u
#define DEMO_BYPASS_APPROACH_MAX_MS 4000u
#define DEMO_BYPASS_BLOCK_CM 12u       /* forward return this close: fault    */
#define DEMO_RUN_MAX_MS 120000u
/* Obstacle profile (SONAR_REFINED=1): refined corner scan, demo_measure_width(),
 * then side clearance on a 10-degree grid. 0 keeps the 25-point 5-degree sweep. */
#ifndef DEMO_SONAR_REFINED
#define DEMO_SONAR_REFINED 0
#endif
/* Smooth tracking (SMOOTH_TRACK=1). 0 keeps straight-while-on-line, then a
 * fixed 33% arc and LINE_LOST after 300 ms. Steer is % slowing of the inner
 * wheel; + uses the same sign as the arc after a centre-only loss.
 * Darkness is 0..4095. All values are first guesses for floor tuning. */
#ifndef DEMO_SMOOTH_TRACK
#define DEMO_SMOOTH_TRACK 0
#endif
#ifndef DEMO_TRACK_TARGET
#define DEMO_TRACK_TARGET 3000   /* centre darkness held on the line edge  */
#endif
#ifndef DEMO_TRACK_KP
#define DEMO_TRACK_KP 20         /* % steer per 1000 centre darkness error */
#endif
#ifndef DEMO_TRACK_KL
#define DEMO_TRACK_KL 30         /* % steer per 1000 code-left darkness    */
#endif
#define DEMO_TRACK_LEFT_ON 600u  /* code-left darkness that starts counting */
#ifndef DEMO_TRACK_KD
#define DEMO_TRACK_KD 5          /* tenths of the P change per IR sample   */
#endif
#define DEMO_TRACK_MAX_STEER 70  /* on-line limit; code-left only uses it  */
#define DEMO_TRACK_DIR_MIN 5     /* steer at or below -this latches dir -1 */
#ifndef DEMO_TRACK_MIN_DUTY
#define DEMO_TRACK_MIN_DUTY 14   /* forward duty at the steering limit     */
#endif
#define DEMO_SMOOTH_ARC_MS 700u  /* arc window before LINE_LOST            */
#define DEMO_SMOOTH_ARC_RAMP_MS 400u
#define DEMO_SMOOTH_ARC_MAX_STEER 90 /* inner wheel 10%: still forward     */
#if DEMO_TRACK_MIN_DUTY < 1 || DEMO_TRACK_MIN_DUTY > DEMO_BASE_DUTY
#define DEMO_TRACK_FLOOR DEMO_BASE_DUTY
#else
#define DEMO_TRACK_FLOOR DEMO_TRACK_MIN_DUTY
#endif
/* User measurements, 2026-11-21; references for validation, not generated
 * sensor results. Buddy 4's estimator uses acceleration and its lever arm. */
#define DEMO_WHEELBASE_MM 80u
#define DEMO_MEASURED_TRACK_MM 264u
#define DEMO_MEASURED_WHEEL_MM 65u
#define DEMO_MEASURED_LINE_MM 18u
#define DEMO_PICKUP_TILT_DEG 45
#define DEMO_PICKUP_HOLD_MS 150u
#endif
