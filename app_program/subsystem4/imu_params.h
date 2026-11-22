/*
 *  imu_params.h
 *  Every Subsystem 4 value that needs the real car, in one place.
 *
 *  Each value is a best estimate until it is measured.  The tag in each
 *  comment says what the measurement waits on, and how it is measured:
 *
 *    [BENCH]   Buddy 4 on the bench: the car still or tilted by hand
 *    [MOTORS]  Buddy 2's motors running (speed loop, acceleration ramps)
 *    [HUMPS]   driving over the hump track (big 2.3 cm, small 1.0 cm)
 *    [COURSE]  a full run on the course
 *
 *  Units: accelerations in mg, angles in centidegrees (cdeg, 0.01 deg),
 *  lengths in micrometres (um), times in milliseconds.
 *
 *  This header has no kernel dependencies (host tests include it).
 */

#ifndef IMU_PARAMS_H
#define IMU_PARAMS_H

/* =====================================================================
 *  Axis mapping, sensor -> car body.  [BENCH] Stage 2 tilt test.
 *
 *  Body frame: x forward, y left, z up (right-handed).  Each body axis
 *  takes one sensor axis (0 = X, 1 = Y, 2 = Z) times a sign.
 *
 *  Found 2026-11-04 from the Stage 2 magnetometer data (tilts, six held
 *  positions, turn): body = (+X, -Y, -Z), the chip facing down.  Of the
 *  24 possible mappings only this one gives the same vertical field in
 *  all 12 holds (spread 22 mG; next best 86 mG), and heading falls in the
 *  anticlockwise turn.  The LSM303D's accelerometer and magnetometer axes
 *  point the same way (datasheet Figure 2), so the accelerometer takes
 *  the same mapping.
 *  Confirmed on the second board's accelerometer 2026-11-12 (S4-M01b):
 *  all 15 labelled holds (tilts, six positions) tilt the labelled way,
 *  and of the 24 right-handed mappings only this one fits every label.
 * ===================================================================== */
#define IMU_ACC_BX_AXIS         0
#define IMU_ACC_BX_SIGN         (1)
#define IMU_ACC_BY_AXIS         1
#define IMU_ACC_BY_SIGN         (-1)
#define IMU_ACC_BZ_AXIS         2
#define IMU_ACC_BZ_SIGN         (-1)

/* Magnetometer axes: same as the accelerometer (see above). */
#define IMU_MAG_BX_AXIS         0
#define IMU_MAG_BX_SIGN         (1)
#define IMU_MAG_BY_AXIS         1
#define IMU_MAG_BY_SIGN         (-1)
#define IMU_MAG_BZ_AXIS         2
#define IMU_MAG_BZ_SIGN         (-1)

/* Accelerometer zero-g offsets, sensor axes, mg (datasheet: typically
   +-60 mg; the 50 Hz anti-alias setting adds about +50-60 mg on every
   axis of the second board, so fit them with the production settings).
   Fitted 2026-11-12 from S4-M01b (second board, production settings,
   still windows only): the offsets and per-axis scales that make
   |a| = 1 g in all 10 held orientations (tilts and six positions).
   Residual 1.6 mg RMS (1.1 mg over 633 still lines), each value
   +-1.6 mg (1 sigma).  testing/subsystem4/tools/accfit.py.
   The self-test then takes the tilt left at rest (13.8 deg nose-down
   on the second board) out as a rotation (imu_cal_finish). */
#define IMU_ACC_ZG_X_MG         78
#define IMU_ACC_ZG_Y_MG         99
#define IMU_ACC_ZG_Z_MG         75

/* Accelerometer scale: raw 16-bit counts per 1 g.  LSM303D datasheet
   Table 3: 0.122 mg/LSB at +-4 g -> 8197 per g (typical, not
   guaranteed). */
#define IMU_ACC_RAW_PER_G       8197

/* Per-axis scale correction, sensor axes, per mille (1000 = datasheet):
   counts per g on each axis / 8197 x 1000.  An uncorrected gain
   mismatch between axes adds a vertical error that changes with pitch,
   like a zero-g error.
   Same fit as the offsets above, each value +-1.7 (1 sigma): Y and Z
   read 3.1 % low, X 0.5 % low.  (The diagnosis log's "Z 7 % low" came
   from ignoring the offsets and the 19 deg lean of its upside-down
   hold.) */
#define IMU_ACC_SCALE_X_PERMIL  995
#define IMU_ACC_SCALE_Y_PERMIL  969
#define IMU_ACC_SCALE_Z_PERMIL  969

/* =====================================================================
 *  Magnetometer calibration (one-off full circle, design review 6.4).
 *  Body frame, milligauss.
 *
 *  PROVISIONAL, second board, motors off: S4-M01b, 2026-11-12
 *  (testing/subsystem4/tools/accfit.py).  The field at the sensor was
 *  not uniform in that log: a sphere fit to every orientation leaves
 *  40 mG RMS, and the vertical field differs by up to 410 mG between
 *  holds (a uniform field gives a few mG).  So the horizontal offsets
 *  come from the two flat turns instead (circle in the level plane,
 *  17 mG RMS; the first turn alone 5.8 mG), which is what heading on
 *  flat ground uses, and the vertical one from the sphere fit.
 *  Sensor-frame centre X -69.7, Y -241.5, Z +94.2 mG; horizontal field
 *  250 mG.  On the turns this keeps heading within about 11 deg (the
 *  sphere centre: 21 deg).
 *  [MOTORS] final values from S4-M07 (A5): car on the floor where it
 *  runs, motors loaded, slow 360 deg turns plus tilts; same script.
 *  Scale: the turns trace an ellipse of axis ratio 0.92-0.97, worth at
 *  most about 2 deg of heading, so it stays 1000 until S4-M07.
 * ===================================================================== */
#define IMU_MAG_OFF_BX_MG       (-70)
#define IMU_MAG_OFF_BY_MG       242
#define IMU_MAG_OFF_BZ_MG       (-94)
#define IMU_MAG_SCALE_BX_PERMIL 1000    /* x span -> mean span           */
#define IMU_MAG_SCALE_BY_PERMIL 1000

/* =====================================================================
 *  Self-test at rest (FR20).  [BENCH] Stage 2 log, car still on the
 *  table: |a| and the spread of the 10 ms means over 1 s.
 * ===================================================================== */
#define IMU_ST_G_TOL_MG         150     /* |a| must be 1 g +- this       */
#define IMU_ST_STILL_MG         40      /* max - min of 10 ms means      */

/* =====================================================================
 *  Geometry: IMU chip relative to the rear axle (hub centre).  Measured
 *  by hand 2026-11-04 (1.75 cm behind, 9.2 cm above).  [BENCH] re-check
 *  the 1.75 cm: castor-to-chip 10.2 cm minus wheelbase 8 cm gives 2.2.
 *  A 4.5 mm error moves the big-hump peak by about 1 mm.
 * ===================================================================== */
#define IMU_LEVER_BACK_UM       17500
#define IMU_LEVER_UP_UM         92000

/* =====================================================================
 *  Gravity direction filter: time constant of the low-pass that gives
 *  tilt and "down".  Shift k: each 10 ms cycle moves 1/2^k of the way.
 *  k = 2 -> about 35 ms.  Tuned on the host hump model; [HUMPS] check
 *  that the peak stays within NFR15 at the real speed.
 * ===================================================================== */
#define IMU_GRAV_LPF_SHIFT      2

/* =====================================================================
 *  Hump detection (FR22, design review 6.4).
 * ===================================================================== */
/* Climb / descend pitch thresholds.  Design review.  [HUMPS] the small
   hump only reaches 5.9 deg, so keep these well below that. */
#define IMU_HUMP_UP_CDEG        300
#define IMU_HUMP_DOWN_CDEG      (-300)

/* "Level": |pitch| below this.  Used for the end of a hump and for the
   flat reference before it.  [HUMPS] above the pitch noise when
   driving on flat track. */
#define IMU_HUMP_LEVEL_CDEG     150

/* Tighter level band that ends a hump: the rear axle is then within
   about 1 mm of the floor (wheelbase x sin 0.75 deg).  [HUMPS] pitch
   noise after the hump must stay inside it. */
#define IMU_HUMP_END_CDEG       75

/* "Rising": the IMU itself must rise at least this much in
   IMU_HUMP_RISE_MS.  Small hump at speed v rises v x sin(5.9 deg) x
   50 ms: 0.5 mm at 10 cm/s.  [MOTORS] needs Buddy 2's navigation speed;
   [HUMPS] check on the small hump at that speed. */
#define IMU_HUMP_RISE_UM        300
#define IMU_HUMP_RISE_MS        50      /* design review                 */

/* "Dipped first": while only the castor climbs, the IMU (behind and
   above the rear axle) drops by 1.75 sin(pitch) + 9.2 (1 - cos(pitch))
   cm: 2.3 mm on the small hump, 6.4 mm on the big one.  Forward
   acceleration never makes it dip, so this keeps starts and stops from
   looking like humps.  Kept small because the integrated height wanders
   with sensor noise: at 10 mg per sample and 10 cm/s the small hump's dip
   can be hidden (host test S4-H09 10 mg row).  0 turns the check off
   (then starts and stops may briefly show "Climbing hump").
   [BENCH]+[MOTORS] noise ("vib") while driving; [HUMPS] small hump at the
   real speed. */
#define IMU_HUMP_DIP_UM         100

#define IMU_HUMP_LEVEL_MS       100     /* level this long ends a hump   */
#define IMU_HUMP_CLIMB_MS       3000    /* no descent: nothing recorded  */
#define IMU_HUMP_DESC_MS        3000    /* no level: nothing recorded    */

/* Smallest peak reported; anything lower is treated as noise (NFR16).
   [HUMPS] 5 m of flat track must give no hump. */
#define IMU_HUMP_MIN_MM         4

/* =====================================================================
 *  Motion class (FR24, NFR18).  Thresholds from the design review, with
 *  hysteresis: a condition turns on above *_ON and off below *_OFF.
 * ===================================================================== */
/* Vibration (mean |change| between 400 Hz samples) below which the car
   counts as still.  [BENCH] at rest vs [MOTORS] at the slowest speed. */
#define IMU_STILL_VIB_ON_MG     8
#define IMU_STILL_VIB_OFF_MG    12

#define IMU_STILL_RATE_ON_CDPS  500     /* turn rate < 5 deg/s           */
#define IMU_STILL_RATE_OFF_CDPS 800

/* [MOTORS] Buddy 2's turns: must exceed 30 deg/s while turning. */
#define IMU_TURN_ON_CDPS        3000
#define IMU_TURN_OFF_CDPS       2000

/* [MOTORS] Buddy 2's start and stop ramps vs steady speed. */
#define IMU_ACCEL_ON_MG         50
#define IMU_ACCEL_OFF_MG        35

#define IMU_CLASS_HOLD_MS       100     /* design review NFR18           */

/* Turn-rate window: least-squares slope of the heading over this time.
   LSM303D magnetometer noise is ~5 mG RMS (Table 3) -> ~0.7 deg per
   sample -> ~2 deg/s of rate noise over 300 ms.  Longer = less noise,
   slower response.  [BENCH] rate noise at rest; [MOTORS] response. */
#define IMU_TURN_WINDOW_MS      300

/* =====================================================================
 *  Collision (FR25).  [HUMPS] + gentle bump: log the per-cycle peak
 *  ("pk" in the bench readout) over both humps and in a gentle push
 *  into an obstacle, then set this between the two.
 * ===================================================================== */
#define IMU_COLLISION_MG        1500    /* design review                 */
#define IMU_COLLISION_IGNORE_MS 500     /* design review                 */

/* Fallback (S4_CFG_COLLISION_LEVEL_CHECK): the spike counts only if
   pitch changes by less than this over the next CONFIRM_MS.  [HUMPS]
   pitch change 30 ms after a hump kick vs after a bump. */
#define IMU_COLLISION_CONFIRM_MS    30
#define IMU_COLLISION_LEVEL_CDEG    50

/* =====================================================================
 *  Tip-over (FR26).  Design review.  [BENCH] tilt the car by hand past
 *  60 deg with the motors flag forced on.
 * ===================================================================== */
#define IMU_TIP_CDEG            6000
#define IMU_TIP_MS              500

#endif /* IMU_PARAMS_H */
