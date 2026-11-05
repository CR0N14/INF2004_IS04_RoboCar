/*
 *  motion_config.h
 *  Every pin and tunable value for Subsystem 2 -- Motion Control.
 *
 *  Change values HERE, not inside the .c files.  Each value says where it
 *  came from, so it can be defended at the tribunal and updated after
 *  measuring.  Values marked TODO are placeholders that MUST be measured
 *  before the car can drive closed-loop (subsystem2_test.c test S2-U09
 *  fails until they are filled in).
 *
 *  Ported from the stand-alone bench kit in z_motion_control/ (Pico SDK,
 *  no RTOS).  The bench kit is still the right tool for bring-up steps
 *  1-3 (motor direction, counts per rev, duty sweep).
 *
 *  Owner: S2 (Motion Control), student 2303576.
 */

#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ------------------------------------------------------------------------- */
/* Which wheel.  Used by every S2 file so they all index wheels the same way. */
/* ------------------------------------------------------------------------- */
typedef enum
{
    WHEEL_LEFT  = 0,
    WHEEL_RIGHT = 1,
    WHEEL_COUNT = 2
} wheel_t;

/* ------------------------------------------------------------------------- */
/* Motor driver pins (Cytron Robo Pico on-board driver).                      */
/*                                                                           */
/* Named by what they DO on our car rather than the board label, so no       */
/* "invert" flag is needed.  Source: team bench check, 2026-11-04:           */
/*   GP8  = left forward,  GP9  = left backward      (board M1A / M1B)       */
/*   GP11 = right forward, GP10 = right backward     (board M2B / M2A)       */
/* The right motor is mounted mirror-image, so its "forward" pin is M2B.     */
/*                                                                           */
/* PWM: GP8/9 are PWM slice 4 (A/B), GP10/11 are slice 5 (A/B).  The servo   */
/* on GP12 uses slice 6, so nothing clashes (TEAM.md section 3).             */
/* ------------------------------------------------------------------------- */
#define MOTOR_L_PIN_FWD     8u
#define MOTOR_L_PIN_REV     9u
#define MOTOR_R_PIN_FWD     11u
#define MOTOR_R_PIN_REV     10u

/* PWM frequency.  Source: NFR9: fixed at the driver's 20 kHz maximum, above
 * human hearing, so no motor whine. */
#define PWM_FREQ_HZ         20000u

/* ------------------------------------------------------------------------- */
/* Encoder pins (two-channel A/B hall encoders).                              */
/* Source: Week 6 pin table / TEAM.md: left on Grove 3 (GP4/GP5),             */
/* right on Grove 4 (GP16/GP17).                                              */
/* ------------------------------------------------------------------------- */
#define ENC_L_PIN_A         4u
#define ENC_L_PIN_B         5u
#define ENC_R_PIN_A         16u
#define ENC_R_PIN_B         17u

/* Set to 1 if that wheel's count goes DOWN when the wheel turns forward.
 * Source: TODO: bench kit test_encoder (step 2).  The right motor is
 * mirror-mounted, so expect ENC_R_INVERT = 1, but CHECK it. */
#define ENC_L_INVERT        0
#define ENC_R_INVERT        0

/* Internal pull-ups on the encoder pins.  Hall encoders are often
 * open-collector; 1 is harmless if the encoder board has its own. */
#define ENC_USE_PULLUPS     1

/* ------------------------------------------------------------------------- */
/* Measured geometry.  0 = not measured yet: the car refuses to drive.         */
/* ------------------------------------------------------------------------- */

/* Encoder counts for ONE wheel turn with x4 decoding (every edge of A and B).
 * Source: TODO: bench kit test_encoder: turn a wheel 10 times by hand,
 * count / 10. */
#define ENC_COUNTS_PER_REV  0

/* Wheel diameter.  Source: team measurement 2026-11-04, "wheel height about
 * 6.5 cm" (a wheel's height standing up = its diameter).  For NFR8 (+-5 %
 * over 1 m) check it: roll the car 10 wheel turns and measure the distance;
 * diameter = distance / (10 * pi). */
#define WHEEL_DIAMETER_MM   65

/* Distance between the centres of the two tyres (where they touch the
 * floor).  Used to turn a spin angle into wheel travel.
 * Source: TODO: measure with a ruler. */
#define TRACK_WIDTH_MM      0

/* ------------------------------------------------------------------------- */
/* Control loop timing.                                                       */
/* ------------------------------------------------------------------------- */

/* Control period.  Source: NFR5.  Must be a whole number of kernel ticks:
 * the kernel tick is CNF_TIMER_PERIOD = 10 ms (config/config.h), so 10 ms is
 * also the fastest this loop can run. */
#define MOTION_PERIOD_MS    10u

/* Speed is measured over the last N control periods (here 5 x 10 ms = 50 ms).
 * One period alone holds very few encoder counts at low speed, so the
 * measurement jumps in big steps; a longer window is smoother but reacts
 * later.  Source: design choice; revisit after the step 3 sweep. */
#define SPEED_WINDOW_TICKS  5u

/* ------------------------------------------------------------------------- */
/* Speed controller (one PI(D) loop per wheel).  Units: duty % and mm/s.      */
/* All STARTING GUESSES, not measured: tune in bring-up step 4.               */
/* ------------------------------------------------------------------------- */

/* Feed-forward: what duty roughly gives a wanted speed, from the step 3 duty
 * sweep (wheels on the floor).  duty_ff = START + (100 - START) * v / TOP.
 *   MOTOR_START_DUTY_PCT  lowest duty at which the wheel keeps turning
 *   MOTOR_TOP_SPEED_MMPS  speed at 100 % duty
 * Source: TODO: step 3 sweep.  TOP = 0 switches feed-forward off (pure PI). */
#define MOTOR_START_DUTY_PCT    0
#define MOTOR_TOP_SPEED_MMPS    0

#define PID_KP      0.05f   /* duty % per (mm/s) of error                    */
#define PID_KI      0.20f   /* duty % per mm of accumulated error            */
#define PID_KD      0.0f    /* duty % per (mm/s^2); start with 0             */
#define PID_I_MAX   60.0f   /* integral term limit, duty % (anti-windup)     */

/* Below this measured speed a wheel counts as "stopped".  Source: guess. */
#define STOPPED_SPEED_MMPS  5

/* ------------------------------------------------------------------------- */
/* Driving speeds (mm/s).  Starting guesses: tune on the track.               */
/* Telemetry can show up to 1023 mm/s (s2_tlm_t speed fields are 10 bits).    */
/* ------------------------------------------------------------------------- */
#define CRUISE_SPEED_MMPS   150     /* line following                        */
#define TURN_SPEED_MMPS     100     /* each wheel during a spin turn         */
#define AVOID_SPEED_MMPS    120     /* obstacle bypass                       */
#define RECOVERY_SPEED_MMPS 60      /* creeping while searching for the line */

/* ------------------------------------------------------------------------- */
/* Stopping (FR12, FR14, NFR11).                                              */
/* ------------------------------------------------------------------------- */

/* Normal stop (MOTORS_ENABLED cleared, not ERROR): the speed target drops by
 * this much each period, i.e. 10 mm/s per 10 ms = 1 m/s^2.  ERROR always
 * brakes at once, so it is never slower than a normal stop (NFR11). */
#define STOP_DECEL_MMPS_PER_TICK    10

/* A normal stop that takes longer than this brakes anyway.  Source: guess. */
#define STOP_TIMEOUT_MS             1000u

/* ------------------------------------------------------------------------- */
/* Stall detection.  Source: FR10: duty > 20 % AND no encoder pulse for       */
/* 300 ms -> stall fault.                                                     */
/* ------------------------------------------------------------------------- */
#define STALL_DUTY_MIN_PCT  20
#define STALL_TIME_MS       300u

/* ------------------------------------------------------------------------- */
/* Steering input from S3 Line Following (s2_set_steering()).                 */
/* ------------------------------------------------------------------------- */

/* A steering value older than this is ignored and the car holds still.
 * Source: TODO: agree with S3 (open item).  S3 updates every 10 ms, so
 * 100 ms = 10 missed updates in a row. */
#define STEER_STALE_MS      100u

/* BRING-UP ONLY.  1 = in line following, drive straight at cruise speed
 * when there is no fresh steering (for tuning the speed loop before S3 is
 * ready).  Must be 0 for the real run. */
#define MOTION_BRINGUP_IGNORE_STEERING  0

/* ------------------------------------------------------------------------- */
/* Turns at junctions / barcodes (FR13).  Spin turns: wheels in opposite      */
/* directions, so the car turns on the spot.                                  */
/* ------------------------------------------------------------------------- */

/* Drive this far forward before a left/right spin, so the axle (not the IR
 * sensors in front of it) is over the junction.  Source: TODO: measure the
 * distance from the IR sensors to the axle. */
#define TURN_PRE_DRIVE_MM   0

/* "Straight on" at a junction: drive this far to clear it.  Source: guess. */
#define JUNCTION_CLEAR_MM   50

#define TURN_SIDE_DEG       90      /* left / right turn                     */
#define TURN_UTURN_DEG      180     /* U-turn                                */

/* ------------------------------------------------------------------------- */
/* Obstacle bypass (SM_NAV_CMD_AVOID).  DRAFT: how avoidance moves reach      */
/* motion is an open item with S1/S5.  For LEFT (RIGHT is the mirror image):  */
/*   spin out AVOID_OUT_DEG, drive AVOID_SIDE_MM, spin back 2 x AVOID_OUT_DEG,*/
/*   drive AVOID_SIDE_MM, then report BYPASS_DONE; S3 then looks for the line */
/*   (State Machine nav mode RECOVERY).                                       */
/* REVERSE: back up AVOID_REVERSE_MM, then report BYPASS_DONE.                */
/* Source: guesses; size them for the real obstacle.                          */
/* ------------------------------------------------------------------------- */
#define AVOID_OUT_DEG       45
#define AVOID_SIDE_MM       250
#define AVOID_REVERSE_MM    150

/* ------------------------------------------------------------------------- */
/* Debug output.                                                              */
/* ------------------------------------------------------------------------- */

/* Print counts, speeds and duty to the console every N ms (0 = off).
 * Handy for bring-up step 4.  It costs console time, so leave it off for
 * real runs.  Source: design choice. */
#define MOTION_DEBUG_PRINT_MS   0u

#endif /* MOTION_CONFIG_H */
