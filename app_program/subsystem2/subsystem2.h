/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control.  Student ID: 2303576.
 *
 *	Public interface: only what OTHER subsystems may use.  Everything else
 *	is local to the subsystem2/ files:
 *	  motion_config.h    pins and tunables
 *	  drv_motor.c/.h     PWM motor driver      (Motion Control task only)
 *	  drv_encoder.c/.h   quadrature encoders   (interrupt handler)
 *	  motion_logic.c/.h  PID, speed, steering mix, stall, manoeuvres
 *	  subsystem2.c       Motion Control task, statechart, these functions
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

#define S2_PRI_MAIN	PRI_MOTION_CONTROL	/* app.h priority map */

/* ---------------------------------------------------------------------
 *  IF-07 telemetry snapshot, S2 -> S1 Networking
 *
 *  The latest values only.  s2_get_tlm() copies the snapshot out under
 *  this subsystem's own mutex and returns E_OK; Networking calls it every
 *  200 ms.  Until it is implemented here, subsystem1/net_tlm_default.c
 *  supplies a weak stub that returns E_NOSPT, and the values are left
 *  out of telemetry.
 *
 *  Each field holds only the bits its range needs (Change Requirement 1).
 *  Signed quantities are stored as magnitude + direction bit.  If a field
 *  or range changes, tell S1: subsystem1/net_codec.c packs these fields
 *  for MQTT and docs/MQTT.md documents them.
 * --------------------------------------------------------------------- */

typedef enum
{
    S2_MOTION_DISABLED = 0,
    S2_MOTION_LINE_FOLLOWING,
    S2_MOTION_TURNING,
    S2_MOTION_AVOIDING,
    S2_MOTION_STALL_FAULT
} s2_motion_state_t;

/* 71 bits in 3 words */
typedef struct
{
    unsigned int speed_l_mms    : 10;   /* |left speed|, 0..1023 mm/s    */
    unsigned int b_rev_l        : 1;    /* left wheel turning backwards  */
    unsigned int speed_r_mms    : 10;   /* |right speed|, 0..1023 mm/s   */
    unsigned int b_rev_r        : 1;    /* right wheel turning backwards */
    unsigned int motion_state   : 3;    /* s2_motion_state_t             */
    unsigned int enc_l          : 16;   /* left pulse count, wraps       */
    unsigned int enc_r          : 16;   /* right pulse count, wraps      */
    unsigned int distance_cm    : 14;   /* this run, 0..16383 cm         */
} s2_tlm_t;

ER s2_get_tlm(s2_tlm_t *p_tlm);

/* ---------------------------------------------------------------------
 *  Steering input, S3 Line Following -> S2  (DRAFT: format to be agreed
 *  with S3, then recorded in TEAM.md section 5)
 *
 *  S3 calls s2_set_steering() every time it has a new line position
 *  (e.g. every 10 ms).  S2 keeps only the latest value, under its own
 *  mutex, and uses it only while the State Machine's nav mode is
 *  TRACKING or RECOVERY.  A value older than STEER_STALE_MS is ignored
 *  and the car holds still until fresh steering arrives.
 *
 *  steer_pct: -100..+100 (clamped).  Positive = steer RIGHT.  It sets how
 *  much the inside wheel slows down: 0 = straight, 50 = inside wheel at
 *  half speed, 100 = inside wheel stopped (pivot).
 *
 *  Never blocks for long: the mutex is held only to copy two values.
 *  Returns E_OK, or the tk_loc_mtx() error.
 * --------------------------------------------------------------------- */
#define S2_STEER_MAX_PCT    100

ER s2_set_steering(INT steer_pct);

#endif /* SUBSYSTEM2_H */
