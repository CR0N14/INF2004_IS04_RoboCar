/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU-Based Motion and Terrain Monitoring.  Student ID: 2303557.
 *
 *	What other subsystems may use.  Subsystem 4 talks to the others only
 *	through:
 *	  IF-07  s4_get_tlm()          snapshot below, read by S1 Networking
 *	  IF-08  net_post_event()      NET_EVT_HUMP, NET_EVT_MOTION_CLASS,
 *	                               NET_EVT_IMU_FAULT (S1 net_types.h)
 *	  IF-01  sm_post_event()       SM_EVT_READY / SM_EVT_SELFTEST_FAIL
 *	                               (arg SM_SUBSYS_IMU), SM_EVT_FAULT with
 *	                               SM_FAULT_COLLISION, SM_FAULT_TIP_OVER,
 *	                               SM_FAULT_IMU (non-fatal)
 *	  IF-02  sm_mode_flags()       reads SENSORS / MOTORS / INITIALISING
 *	Everything else is private to this folder.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_MAIN	PRI_IMU			/* app.h priority map */

/* ---------------------------------------------------------------------
 *  IF-07 telemetry snapshot, S4 -> S1 Networking
 *
 *  The latest values only.  s4_get_tlm() copies the snapshot out under
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
    S4_IMU_OK = 0,
    S4_IMU_SELFTEST,        /* self-test / calibration running           */
    S4_IMU_FAULT,           /* sensor not answering: data invalid        */
    S4_IMU_IDLE             /* SENSORS_ENABLED clear                     */
} s4_imu_status_t;

typedef enum
{
    S4_CLASS_STATIONARY = 0,
    S4_CLASS_MOVING_STRAIGHT,
    S4_CLASS_ACCELERATING,
    S4_CLASS_TURNING,
    S4_CLASS_CLIMBING_HUMP,
    S4_CLASS_DESCENDING_HUMP,
    S4_CLASS_SUDDEN_IMPACT
} s4_motion_class_t;

/* 56 bits in 2 words */
typedef struct
{
    unsigned int pitch_half_deg : 8;    /* |pitch|, 0..180 x 0.5 deg     */
    unsigned int b_pitch_down   : 1;    /* nose down                     */
    unsigned int roll_half_deg  : 8;    /* |roll|, 0..180 x 0.5 deg      */
    unsigned int b_roll_left    : 1;    /* rolled to the left (left side */
                                        /*   lower than the right)       */
    unsigned int heading_deg    : 9;    /* 0..359 deg, clockwise from    */
                                        /*   magnetic north              */
    unsigned int imu_status     : 2;    /* s4_imu_status_t               */
    unsigned int turn_rate_dps  : 9;    /* |turn rate|, 0..511 deg/s     */
    unsigned int b_turn_left    : 1;    /* turning anticlockwise         */
    unsigned int motion_class   : 3;    /* s4_motion_class_t             */
    unsigned int last_peak_mm   : 7;    /* last hump peak, 0..127 mm     */
    unsigned int max_peak_mm    : 7;    /* highest peak this run         */
} s4_tlm_t;

ER s4_get_tlm(s4_tlm_t *p_tlm);

#endif /* SUBSYSTEM4_H */
