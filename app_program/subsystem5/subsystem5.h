/*
 *	subsystem5.h
 *	Subsystem 5 -- Adaptive Ultrasonic Scanning and Obstacle Profiling.  Student ID: 2303571.
 *
 *	EMPTY TEMPLATE.  Put here only what OTHER subsystems may use (object IDs,
 *	message formats, priorities); keep everything else LOCAL in the .c.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../subsystem1/sm_types.h"

#define S5_PRI_MAIN	PRI_ULTRASOUND		/* app.h priority map */

/* ---------------------------------------------------------------------
 *  IF-07 telemetry snapshot, S5 -> S1 Networking
 *
 *  The latest values only.  s5_get_tlm() copies the snapshot out under
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
    S5_SCAN_IDLE = 0,
    S5_SCAN_MONITORING,
    S5_SCAN_SCANNING,
    S5_SCAN_PROFILING,
    S5_SCAN_AVOIDANCE,
    S5_SCAN_RECOVERY,
    S5_SCAN_ERROR
} s5_scan_state_t;

#define S5_NO_ECHO_CM   511u    /* distance_cm when nothing echoes       */

/* 64 bits in 2 words */
typedef struct
{
    unsigned int scan_state     : 3;    /* s5_scan_state_t               */
    unsigned int distance_cm    : 9;    /* 0..400 cm, S5_NO_ECHO_CM      */
    unsigned int servo_deg      : 8;    /* 0..180 deg, 90 = ahead        */
    unsigned int recommendation : 3;    /* sm_avoid_t                    */
    unsigned int b_profile      : 1;    /* profile fields below valid    */
    unsigned int left_clear_cm  : 8;    /* 0..255 cm                     */
    unsigned int bearing_deg    : 8;    /* obstacle centre, 0..180 deg   */
    unsigned int width_cm       : 7;    /* 0..127 cm                     */
    unsigned int closest_cm     : 9;    /* 0..400 cm                     */
    unsigned int right_clear_cm : 8;    /* 0..255 cm                     */
} s5_tlm_t;

ER s5_get_tlm(s5_tlm_t *p_tlm);

#endif /* SUBSYSTEM5_H */
