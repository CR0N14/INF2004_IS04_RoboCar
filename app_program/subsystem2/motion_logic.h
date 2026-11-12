/*
 *  motion_logic.h
 *  Motion Control maths: unit conversion, speed measurement, the PI(D)
 *  speed controller, steering mix, stall detection and manoeuvre segments.
 *
 *  Pure logic: no kernel calls and no hardware access, only numbers in and
 *  numbers out.  That keeps it easy to read and lets subsystem2_test.c test
 *  it with made-up inputs (the same split S1 uses for sm_logic.c).
 *
 *  Units used everywhere: distance mm, speed mm/s, duty % (-100..100),
 *  time in control periods ("ticks", MOTION_PERIOD_MS each).
 */

#ifndef MOTION_LOGIC_H
#define MOTION_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include "motion_config.h"

/* TRUE once the measured values the controller needs are filled in */
bool ml_config_ok(void);

/* ---- Unit conversion ------------------------------------------------- */

/* Wheel travel for one encoder count: pi * diameter / counts per rev */
float ml_mm_per_count(void);

/* Wheel travel (mm) for each wheel to spin the car by deg degrees on the
 * spot: each wheel moves along a circle of diameter TRACK_WIDTH_MM, so
 * arc = pi * track * deg / 360. */
float ml_spin_arc_mm(int32_t deg);

/* ---- Speed measurement ----------------------------------------------- */

/* Remembers the last SPEED_WINDOW_TICKS + 1 counts of one wheel, so speed
 * = counts moved over the window / window time. */
typedef struct
{
    int32_t history[SPEED_WINDOW_TICKS + 1u];
    uint8_t newest;     /* index of the latest sample                  */
    uint8_t filled;     /* samples stored so far, up to window + 1     */
} ml_speed_t;

void  ml_speed_reset(ml_speed_t *p_s, int32_t count);
/* Add this tick's count; returns the speed in mm/s (signed) */
float ml_speed_update(ml_speed_t *p_s, int32_t count);

/* ---- PI(D) speed controller, one per wheel --------------------------- */

typedef struct
{
    float i_term;       /* integral term, already in duty %  */
    float prev_error;   /* for the D term                    */
    bool  b_first;      /* no previous error yet             */
} ml_pid_t;

void ml_pid_reset(ml_pid_t *p_pid);

/* One control step.  Returns the duty (-100..100) for this wheel:
 *   duty = feed-forward(target) + KP*e + I + KD*de/dt,  e = target - measured
 * dt_s = control period in seconds. */
int32_t ml_pid_step(ml_pid_t *p_pid, float target_mmps, float measured_mmps,
                    float dt_s);

/* Feed-forward part alone (exposed for tests) */
float ml_feed_forward(float target_mmps);

/* ---- Steering mix (line following) ----------------------------------- */

/* Turn a base speed and a steering value into two wheel targets.
 * steer_pct: -100..+100, positive = steer RIGHT.  The inside wheel slows
 * down and the outside wheel keeps the base speed, so neither wheel ever
 * goes faster than base:
 *   steer   0 -> both at base
 *   steer +50 -> left base, right base/2
 *   steer+100 -> left base, right 0 (pivot on the right wheel) */
void ml_steer_mix(int32_t base_mmps, int32_t steer_pct,
                  int32_t *p_left_mmps, int32_t *p_right_mmps);

/* ---- Stall detection (FR10) ------------------------------------------ */

typedef struct
{
    uint32_t still_ticks;   /* ticks with duty above limit and no pulse */
} ml_stall_t;

void ml_stall_reset(ml_stall_t *p_st);

/* Call once per tick per wheel.  Returns TRUE when the wheel has been
 * driven at more than STALL_DUTY_MIN_PCT without one encoder count for
 * STALL_TIME_MS. */
bool ml_stall_update(ml_stall_t *p_st, int32_t duty_pct, int32_t delta_counts);

/* ---- Manoeuvre segments (turns and obstacle bypass) ------------------ */

typedef enum
{
    ML_SEG_STRAIGHT = 0,    /* value = mm, + forward, - backward          */
    ML_SEG_SPIN             /* value = degrees, + left (anticlockwise)    */
} ml_seg_kind_t;

typedef struct
{
    ml_seg_kind_t kind;
    int16_t       value;
} ml_seg_t;

/* Wheel speed targets for a segment at the given speed (mm/s, > 0) */
void ml_seg_targets(ml_seg_t const *p_seg, int32_t speed_mmps,
                    int32_t *p_left_mmps, int32_t *p_right_mmps);

/* TRUE once the wheels have moved far enough since the segment started.
 * moved_l_mm / moved_r_mm: signed wheel travel since the start. */
bool ml_seg_done(ml_seg_t const *p_seg, float moved_l_mm, float moved_r_mm);

#endif /* MOTION_LOGIC_H */
