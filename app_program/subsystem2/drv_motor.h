/*
 *  drv_motor.h
 *  Drive the two DC motors through the Robo Pico's on-board driver.
 *
 *  Only the Motion Control task may call these after start-up: it is the
 *  sole writer of PWM slices 4 and 5 (TEAM.md section 3).  Other subsystems
 *  send targets instead.
 *
 *  Ported from z_motion_control/motor.c.  Same behaviour, but written for
 *  micro T-Kernel: this build does not link the Pico SDK runtime, so the
 *  SDK's pwm_*() / clock_get_hz() functions are not available (and the BSP
 *  already uses some of those names for its own, different functions).
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "../app.h"
#include "motion_config.h"

/* Set up both PWM slices at PWM_FREQ_HZ and leave both motors BRAKED.
 * Call once, as early as possible, so the motors are held still. */
void drv_motor_init(void);

/* Drive one wheel.  duty_pct: -100 (full reverse) .. 0 .. +100 (full
 * forward); values outside are clamped.  0 = brake.
 * Forward = "FWD pin PWM, REV pin LOW": in the off part of each PWM cycle
 * both inputs are LOW, which the driver treats as brake, so speed is close
 * to proportional to duty. */
void drv_motor_set(wheel_t wheel, INT duty_pct);

/* Both inputs LOW: the driver shorts the motor, so it stops quickly (FR12). */
void drv_motor_brake(wheel_t wheel);

/* Both inputs HIGH: driver outputs float, so the wheel rolls to a stop. */
void drv_motor_coast(wheel_t wheel);

/* Brake both wheels.  Use for any emergency or fault. */
void drv_motor_brake_all(void);

/* PWM counter top value (counts 0..top).  For tests. */
UW drv_motor_pwm_top(void);

#endif /* DRV_MOTOR_H */
