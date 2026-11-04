/* SG90 on GP12 (PWM slice 6, channel A). No other GPIO is configured. */
#include "drv_servo.h"

#define SERVO_PIN          12u
#define SERVO_PERIOD_US    20000u
#define SERVO_MIN_US       500u
#define SERVO_MAX_US       2500u
#define SERVO_MAX_DEG      180u

ER s5_servo_set_angle(UINT angle_deg)
{
    UW pulse_us;

    if (angle_deg > SERVO_MAX_DEG) {
        return E_PAR;
    }

    /* Nominal 0..180 degrees; individual servos may need calibration. */
    pulse_us = SERVO_MIN_US +
        ((SERVO_MAX_US - SERVO_MIN_US) * angle_deg + SERVO_MAX_DEG / 2u) /
        SERVO_MAX_DEG;
    return pwm_set_cc(SERVO_PIN, pulse_us);
}

ER s5_servo_init(void)
{
    ER er;

    /* Reset only our slice: disabled, free-running, normal polarity. */
    out_w(PWM_CH6_BASE + PWM_CHx_CSR, 0u);

    /* BSP has no divider setter. Use the register API from libbsp_pwm.c.
       clk_sys = TMCLK_MHz MHz (125 here); divide by TMCLK_MHz for
       1 MHz / 1 us ticks. DIV is an 8.4 fixed-point register. */
    out_w(PWM_CH6_BASE + PWM_CHx_DIV, (UW)TMCLK_MHz << 4);
    out_w(PWM_CH6_BASE + PWM_CHx_CTR, 0u);

    er = pwm_set_wrap(SERVO_PIN, SERVO_PERIOD_US - 1u);
    if (er < E_OK) {
        return er;
    }
    er = s5_servo_set_angle(90u);
    if (er < E_OK) {
        return er;
    }
    er = pwm_set_pin(SERVO_PIN);
    if (er < E_OK) {
        return er;
    }
    return pwm_set_enabled(SERVO_PIN, TRUE);
}
