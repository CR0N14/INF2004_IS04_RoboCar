/*
 *  drv_motor.c
 *  PWM motor driving for the Robo Pico (micro T-Kernel version).
 *
 *  Driver truth table (Robo Pico datasheet), per motor, inputs A and B:
 *    A LOW,  B LOW  -> brake (outputs low)
 *    A HIGH, B LOW  -> one direction
 *    A LOW,  B HIGH -> the other direction
 *    A HIGH, B HIGH -> coast (outputs Hi-Z)
 *  Note brake/coast are the opposite way round from many other boards.
 *  Which pin means "forward" on OUR car is in motion_config.h.
 *
 *  How the RP2040 PWM works (datasheet section 4.5):
 *    Each slice has a 16-bit counter that counts 0, 1, ... TOP, then wraps
 *    to 0.  Each of its two channels (A = even GPIO, B = odd GPIO) has a
 *    compare value CC: the pin is HIGH while counter < CC.  So
 *      frequency = clk_sys / (DIV * (TOP + 1))
 *      duty      = CC / (TOP + 1)        (CC = TOP + 1 means 100 %)
 *    At clk_sys = 125 MHz, DIV = 1 and 20 kHz: TOP + 1 = 6250 steps.
 *
 *  This file uses the BSP's own helpers (bsp/libbsp.h: pwm_set_pin,
 *  pwm_set_wrap, pwm_set_cc, pwm_set_enabled).  They take a GPIO NUMBER and
 *  work out the slice/channel themselves.  The clock divider has no BSP
 *  helper, so it is written directly with out_w().
 */

#include "drv_motor.h"

/* clk_sys, fixed by the kernel's clock set-up (kernel/sysdepend/
 * pico_rp2040/cpu_clock.c sets PLL_SYS to 125 MHz).  The kernel runs a fixed
 * clock, so a constant is fine here (the SDK version read it at run time). */
#define PWM_SRC_CLK_HZ  ((UW)CLK_PLL_SYS_FREQ)

/* Slice number of a GPIO: two GPIOs per slice, 8 slices (datasheet 4.5.2) */
#define PWM_SLICE_OF(gpio)      (((gpio) >> 1) & 0x7u)
#define PWM_DIV_REG(slice)      (PWM_BASE + PWM_CHx_DIV + ((slice) * 0x14u))
#define PWM_DIV_INT_LSB         4u      /* DIV is 8.4 fixed point: int << 4 */

/* Integer clock divider.  1 gives the finest duty resolution. */
#define PWM_CLKDIV              1u

typedef struct
{
    UINT pin_fwd;
    UINT pin_rev;
} motor_hw_t;

static motor_hw_t const g_motors[WHEEL_COUNT] =
{
    [WHEEL_LEFT]  = { MOTOR_L_PIN_FWD, MOTOR_L_PIN_REV },
    [WHEEL_RIGHT] = { MOTOR_R_PIN_FWD, MOTOR_R_PIN_REV },
};

/* PWM counter counts 0..g_pwm_top then wraps.  A compare value of
 * g_pwm_top + 1 keeps the pin high for the whole cycle (100 %). */
static UW g_pwm_top;

static void
set_levels(motor_hw_t const *p_m, UW level_fwd, UW level_rev)
{
    (void)pwm_set_cc(p_m->pin_fwd, level_fwd);
    (void)pwm_set_cc(p_m->pin_rev, level_rev);
}

void
drv_motor_init(void)
{
    /* TOP for the wanted frequency: 125 000 000 / (1 * 20 000) - 1 = 6249 */
    g_pwm_top = (PWM_SRC_CLK_HZ / (PWM_CLKDIV * PWM_FREQ_HZ)) - 1u;

    for (INT w = 0; w < (INT)WHEEL_COUNT; w++)
    {
        motor_hw_t const *p_m = &g_motors[w];
        UINT const slice = PWM_SLICE_OF(p_m->pin_fwd);

        /* Both pins of a motor are channels A and B of the same slice, so
           one slice set-up covers both. */
        out_w(PWM_DIV_REG(slice), PWM_CLKDIV << PWM_DIV_INT_LSB);
        (void)pwm_set_wrap(p_m->pin_fwd, g_pwm_top);

        set_levels(p_m, 0u, 0u);    /* compare 0 = both LOW = brake */

        /* Route the pins to the PWM block only after the levels are 0, so
           the motors never see a random level at start-up. */
        (void)pwm_set_pin(p_m->pin_fwd);
        (void)pwm_set_pin(p_m->pin_rev);
        (void)pwm_set_enabled(p_m->pin_fwd, TRUE);
    }
}

void
drv_motor_set(wheel_t wheel, INT duty_pct)
{
    motor_hw_t const *p_m = &g_motors[wheel];
    UW level;

    if (duty_pct > 100)
    {
        duty_pct = 100;
    }
    if (duty_pct < -100)
    {
        duty_pct = -100;
    }

    /* duty % -> compare value.  Multiply first so integer division does
       not throw the fraction away: 40 % -> 40 * 6250 / 100 = 2500. */
    level = ((UW)((duty_pct < 0) ? -duty_pct : duty_pct) * (g_pwm_top + 1u))
            / 100u;

    if (duty_pct >= 0)
    {
        set_levels(p_m, level, 0u);     /* forward (0 % = brake) */
    }
    else
    {
        set_levels(p_m, 0u, level);     /* backward */
    }
}

void
drv_motor_brake(wheel_t wheel)
{
    set_levels(&g_motors[wheel], 0u, 0u);
}

void
drv_motor_coast(wheel_t wheel)
{
    UW const full = g_pwm_top + 1u;

    set_levels(&g_motors[wheel], full, full);
}

void
drv_motor_brake_all(void)
{
    drv_motor_brake(WHEEL_LEFT);
    drv_motor_brake(WHEEL_RIGHT);
}

UW
drv_motor_pwm_top(void)
{
    return g_pwm_top;
}
