/* Trigger/echo ultrasonic demo: GP1 TRIG, GP0 ECHO. */
#include "drv_ultrasonic.h"

#define ULTRASONIC_TRIG_PIN       1u
#define ULTRASONIC_ECHO_PIN       0u
#define ECHO_RISE_TIMEOUT_US      30000u
#define ECHO_HIGH_TIMEOUT_US      30000u
#define ULTRASONIC_MAX_CM         400u

static UW now_us(void)
{
    /* Existing RP2040 free-running 1 MHz timer; read only, no reconfiguration.
       Unlike the latched TIMELR/TIMEHR pair, RAWL has no shared latch. */
    return in_w(TIMER_TIMERAWL);
}

static void delay_us(UW duration_us)
{
    UW start = now_us();

    /* Unsigned subtraction also handles the low word wrapping. */
    while ((UW)(now_us() - start) < duration_us) {
    }
}

static bool wait_echo(UINT level, UW timeout_us, UW *edge_us)
{
    UW start = now_us();

    for (;;) {
        UINT echo = gpio_get_val(ULTRASONIC_ECHO_PIN);
        UW now = now_us();

        if ((UW)(now - start) >= timeout_us) {
            return false;
        }
        if (echo == level) {
            *edge_us = now;
            return true;
        }
    }
}

ER s5_ultrasonic_init(void)
{
    ER er;

    /* GP0/GP1 cannot be shared with UART0 crash reporting. */
    if (!BOARD_UART0_FREE) {
        return E_NOSPT;
    }

    er = gpio_set_pin(ULTRASONIC_ECHO_PIN, GPIO_MODE_IN);
    if (er < E_OK) {
        return er;
    }
    /* Preload low before enabling the output to avoid a spurious trigger. */
    er = gpio_set_val(ULTRASONIC_TRIG_PIN, 0u);
    if (er < E_OK) {
        return er;
    }
    return gpio_set_pin(ULTRASONIC_TRIG_PIN, GPIO_MODE_OUT);
}

UINT s5_ultrasonic_read_cm(void)
{
    UW rise_us, fall_us, pulse_us;
    UINT distance_cm;

    /* A stale/stuck-high ECHO is not a new measurement. Do not trigger it. */
    if (gpio_get_val(ULTRASONIC_ECHO_PIN) != 0u) {
        return S5_NO_ECHO_CM;
    }

    (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 0u);
    delay_us(2u);
    (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 1u);
    delay_us(10u);
    (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 0u);

    /* Busy polling is bounded; interrupts and task preemption stay enabled.
       Scheduling delays can affect accuracy or cause a missed echo. */
    if (!wait_echo(1u, ECHO_RISE_TIMEOUT_US, &rise_us) ||
        !wait_echo(0u, ECHO_HIGH_TIMEOUT_US, &fall_us)) {
        return S5_NO_ECHO_CM;
    }

    pulse_us = (UW)(fall_us - rise_us);
    if (pulse_us == 0u || pulse_us >= ECHO_HIGH_TIMEOUT_US) {
        return S5_NO_ECHO_CM;
    }
    /* Round-trip pulse: approximately 58 us/cm at room temperature. */
    distance_cm = (UINT)((pulse_us + 29u) / 58u);
    if (distance_cm == 0u || distance_cm > ULTRASONIC_MAX_CM) {
        return S5_NO_ECHO_CM;
    }
    return distance_cm;
}
