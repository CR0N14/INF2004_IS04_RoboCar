/* Trigger/echo ultrasonic demo: GP1 TRIG, GP0 ECHO. */
#include "drv_ultrasonic.h"
#include "../demo_config.h"

#define ULTRASONIC_TRIG_PIN       1u
#define ULTRASONIC_ECHO_PIN       0u
#define ECHO_RISE_TIMEOUT_US      30000u
#define ECHO_HIGH_TIMEOUT_US      30000u
#define ULTRASONIC_MAX_CM         400u
/* ECHO already HIGH this many reads in a row: trigger anyway to recover. */
#define ECHO_STUCK_RETRIGGER      3u

static UINT echo_stuck_reads;

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

#if ENCODER_FREE_DEMO
/* Encoders are disabled in this profile: S5 exclusively owns BANK0 IRQ.
 * Timestamp edges in the ISR, not when the task eventually runs again.
 * If both edges are pending, timing is ambiguous: reject rather than guess. */
#define ECHO_IRQ 13u
#define ECHO_INTR (IO_BANK0_BASE + 0x0f0u)
#define ECHO_EDGES 0xcu
static volatile UW capture_state, capture_rise, capture_width;
static void echo_isr(UINT intno)
{
    (void)intno;
    UW edges = in_w(ECHO_INTR) & ECHO_EDGES;
    UW stamp = now_us();
    out_w(ECHO_INTR, edges);
    if (capture_state == 0u || capture_state >= 3u) return;
    if (edges == 8u && capture_state == 1u) {
        capture_rise = stamp;
        capture_state = 2u;
    } else if (edges == 4u && capture_state == 2u) {
        capture_width = stamp - capture_rise;
        capture_state = 3u;
    } else if (edges) capture_state = 4u;
}
#else
static bool wait_echo(UINT level, UW timeout_us, UW *edge_us)
{
    UW start = now_us();
    for (;;) {
        UINT echo = gpio_get_val(ULTRASONIC_ECHO_PIN);
        UW now = now_us();
        if ((UW)(now - start) >= timeout_us) return false;
        if (echo == level) { *edge_us = now; return true; }
    }
}
#endif

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
    er = gpio_set_pin(ULTRASONIC_TRIG_PIN, GPIO_MODE_OUT);
    if (er < E_OK) return er;
#if ENCODER_FREE_DEMO
    T_DINT dint = {0};
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)echo_isr;
    er = tk_def_int(ECHO_IRQ, &dint);
    if (er < E_OK) return er;
    UINT mask;
    DI(mask);
    UW enable = IO_BANK0_BASE + (in_w(SIO_CPUID_ADDR) ? 0x130u : 0x100u);
    out_w(ECHO_INTR, ECHO_EDGES);
    set_w(enable, ECHO_EDGES);
    EnableInt(ECHO_IRQ, INTPRI_MAX_EXTINT_PRI);
    EI(mask);
#endif
    return E_OK;
}

UINT s5_ultrasonic_read_cm(void)
{
    UW pulse_us;
#if !ENCODER_FREE_DEMO
    UW rise_us, fall_us;
#endif
    UINT distance_cm;

    /* A stale/stuck-high ECHO is not a new measurement, so this read is
       invalid either way.  Normally the sensor drops ECHO by itself and
       skipping the trigger is right; but if it stays HIGH, never triggering
       again means it can never recover.  After a few such reads, send one
       trigger pulse (the reading is still reported as no echo). */
    if (gpio_get_val(ULTRASONIC_ECHO_PIN) != 0u) {
        if (++echo_stuck_reads >= ECHO_STUCK_RETRIGGER) {
            UINT stuck_mask;

            echo_stuck_reads = 0u;
            DI(stuck_mask);
            (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 0u);
            delay_us(2u);
            (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 1u);
            delay_us(10u);
            (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 0u);
            EI(stuck_mask);
        }
        return S5_NO_ECHO_CM;
    }
    echo_stuck_reads = 0u;

#if ENCODER_FREE_DEMO
    capture_state = 0u;
    out_w(ECHO_INTR, ECHO_EDGES);
    capture_state = 1u;
#endif
    UINT trigger_mask;
    DI(trigger_mask); /* Only the 12 us trigger; never the echo wait. */
    (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 0u);
    delay_us(2u);
    (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 1u);
    delay_us(10u);
    (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 0u);

    EI(trigger_mask);
#if ENCODER_FREE_DEMO
    UW start = now_us();
    while (capture_state < 3u && (UW)(now_us()-start) < 60000u)
        tk_dly_tsk(1);
    UW state = capture_state;
    pulse_us = capture_width;
    capture_state = 0u;
    if (state != 3u) return S5_NO_ECHO_CM;
#else
    /* Busy polling is bounded; interrupts and task preemption stay enabled.
       Scheduling delays can affect accuracy or cause a missed echo. */
    if (!wait_echo(1u, ECHO_RISE_TIMEOUT_US, &rise_us) ||
        !wait_echo(0u, ECHO_HIGH_TIMEOUT_US, &fall_us)) {
        return S5_NO_ECHO_CM;
    }

    pulse_us = (UW)(fall_us - rise_us);
#endif
    if (pulse_us < 116u || pulse_us > 23200u) {
        return S5_NO_ECHO_CM;
    }
    /* Round-trip pulse: approximately 58 us/cm at room temperature. */
    distance_cm = (UINT)((pulse_us + 29u) / 58u);
    if (distance_cm == 0u || distance_cm > ULTRASONIC_MAX_CM) {
        return S5_NO_ECHO_CM;
    }
    return distance_cm;
}
