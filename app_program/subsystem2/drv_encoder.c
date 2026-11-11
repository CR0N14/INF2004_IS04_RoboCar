/*
 *  drv_encoder.c
 *  Quadrature decoding in a GPIO interrupt (micro T-Kernel version).
 *
 *  How quadrature works: A and B are two square waves a quarter-cycle
 *  apart.  Combine them into a 2-bit state (A << 1 | B).  Turning one way
 *  the state goes 00 -> 10 -> 11 -> 01 -> 00; the other way it goes in
 *  reverse.  Looking up (previous state, new state) in a table gives +1, -1
 *  or "impossible" (both bits changed at once: an edge was missed).
 *
 *  Why this differs from z_motion_control/encoder.c
 *  -------------------------------------------------
 *  The SDK version installed its handler with gpio_add_raw_irq_handler_
 *  masked() and irq_set_enabled().  In this kernel build those SDK calls
 *  are EMPTY STUBS (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c),
 *  so that code would compile and link but the encoders would never count.
 *  micro T-Kernel owns the vector table, so here we:
 *    1. tk_def_int(IO_IRQ_BANK0, ...) to point the vector at our handler,
 *    2. turn on edge interrupts for our 4 pins in the IO_BANK0 registers,
 *    3. EnableInt() to switch the interrupt on in the NVIC.
 *
 *  Dual core: each RP2040 core has its own NVIC and its own set of IO_BANK0
 *  enable registers (PROC0_INTE / PROC1_INTE).  Steps 2 and 3 must be done
 *  for the SAME core, so they run with interrupts disabled (DI), which also
 *  stops the kernel moving this task to the other core in between.  The
 *  handler then always runs on that core.
 *
 *  Sharing: the RP2040 has ONE interrupt (IO_IRQ_BANK0) for all GPIO pins,
 *  and tk_def_int() keeps one handler per interrupt.  If S5 (ultrasonic
 *  echo) also wants GPIO interrupts, one of us must own IO_IRQ_BANK0 and
 *  call the other's handler.  This handler only touches our own 4 pins, so
 *  it is ready to be called from a shared dispatcher.
 */

#include "drv_encoder.h"

/* RP2040 datasheet 2.3.2 (interrupt numbers) and 2.19.6.1 (IO_BANK0
 * registers); offsets checked against pico-sdk hardware/regs/io_bank0.h.
 * The kernel's sysdef.h has IO_BANK0_BASE but not these registers. */
#define IO_IRQ_BANK0            13u
#define IO_BANK0_INTR(r)        (IO_BANK0_BASE + 0x0f0u + (4u * (r)))
#define IO_BANK0_PROC0_INTE(r)  (IO_BANK0_BASE + 0x100u + (4u * (r)))
#define IO_BANK0_PROC1_INTE(r)  (IO_BANK0_BASE + 0x130u + (4u * (r)))
#define IO_BANK0_INT_REGS       4u      /* 8 GPIOs per register, 30 GPIOs */

/* Each GPIO has 4 event bits: LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH */
#define GPIO_EVT_EDGE_FALL      0x4u
#define GPIO_EVT_EDGE_RISE      0x8u
#define GPIO_EVT_EDGES          (GPIO_EVT_EDGE_FALL | GPIO_EVT_EDGE_RISE)
#define GPIO_EVT_REG(gpio)      ((gpio) / 8u)
#define GPIO_EVT_SHIFT(gpio)    (4u * ((gpio) % 8u))

/* NVIC priority for the encoder interrupt: the highest an external
 * interrupt may use, so edges are not missed (NFR7).  The handler is short
 * and calls no kernel functions. */
#define ENC_INT_LEVEL           INTPRI_MAX_EXTINT_PRI

typedef struct
{
    UINT    pin_a;
    UINT    pin_b;
    int8_t  direction;      /* +1, or -1 if ENC_x_INVERT */
} encoder_hw_t;

static encoder_hw_t const g_enc[WHEEL_COUNT] =
{
    [WHEEL_LEFT]  = { ENC_L_PIN_A, ENC_L_PIN_B, ENC_L_INVERT ? -1 : +1 },
    [WHEEL_RIGHT] = { ENC_R_PIN_A, ENC_R_PIN_B, ENC_R_INVERT ? -1 : +1 },
};

/* Written ONLY by encoder_isr() (after init).  32-bit aligned, so a task on
 * either core reads each one in a single instruction. */
static volatile int32_t  g_counts[WHEEL_COUNT];
static volatile uint32_t g_errors[WHEEL_COUNT];
static volatile uint8_t  g_last_state[WHEEL_COUNT];

/* Our pins' edge bits in each of the 4 IO_BANK0 interrupt registers.
 * Built once in drv_encoder_init(), read by the handler. */
static UW g_edge_mask[IO_BANK0_INT_REGS];
static UW g_inte_base;      /* PROC0_INTE(0) or PROC1_INTE(0) */

/* Index = (previous state << 2) | new state.  QDEC_ERROR = impossible. */
static int8_t const g_qdec_table[16] =
{
/* new:      00          01          10          11       previous: */
             0,         -1,         +1,  QDEC_ERROR,    /* 00 */
            +1,          0,  QDEC_ERROR,         -1,    /* 01 */
            -1,  QDEC_ERROR,          0,         +1,    /* 10 */
    QDEC_ERROR,         +1,         -1,          0,     /* 11 */
};

INT
drv_encoder_qdec(uint8_t prev, uint8_t now)
{
    return g_qdec_table[((prev & 0x3u) << 2) | (now & 0x3u)];
}

static uint8_t
read_state(UW all_pins, encoder_hw_t const *p_e)
{
    uint8_t const a = (uint8_t)((all_pins >> p_e->pin_a) & 1u);
    uint8_t const b = (uint8_t)((all_pins >> p_e->pin_b) & 1u);

    return (uint8_t)((a << 1) | b);
}

/*
 * Runs on every edge of any encoder pin.  Kept short: no printing, no
 * kernel calls, one table lookup per wheel.
 */
static void
encoder_isr(UINT intno)
{
    (void)intno;

    /* 1. Acknowledge OUR pins' edges, and only ours: these bits are
          "write 1 to clear", so writing our mask leaves other pins'
          pending edges alone. */
    for (UINT r = 0u; r < IO_BANK0_INT_REGS; r++)
    {
        if (0u != g_edge_mask[r])
        {
            out_w(IO_BANK0_INTR(r), g_edge_mask[r]);
        }
    }

    /* 2. Read all pins once, then update each wheel that changed.  Reading
          AFTER acknowledging means an edge arriving now triggers us again
          instead of being lost. */
    UW const all = in_w(GPIO_IN);

    for (INT w = 0; w < (INT)WHEEL_COUNT; w++)
    {
        uint8_t const now = read_state(all, &g_enc[w]);
        INT const step = drv_encoder_qdec(g_last_state[w], now);

        if (QDEC_ERROR == step)
        {
            g_errors[w]++;      /* both bits changed: missed an edge */
        }
        else if (0 != step)
        {
            g_counts[w] += step * g_enc[w].direction;
        }
        g_last_state[w] = now;
    }
}

static void
add_edge_bits(UINT gpio)
{
    g_edge_mask[GPIO_EVT_REG(gpio)] |= GPIO_EVT_EDGES << GPIO_EVT_SHIFT(gpio);
}

ER
drv_encoder_init(void)
{
    T_DINT  dint = {0};
    UINT    imask;
    ER      er;

    /* Pins: input, SIO function, optional pull-up (pull-down off) */
    for (INT w = 0; w < (INT)WHEEL_COUNT; w++)
    {
        UINT const pins[2] = { g_enc[w].pin_a, g_enc[w].pin_b };

        for (INT i = 0; i < 2; i++)
        {
            (void)gpio_set_pin(pins[i], GPIO_MODE_IN);  /* BSP helper */
            clr_w(GPIO(pins[i]), GPIO_PDE);
            if (ENC_USE_PULLUPS)
            {
                set_w(GPIO(pins[i]), GPIO_PUE);
            }
            add_edge_bits(pins[i]);
        }
    }

    /* Let the pull-ups settle (one kernel tick), then record where each
       encoder starts, so the first edge decodes correctly. */
    (void)tk_dly_tsk(1);
    {
        UW const all = in_w(GPIO_IN);

        for (INT w = 0; w < (INT)WHEEL_COUNT; w++)
        {
            g_last_state[w] = read_state(all, &g_enc[w]);
            g_counts[w] = 0;
            g_errors[w] = 0u;
        }
    }

    /* 1. Point the IO_IRQ_BANK0 vector at our handler */
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)encoder_isr;
    er = tk_def_int(IO_IRQ_BANK0, &dint);
    if (er < E_OK)
    {
        return er;
    }

    /* 2 + 3. Enable our pins' edges and the NVIC line on THIS core */
    DI(imask);
    g_inte_base = (0u == in_w(SIO_CPUID_ADDR)) ? IO_BANK0_PROC0_INTE(0u)
                                               : IO_BANK0_PROC1_INTE(0u);
    for (UINT r = 0u; r < IO_BANK0_INT_REGS; r++)
    {
        if (0u != g_edge_mask[r])
        {
            out_w(IO_BANK0_INTR(r), g_edge_mask[r]);        /* clear stale */
            set_w(g_inte_base + (4u * r), g_edge_mask[r]);  /* enable ours */
        }
    }
    EnableInt(IO_IRQ_BANK0, ENC_INT_LEVEL);
    EI(imask);

    return E_OK;
}

int32_t
drv_encoder_count(wheel_t wheel)
{
    return g_counts[wheel];     /* single 32-bit read: can't be half-updated */
}

uint32_t
drv_encoder_errors(wheel_t wheel)
{
    return g_errors[wheel];
}

BOOL
drv_encoder_irq_enabled(void)
{
    if (0u == g_inte_base)
    {
        return FALSE;           /* drv_encoder_init() not run or failed */
    }
    for (UINT r = 0u; r < IO_BANK0_INT_REGS; r++)
    {
        if ((in_w(g_inte_base + (4u * r)) & g_edge_mask[r]) != g_edge_mask[r])
        {
            return FALSE;
        }
    }
    return TRUE;
}
