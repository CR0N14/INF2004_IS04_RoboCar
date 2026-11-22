/*
 *  imu_console.c
 *  Subsystem 4 console output.  See imu_console.h.
 *
 *  tm_printf() supports only d i u x X o p s c, so every fractional
 *  value is printed as an integer in stated units (mg, 0.1 deg, ...).
 */

#include "imu_console.h"
#include "imu_cfg.h"
#include "imu_params.h"
#include "drv_imu.h"
#include "subsystem4.h"

/* ---- Stage 2 bring-up readout (S4_CONSOLE_RAW) -------------------- */

#define RAW_PRINT_CYCLES    50u     /* raw values every 500 ms           */
#define RAW_SCAN_CYCLES     500u    /* scan, ID and set-up every 5 s     */
#define RAW_RETRY_CYCLES    100u    /* set-up retry every 1 s if not OK  */
#define RAW_FIFO_MAX        12u     /* samples read per cycle at most    */
#define RAW_SCAN_FIRST      0x08u   /* 0x00-0x07 and 0x78-0x7F reserved  */
#define RAW_SCAN_LAST       0x77u
#define RAW_MS_PER_CYCLE    10u
#define RAW_LOST_CYCLES     5u      /* failed cycles in a row: set up again */
#define RAW_BURST_SAMPLES   4u
#define RAW_REG_CTRL0       0x1Fu
#define RAW_REG_FIFO_CTRL   0x2Eu
#define RAW_CTRL_REGS       8u

/* Magnetometer at +-2 gauss: 0.080 mgauss/LSB (Table 3) = 2/25 */
#define MAG_MG_NUM          2
#define MAG_MG_DEN          25

typedef struct
{
    int32_t  acc_sum[3];    /* raw counts, sensor axes                   */
    uint32_t acc_n;
    int32_t  mag_sum[3];
    uint32_t mag_n;
    uint32_t cycles;        /* cycles with an accelerometer read         */
    uint32_t fifo_total;
    uint8_t  fifo_min;
    uint8_t  fifo_max;
    uint32_t bus_us_max;    /* I2C time in one cycle, worst case         */
} raw_window_t;

static raw_window_t g_win;

static void
raw_window_reset(void)
{
    uint32_t k;

    for (k = 0u; k < 3u; k++)
    {
        g_win.acc_sum[k] = 0;
        g_win.mag_sum[k] = 0;
    }
    g_win.acc_n = 0u;
    g_win.mag_n = 0u;
    g_win.cycles = 0u;
    g_win.fifo_total = 0u;
    g_win.fifo_min = UINT8_MAX;
    g_win.fifo_max = 0u;
    g_win.bus_us_max = 0u;
}

static int32_t
isqrt_u32(uint32_t x)
{
    uint32_t r = 0u;
    uint32_t bit = 1uL << 30;

    while (bit > x)
    {
        bit >>= 2;
    }
    while (0u != bit)
    {
        if (x >= (r + bit))
        {
            x -= r + bit;
            r = (r >> 1) + bit;
        }
        else
        {
            r >>= 1;
        }
        bit >>= 2;
    }
    return (int32_t)r;
}

static int32_t
div_round(int32_t num, int32_t den)
{
    return (num >= 0) ? ((num + (den / 2)) / den) : ((num - (den / 2)) / den);
}

static void
print_time(uint32_t cycle)
{
    uint32_t const ds = (cycle * RAW_MS_PER_CYCLE) / 100u;

    tm_printf((UB *)"s4 t=%u.%u ", (unsigned int)(ds / 10u),
              (unsigned int)(ds % 10u));
}

static uint8_t g_raw_last_addr = DRV_IMU_NO_ADDR;  /* for "address changed" */

static char const *
sa0_name(uint8_t addr)
{
    return (DRV_IMU_ADDR_SA0_HIGH == addr) ? "SA0 high" : "SA0 low";
}

/* Warn if the chip turns up at a different address than last time: the
   board's SA0 pin may be floating.  true if it moved. */
static bool
addr_changed(uint8_t *p_last, uint8_t addr)
{
    bool const b_moved = (DRV_IMU_NO_ADDR != *p_last) &&
                         (DRV_IMU_NO_ADDR != addr) && (addr != *p_last);

    if (b_moved)
    {
        tm_printf((UB *)"s4 ADDRESS CHANGED 0x%02x -> 0x%02x: SA0 may be "
                  "floating on this board (wire the SA0 pin to GND or 3V3)\n",
                  (unsigned int)*p_last, (unsigned int)addr);
    }
    if (DRV_IMU_NO_ADDR != addr)
    {
        *p_last = addr;
    }
    return b_moved;
}

/* Bus scan, then WHO_AM_I at 0x1D and 0x1E.  Returns the address of the
   LSM303D (DRV_IMU_NO_ADDR if none); only that address is ever set up. */
static uint8_t
raw_scan_and_identify(uint32_t cycle)
{
    uint8_t id_hi = 0u;
    uint8_t id_lo = 0u;
    uint8_t addr;
    uint32_t found = 0u;
    uint32_t a;
    ER er_hi;
    ER er_lo;

    print_time(cycle);
    tm_printf((UB *)"scan I2C1 (GP2 SDA, GP3 SCL):");
    for (a = RAW_SCAN_FIRST; a <= RAW_SCAN_LAST; a++)
    {
        if (drv_imu_probe((uint8_t)a))
        {
            tm_printf((UB *)" 0x%02x", (unsigned int)a);
            found++;
        }
    }
    tm_printf((UB *)"  (%u found)\n", (unsigned int)found);

    er_hi = drv_imu_whoami_at(DRV_IMU_ADDR_SA0_HIGH, &id_hi);
    er_lo = drv_imu_whoami_at(DRV_IMU_ADDR_SA0_LOW, &id_lo);
    (void)drv_imu_find(&addr);
    print_time(cycle);
    tm_printf((UB *)"id: WHO_AM_I (0Fh) at 0x1d = %02x (er %d), at 0x1e = %02x "
              "(er %d), want 49 -> ", (unsigned int)id_hi, (int)er_hi,
              (unsigned int)id_lo, (int)er_lo);
    if (DRV_IMU_NO_ADDR != addr)
    {
        tm_printf((UB *)"LSM303D OK at 0x%02x (%s)\n", (unsigned int)addr,
                  sa0_name(addr));
    }
    else
    {
        tm_printf((UB *)"no LSM303D found: stop and report\n");
    }
    return addr;
}

static void
raw_print_config(uint32_t cycle, ER cfg_er)
{
    uint8_t val;
    uint32_t i;

    print_time(cycle);
    tm_printf((UB *)"cfg (er %d): CTRL0-7", (int)cfg_er);
    for (i = 0u; i < RAW_CTRL_REGS; i++)
    {
        val = 0u;
        (void)drv_imu_read_reg((uint8_t)(RAW_REG_CTRL0 + i), &val);
        tm_printf((UB *)" %02x", (unsigned int)val);
    }
    val = 0u;
    (void)drv_imu_read_reg(RAW_REG_FIFO_CTRL, &val);
    tm_printf((UB *)"  FIFO_CTRL %02x  (want 40 8f c8 00 00 74 00 00, 40)\n",
              (unsigned int)val);
}

static void
raw_print_stats(uint32_t cycle)
{
    drv_imu_stats_t st;

    drv_imu_get_stats(&st);
    print_time(cycle);
    tm_printf((UB *)"i2c: %u transfers to the IMU, %u failed (%u no-ack, %u "
              "timeout), %u bus clears, %u FIFO overruns, longest %u us "
              "(%u scan probes not counted)\n",
              (unsigned int)st.xfers, (unsigned int)st.fails,
              (unsigned int)st.nacks, (unsigned int)st.timeouts,
              (unsigned int)st.recoveries, (unsigned int)st.fifo_overruns,
              (unsigned int)st.max_xfer_us, (unsigned int)st.probes);
}

/* One multi-sample burst: does the FIFO pointer wrap 2Dh -> 28h? */
static void
raw_burst_check(uint32_t cycle)
{
    drv_imu_raw_t s[RAW_BURST_SAMPLES];
    uint8_t before = 0u;
    uint8_t after = 0u;
    uint32_t i;
    ER er;

    er = drv_imu_burst_check(s, (uint8_t)RAW_BURST_SAMPLES, &before, &after);
    print_time(cycle);
    if (E_OK != er)
    {
        tm_printf((UB *)"fifo burst check: not run (er %d, %u waiting)\n",
                  (int)er, (unsigned int)before);
        return;
    }
    tm_printf((UB *)"fifo burst check: %u samples in one read, level %u -> %u:"
              " %s ", (unsigned int)RAW_BURST_SAMPLES, (unsigned int)before,
              (unsigned int)after,
              (((int32_t)before - (int32_t)after) >=
               (int32_t)(RAW_BURST_SAMPLES - 1u)) ?
              "wraps, burst OK" : "does NOT wrap, keep one read per sample");
    for (i = 0u; i < RAW_BURST_SAMPLES; i++)
    {
        tm_printf((UB *)" [%+d %+d %+d]", (int)s[i].v[0], (int)s[i].v[1],
                  (int)s[i].v[2]);
    }
    tm_printf((UB *)"\n");
}

/* One 10 ms cycle: drain the FIFO, read the magnetometer every second
   cycle (50 Hz).  Returns false if any read failed. */
static bool
raw_read(uint32_t cycle)
{
    drv_imu_raw_t s[RAW_FIFO_MAX];
    drv_imu_raw_t m;
    uint32_t const t0 = drv_imu_now_us();
    uint32_t bus_us;
    uint8_t n = 0u;
    uint8_t i;
    uint32_t k;
    bool b_ovr;
    bool b_ok = true;

    if (E_OK == drv_imu_read_acc(s, RAW_FIFO_MAX, &n, &b_ovr))
    {
        for (i = 0u; i < n; i++)
        {
            for (k = 0u; k < 3u; k++)
            {
                g_win.acc_sum[k] += s[i].v[k];
            }
        }
        g_win.acc_n += n;
        g_win.cycles++;
        g_win.fifo_total += n;
        g_win.fifo_min = (n < g_win.fifo_min) ? n : g_win.fifo_min;
        g_win.fifo_max = (n > g_win.fifo_max) ? n : g_win.fifo_max;
    }
    else
    {
        b_ok = false;
    }

    if (0u == (cycle & 1u))
    {
        if (E_OK == drv_imu_read_mag(&m))
        {
            for (k = 0u; k < 3u; k++)
            {
                g_win.mag_sum[k] += m.v[k];
            }
            g_win.mag_n++;
        }
        else
        {
            b_ok = false;
        }
    }

    bus_us = drv_imu_now_us() - t0;
    if (bus_us > g_win.bus_us_max)
    {
        g_win.bus_us_max = bus_us;
    }
    return b_ok;
}

static void
raw_print_values(uint32_t cycle, uint32_t fails)
{
    int32_t a[3] = { 0, 0, 0 };
    int32_t mg[3];
    int32_t m[3] = { 0, 0, 0 };
    int32_t b_mg[3];
    uint32_t k;
    int32_t a_mag;
    int32_t b_mag;

    for (k = 0u; k < 3u; k++)
    {
        if (0u != g_win.acc_n)
        {
            a[k] = div_round(g_win.acc_sum[k], (int32_t)g_win.acc_n);
        }
        if (0u != g_win.mag_n)
        {
            m[k] = div_round(g_win.mag_sum[k], (int32_t)g_win.mag_n);
        }
        mg[k] = div_round(a[k] * 1000, IMU_ACC_RAW_PER_G);
        b_mg[k] = div_round(m[k] * MAG_MG_NUM, MAG_MG_DEN);
    }
    a_mag = isqrt_u32((uint32_t)((mg[0] * mg[0]) + (mg[1] * mg[1]) +
                                 (mg[2] * mg[2])));
    b_mag = isqrt_u32((uint32_t)((b_mg[0] * b_mg[0]) + (b_mg[1] * b_mg[1]) +
                                 (b_mg[2] * b_mg[2])));

    print_time(cycle);
    tm_printf((UB *)"@%02x ", (unsigned int)drv_imu_addr());
    tm_printf((UB *)"acc mg X=%+d Y=%+d Z=%+d |a|=%d (raw %+d %+d %+d)"
              "  mag mG X=%+d Y=%+d Z=%+d |B|=%d (raw %+d %+d %+d)"
              "  fifo %u/%u/%u  fails %u  bus %u us\n",
              (int)mg[0], (int)mg[1], (int)mg[2], (int)a_mag,
              (int)a[0], (int)a[1], (int)a[2],
              (int)b_mg[0], (int)b_mg[1], (int)b_mg[2], (int)b_mag,
              (int)m[0], (int)m[1], (int)m[2],
              (unsigned int)((UINT8_MAX == g_win.fifo_min) ? 0u :
                             g_win.fifo_min),
              (unsigned int)((0u != g_win.cycles) ?
                             (g_win.fifo_total / g_win.cycles) : 0u),
              (unsigned int)g_win.fifo_max, (unsigned int)fails,
              (unsigned int)g_win.bus_us_max);
}

EXPORT void
imu_console_raw(void (*tick)(void))
{
    uint32_t cycle = 0u;
    uint32_t fails = 0u;        /* failed read cycles since boot         */
    uint32_t in_a_row = 0u;     /* consecutive failed read cycles        */
    uint8_t addr = DRV_IMU_NO_ADDR;
    bool b_cfg = false;
    ER cfg_er = E_OK;

    drv_imu_init();
    raw_window_reset();
    tm_printf((UB *)"s4 raw: LSM303D bring-up (0x1d or 0x1e), I2C1 at %u kHz, "
              "FIFO %s\n", (unsigned int)(S4_CFG_I2C_100KHZ ? 100u : 400u),
              S4_CFG_FIFO_BURST ? "burst" : "one read per sample");

    for (;;)
    {
        if (0u == (cycle % RAW_SCAN_CYCLES))
        {
            addr = raw_scan_and_identify(cycle);
            if (addr_changed(&g_raw_last_addr, addr))
            {
                b_cfg = false;          /* set it up at the new address */
            }
            if ((DRV_IMU_NO_ADDR != addr) && !b_cfg)
            {
                cfg_er = drv_imu_configure();
                b_cfg = (E_OK == cfg_er);
            }
            if (DRV_IMU_NO_ADDR != addr)
            {
                /* Only an identified chip is ever written or read back */
                raw_print_config(cycle, cfg_er);
            }
            else
            {
                b_cfg = false;
            }
            raw_print_stats(cycle);
        }
        else if ((DRV_IMU_NO_ADDR != addr) && !b_cfg &&
                 (0u == (cycle % RAW_RETRY_CYCLES)))
        {
            /* Lost or never set up: find it again, then set it up */
            drv_imu_recover();
            addr = DRV_IMU_NO_ADDR;
            if (E_OK == drv_imu_find(&addr))
            {
                (void)addr_changed(&g_raw_last_addr, addr);
                cfg_er = drv_imu_configure();
                b_cfg = (E_OK == cfg_er);
            }
        }
        else if (b_cfg && (0u == ((cycle + (RAW_SCAN_CYCLES / 2u)) %
                                  RAW_SCAN_CYCLES)))
        {
            raw_burst_check(cycle);     /* every 5 s, between the scans  */
        }
        else
        {
            /* nothing extra this cycle */
        }

        if (b_cfg)
        {
            if (raw_read(cycle))
            {
                in_a_row = 0u;
            }
            else
            {
                fails++;
                in_a_row++;
                if (in_a_row >= RAW_LOST_CYCLES)
                {
                    b_cfg = false;      /* set up again once it answers */
                    in_a_row = 0u;
                }
            }
        }

        if (0u == ((cycle + 1u) % RAW_PRINT_CYCLES))
        {
            raw_print_values(cycle + 1u, fails);
            raw_window_reset();
        }

        tick();
        cycle++;
    }
}

/* ---- Accelerometer diagnosis (S4_CONSOLE_ACCDIAG) ----------------- */

/*
 * Each set-up: write CTRL0-CTRL7 and FIFO_CTRL, let it settle 200 ms,
 * then measure 500 ms.  Without the FIFO, OUT_*_A is read once per 10 ms
 * cycle when STATUS_A says new data is ready.  A working sensor lying
 * still shows |a| about 1000 mg and a spread (sd) of a few mg, with the
 * self-test set-up moving each axis by 70-1700 mg (datasheet Table 3).
 */
#define DIAG_SETTLE_CYCLES  20u
#define DIAG_MEASURE_CYCLES 50u
#define DIAG_BOOT_CYCLES    5u
#define DIAG_REG_STATUS_A   0x27u
#define DIAG_REG_OUT_X_L_A  0x28u
#define DIAG_REG_TEMP_L     0x05u
#define DIAG_REG_CTRL0      0x1Fu
#define DIAG_REG_FIFO_CTRL  0x2Eu
#define DIAG_ZYXDA          0x08u
#define DIAG_BOOT           0x80u
#define DIAG_CTRL5          0xF4u   /* temperature on, mag as normal     */

typedef struct
{
    char const *p_name;
    uint8_t ctrl0;
    uint8_t ctrl1;
    uint8_t ctrl2;
    uint8_t fifo_ctrl;
    uint16_t ug_per_lsb;            /* sensitivity, Table 3              */
    bool    b_boot;                 /* reload factory trim first         */
} diag_cfg_t;

static diag_cfg_t const g_diag[] =
{
    { "A boot, defaults: 100 Hz +-2g AA773 no FIFO", 0x00u, 0x67u, 0x00u,
      0x00u, 61u, true },
    { "B 100 Hz +-4g AA773 no FIFO", 0x00u, 0x67u, 0x08u, 0x00u, 122u, false },
    { "C 400 Hz +-4g AA773 no FIFO", 0x00u, 0x87u, 0x08u, 0x00u, 122u, false },
    { "D 400 Hz +-4g AA50 BDU no FIFO", 0x00u, 0x8Fu, 0xC8u, 0x00u, 122u,
      false },
    { "E ours: D + FIFO stream", 0x40u, 0x8Fu, 0xC8u, 0x40u, 122u, false },
    { "F self-test: A with AST on", 0x00u, 0x67u, 0x02u, 0x00u, 61u, false },
    { "G 100 Hz +-16g AA773 no FIFO", 0x00u, 0x67u, 0x20u, 0x00u, 732u, false }
};

typedef struct
{
    int64_t  sum[3];
    int64_t  sq[3];
    int32_t  lo[3];
    int32_t  hi[3];
    uint32_t n;
} diag_stats_t;

static void
diag_add(diag_stats_t *p_st, int16_t const *p_v)
{
    uint32_t k;

    for (k = 0u; k < 3u; k++)
    {
        int32_t const v = p_v[k];

        p_st->sum[k] += v;
        p_st->sq[k] += (int64_t)v * v;
        if ((0u == p_st->n) || (v < p_st->lo[k]))
        {
            p_st->lo[k] = v;
        }
        if ((0u == p_st->n) || (v > p_st->hi[k]))
        {
            p_st->hi[k] = v;
        }
    }
    p_st->n++;
}

static ER
diag_apply(diag_cfg_t const *p_cfg, void (*tick)(void), uint8_t *p_rb)
{
    uint8_t const ctrl[8] =
    {
        p_cfg->ctrl0, p_cfg->ctrl1, p_cfg->ctrl2, 0x00u, 0x00u, DIAG_CTRL5,
        0x00u, 0x00u
    };
    uint32_t i;
    ER er = E_OK;

    if (p_cfg->b_boot)
    {
        er = drv_imu_write_reg(DIAG_REG_CTRL0, DIAG_BOOT);
        for (i = 0u; i < DIAG_BOOT_CYCLES; i++)
        {
            tick();
        }
    }
    if (E_OK == er)
    {
        er = drv_imu_write_reg(DIAG_REG_FIFO_CTRL, 0x00u);
    }
    for (i = 0u; (i < 8u) && (E_OK == er); i++)
    {
        er = drv_imu_write_reg((uint8_t)(DIAG_REG_CTRL0 + i), ctrl[i]);
    }
    if (E_OK == er)
    {
        er = drv_imu_write_reg(DIAG_REG_FIFO_CTRL, p_cfg->fifo_ctrl);
    }
    if (E_OK == er)
    {
        er = drv_imu_read_regs(DIAG_REG_CTRL0, p_rb, 8u);
    }
    if (E_OK == er)
    {
        er = drv_imu_read_reg(DIAG_REG_FIFO_CTRL, &p_rb[8]);
    }
    return er;
}

/* One cycle of samples for this set-up into *p_st (if given) */
static ER
diag_sample(diag_cfg_t const *p_cfg, diag_stats_t *p_st)
{
    drv_imu_raw_t s[RAW_FIFO_MAX];
    uint8_t b[6];
    uint8_t status;
    uint8_t n;
    uint8_t i;
    bool b_ovr;
    ER er;

    if (0u != p_cfg->fifo_ctrl)
    {
        er = drv_imu_read_acc(s, RAW_FIFO_MAX, &n, &b_ovr);
        for (i = 0u; (E_OK == er) && (NULL != p_st) && (i < n); i++)
        {
            diag_add(p_st, s[i].v);
        }
        return er;
    }
    er = drv_imu_read_reg(DIAG_REG_STATUS_A, &status);
    if ((E_OK == er) && (0u != (status & DIAG_ZYXDA)))
    {
        er = drv_imu_read_regs(DIAG_REG_OUT_X_L_A, b, 6u);
        if ((E_OK == er) && (NULL != p_st))
        {
            int16_t v[3];

            for (i = 0u; i < 3u; i++)
            {
                v[i] = (int16_t)(uint16_t)(((uint16_t)b[(2u * i) + 1u] << 8) |
                                           b[2u * i]);
            }
            diag_add(p_st, v);
        }
    }
    return er;
}

static void
diag_print(uint32_t round, diag_cfg_t const *p_cfg, uint8_t const *p_rb,
           diag_stats_t const *p_st, ER er)
{
    int32_t mean_mg[3] = { 0, 0, 0 };
    uint8_t t[2] = { 0u, 0u };
    drv_imu_raw_t m;
    uint32_t k;
    int32_t a_mg;
    int32_t temp;

    (void)drv_imu_read_regs(DIAG_REG_TEMP_L, t, 2u);
    temp = (int32_t)(int16_t)(uint16_t)(((uint16_t)t[1] << 8) | t[0]);
    m.v[0] = 0;
    m.v[1] = 0;
    m.v[2] = 0;
    (void)drv_imu_read_mag(&m);

    tm_printf((UB *)"diag %u @%02x %s: CTRL0-7 %02x %02x %02x %02x %02x %02x "
              "%02x %02x FIFO %02x (er %d) n=%u", (unsigned int)round,
              (unsigned int)drv_imu_addr(), p_cfg->p_name, (unsigned int)p_rb[0], (unsigned int)p_rb[1],
              (unsigned int)p_rb[2], (unsigned int)p_rb[3],
              (unsigned int)p_rb[4], (unsigned int)p_rb[5],
              (unsigned int)p_rb[6], (unsigned int)p_rb[7],
              (unsigned int)p_rb[8], (int)er, (unsigned int)p_st->n);
    for (k = 0u; (k < 3u) && (0u != p_st->n); k++)
    {
        int64_t const mean = p_st->sum[k] / (int64_t)p_st->n;
        int64_t const var = (p_st->sq[k] / (int64_t)p_st->n) - (mean * mean);
        uint32_t const sd = (uint32_t)isqrt_u32((var > 0) ? (uint32_t)var : 0u);

        mean_mg[k] = (int32_t)((mean * p_cfg->ug_per_lsb) / 1000);
        tm_printf((UB *)"  %c mean %+d sd %u [%+d..%+d] = %+d mg",
                  (int)('X' + (int)k), (int)mean, (unsigned int)sd,
                  (int)p_st->lo[k], (int)p_st->hi[k], (int)mean_mg[k]);
    }
    a_mg = isqrt_u32((uint32_t)((mean_mg[0] * mean_mg[0]) +
                                (mean_mg[1] * mean_mg[1]) +
                                (mean_mg[2] * mean_mg[2])));
    tm_printf((UB *)"  |a| %d mg  temp raw %d  mag raw %+d %+d %+d\n",
              (int)a_mg, (int)temp, (int)m.v[0], (int)m.v[1], (int)m.v[2]);
}

EXPORT void
imu_console_accdiag(void (*tick)(void))
{
    uint32_t round = 0u;
    uint8_t id = 0u;
    uint8_t id_hi = 0u;
    uint8_t addr = DRV_IMU_NO_ADDR;
    uint8_t last = DRV_IMU_NO_ADDR;
    ER er;

    drv_imu_init();
    tm_printf((UB *)"s4 accdiag: LSM303D accelerometer diagnosis, %u set-ups, "
              "about 6 s per round\n",
              (unsigned int)(sizeof(g_diag) / sizeof(g_diag[0])));
    for (;;)
    {
        uint32_t c;

        round++;
        (void)drv_imu_whoami_at(DRV_IMU_ADDR_SA0_HIGH, &id_hi);
        (void)drv_imu_whoami_at(DRV_IMU_ADDR_SA0_LOW, &id);
        er = drv_imu_find(&addr);
        tm_printf((UB *)"diag %u: WHO_AM_I at 0x1d %02x, at 0x1e %02x -> %s",
                  (unsigned int)round, (unsigned int)id_hi, (unsigned int)id,
                  (E_OK == er) ? "LSM303D at 0x" : "no LSM303D, nothing "
                  "written\n");
        if (E_OK == er)
        {
            tm_printf((UB *)"%02x (%s)\n", (unsigned int)addr, sa0_name(addr));
            (void)addr_changed(&last, addr);
        }
        if (E_OK != er)
        {
            for (c = 0u; c < (DIAG_MEASURE_CYCLES * 2u); c++)
            {
                tick();
            }
            continue;
        }

        for (c = 0u; c < (sizeof(g_diag) / sizeof(g_diag[0])); c++)
        {
            diag_stats_t st;
            uint8_t rb[9] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };
            uint32_t i;
            uint32_t k;

            for (k = 0u; k < 3u; k++)
            {
                st.sum[k] = 0;
                st.sq[k] = 0;
                st.lo[k] = 0;
                st.hi[k] = 0;
            }
            st.n = 0u;
            er = diag_apply(&g_diag[c], tick, rb);
            for (i = 0u; (i < DIAG_SETTLE_CYCLES) && (E_OK == er); i++)
            {
                tick();
                er = diag_sample(&g_diag[c], NULL);
            }
            for (i = 0u; (i < DIAG_MEASURE_CYCLES) && (E_OK == er); i++)
            {
                tick();
                er = diag_sample(&g_diag[c], &st);
            }
            diag_print(round, &g_diag[c], rb, &st, er);
        }
    }
}

/* ---- Bench readout (S4_CONSOLE_BENCH) and status lines ------------ */

#define BENCH   (S4_CONSOLE_BENCH == S4_CFG_CONSOLE)

static char const * const g_class_name[] =
{
    "STATIONARY", "STRAIGHT", "ACCELERATING", "TURNING", "CLIMBING",
    "DESCENDING", "IMPACT"
};
static char const * const g_status_name[] =
{
    "OK", "SELFTEST", "FAULT", "IDLE"
};
static char const * const g_hump_name[] =
{
    "FLAT", "CLIMB", "DESCEND"
};
static char const * const g_cal_name[] =
{
    "ok", "too few samples", "car moved", "not 1 g"
};

static void
print_ms(uint32_t now_ms)
{
    tm_printf((UB *)"s4 %u.%02u ", (unsigned int)(now_ms / 1000u),
              (unsigned int)((now_ms % 1000u) / 10u));
}

/* cdeg as a signed value with one decimal */
static void
print_deg(char const *p_label, int32_t cdeg)
{
    int32_t const tenths = div_round((cdeg < 0) ? -cdeg : cdeg, 10);

    tm_printf((UB *)"%s %c%d.%d ", p_label, (cdeg < 0) ? '-' : '+',
              (int)(tenths / 10), (int)(tenths % 10));
}

EXPORT void
imu_console_selftest(bool b_ok, uint8_t cal_result, int32_t g_mg,
                     uint8_t addr, uint32_t now_ms)
{
    print_ms(now_ms);
    if (b_ok)
    {
        tm_printf((UB *)"IMU self-test passed: LSM303D at 0x%02x (|g| %d mg)\n",
                  (unsigned int)addr, (int)g_mg);
    }
    else
    {
        tm_printf((UB *)"IMU self-test FAILED 3 times (last: %s, |g| %d mg, "
                  "address %s%02x)%s\n",
                  (cal_result < 4u) ? g_cal_name[cal_result] : "?", (int)g_mg,
                  (DRV_IMU_NO_ADDR == addr) ? "none, " : "0x",
                  (unsigned int)addr,
                  S4_CFG_REPORT_ONLY ? ", report-only: car not stopped" : "");
    }
}

EXPORT void
imu_console_fault(ER er, uint32_t now_ms)
{
    print_ms(now_ms);
    tm_printf((UB *)"IMU FAULT: not responding (er %d), data invalid, "
              "retrying every 1 s\n", (int)er);
}

EXPORT void
imu_console_status(imu_view_t const *p_view, uint8_t status,
                   uint16_t spike_mg, imu_task_dbg_t const *p_dbg,
                   uint32_t now_ms)
{
    if (!BENCH)
    {
        return;
    }
    print_ms(now_ms);
    tm_printf((UB *)"%s ", (status < 4u) ? g_status_name[status] : "?");
    print_deg("pitch", p_view->pitch_cdeg);
    print_deg("roll", p_view->roll_cdeg);
    print_deg("head", p_view->heading_cdeg);
    print_deg("rate", p_view->rate_cdps);
    tm_printf((UB *)"%s %s peak %u/%u mm pk %u mg vib %u fwd %+d fails %u"
              " cpu %u us bus %u us\n",
              (p_view->cls < 7u) ? g_class_name[p_view->cls] : "?",
              (p_view->hump_state < 3u) ? g_hump_name[p_view->hump_state] :
              "?", (unsigned int)p_view->last_peak_mm,
              (unsigned int)p_view->max_peak_mm, (unsigned int)spike_mg,
              (unsigned int)p_view->vib_mg, (int)p_view->fwd_mg,
              (unsigned int)p_dbg->read_fails,
              (unsigned int)p_dbg->logic_us_max,
              (unsigned int)p_dbg->bus_us_max);
}

EXPORT void
imu_console_hump(imu_hump_t const *p_hump, uint32_t now_ms)
{
    if (!BENCH)
    {
        return;
    }
    print_ms(now_ms);
    tm_printf((UB *)"HUMP %u mm (%d um, %u ms, ", (unsigned int)p_hump->peak_mm,
              (int)p_hump->peak_um, (unsigned int)p_hump->duration_ms);
    print_deg("pitch", p_hump->max_pitch_cdeg);
    print_deg("/", p_hump->min_pitch_cdeg);
    tm_printf((UB *)"largest kick %u mg)\n", (unsigned int)p_hump->max_spike_mg);
}

EXPORT void
imu_console_collision(uint16_t spike_mg, bool b_sent, uint32_t now_ms)
{
    if (!BENCH)
    {
        return;
    }
    print_ms(now_ms);
    tm_printf((UB *)"COLLISION %u mg (threshold %d) -> %s\n",
              (unsigned int)spike_mg, IMU_COLLISION_MG,
              b_sent ? "fault sent to State Machine" : "report only");
}

EXPORT void
imu_console_tipover(int32_t pitch_cdeg, int32_t roll_cdeg, bool b_sent,
                    uint32_t now_ms)
{
    if (!BENCH)
    {
        return;
    }
    print_ms(now_ms);
    tm_printf((UB *)"TIP-OVER ");
    print_deg("pitch", pitch_cdeg);
    print_deg("roll", roll_cdeg);
    tm_printf((UB *)"-> %s\n", b_sent ? "fault sent to State Machine" :
              "report only");
}
