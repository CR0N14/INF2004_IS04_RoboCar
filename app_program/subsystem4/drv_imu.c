/*
 *  drv_imu.c
 *  LSM303D driver on I2C1 (GP2 SDA, GP3 SCL), address 0x1D or 0x1E.  See
 *  drv_imu.h.  Register numbers and bits: datasheet DocID023312 Rev 2,
 *  sections 7 and 8.
 *
 *  Register definitions come from the kernel's own RP2040 headers: the
 *  RESETS, IO_BANK0, PADS_BANK0, SIO and TIMER blocks from
 *  include/sys/sysdepend/cpu/rp2040/sysdef.h (through tk/tkernel.h), and
 *  the I2C block from the kernel I2C driver's register header.  No Pico
 *  SDK header is used, so this file builds in every profile, including
 *  CONSOLE=uart WIFI=none, where the makefile adds no SDK include paths.
 *
 *  A transfer pushes its commands into the 16-entry TX FIFO while
 *  draining the 16-entry RX FIFO, so a multi-byte read runs back to back
 *  at full bus speed.  If the task is pre-empted mid-transfer the
 *  controller holds SCL low until the next command arrives.
 */

#include "drv_imu.h"
#include "imu_cfg.h"
#include "../../device/i2c/sysdepend/rp2040/i2c_sysdep.h"

/* ---- Pins and bus ------------------------------------------------- */

#define IMU_PIN_SDA         2u
#define IMU_PIN_SCL         3u
#define IMU_SDA_MASK        (1uL << IMU_PIN_SDA)
#define IMU_SCL_MASK        (1uL << IMU_PIN_SCL)

/* Input on, pull-up (the GY-511 has its own too), Schmitt, 4 mA */
#define IMU_PAD_CFG         (GPIO_IE | GPIO_PUE | GPIO_SHEMITT | \
                             GPIO_DRIVE_4MA)

#define I2C_REG(off)        ((UW)I2C1_BASE + (UW)(off))

/* clk_sys feeds I2C1; SYSCLK is in MHz (sysdef.h) */
#define I2C_CLK_HZ          ((uint32_t)SYSCLK * 1000000uL)
#if S4_CFG_I2C_100KHZ
#define I2C_BAUD_HZ         100000uL
#define I2C_BYTE_US         90uL    /* 9 bit times                       */
#else
#define I2C_BAUD_HZ         400000uL
#define I2C_BYTE_US         23uL
#endif

/* Not named by the kernel header (RP2040 datasheet section 4.3.17) */
#define I2C_ABRT_7B_ADDR_NOACK  (1uL << 0)
#define I2C_SDA_TX_HOLD_MASK    0x0000FFFFuL
#define I2C_HW_FIFO_DEPTH       16u
#define I2C_NO_TARGET           0xFFu

/* A transfer may take 3x its nominal bus time plus 1 ms */
#define XFER_BASE_US        1000uL
#define XFER_MARGIN         3uL
#define STOP_WAIT_US        500uL

/* Bus clear: bit-banged SCL at 100 kHz */
#define CLEAR_HALF_US       5uL
#define CLEAR_PULSES        9u

/* ---- LSM303D registers (Table 16) -------------------------------- */

#define REG_OUT_X_L_M       0x08u
#define REG_WHO_AM_I        0x0Fu
#define REG_CTRL0           0x1Fu   /* CTRL0..CTRL7 are 1Fh..26h         */
#define REG_OUT_X_L_A       0x28u
#define REG_FIFO_CTRL       0x2Eu
#define REG_FIFO_SRC        0x2Fu
#define SUB_AUTO_INC        0x80u   /* SUB bit 7: multi-byte (6.1.1)     */

/* CTRL0..CTRL7, written in this order and read back as a block */
#define CTRL0_VAL           0x40u   /* FIFO_EN                           */
#define CTRL1_VAL           0x8Fu   /* AODR 1000 = 400 Hz, BDU, Z Y X on */
#define CTRL2_VAL           0xC8u   /* ABW 11 = 50 Hz, AFS 001 = +-4 g   */
#define CTRL3_VAL           0x00u   /* no interrupts on INT1             */
#define CTRL4_VAL           0x00u   /* no interrupts on INT2             */
#define CTRL5_VAL           0x74u   /* M_RES 11 high res, M_ODR 101 =    */
                                    /*   100 Hz, temperature off         */
#define CTRL6_VAL           0x00u   /* MFS 00 = +-2 gauss                */
#define CTRL7_VAL           0x00u   /* filter bypassed, MD 00 continuous */
#define CTRL_REGS           8u

#define FIFO_BYPASS         0x00u   /* FM 000, also empties the FIFO     */
#define FIFO_STREAM         0x40u   /* FM 010 (Table 59)                 */

#define SRC_OVRN            0x40u   /* FIFO_SRC (Table 60)               */
#define SRC_EMPTY           0x20u
#define SRC_FSS             0x1Fu

#define SAMPLE_BYTES        6u
#define BURST_MAX_SAMPLES   16u     /* per call; the rest waits a cycle  */

static drv_imu_stats_t g_stats;
static uint8_t  g_tar = I2C_NO_TARGET;  /* address now in IC_TAR        */
static uint8_t  g_addr = DRV_IMU_NO_ADDR;   /* identified LSM303D       */
static bool     g_b_verify;             /* re-read WHO_AM_I before use  */
static uint8_t  g_burst[BURST_MAX_SAMPLES * SAMPLE_BYTES];

/* ---- Helpers ------------------------------------------------------ */

EXPORT uint32_t
drv_imu_now_us(void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

static void
delay_us(uint32_t us)
{
    uint32_t const t0 = drv_imu_now_us();

    while ((drv_imu_now_us() - t0) < us)
    {
        /* spin: the delays here are a few microseconds */
    }
}

static void
count(uint32_t *p_counter)
{
    if (UINT32_MAX != *p_counter)
    {
        (*p_counter)++;
    }
}

static int16_t
le16(uint8_t const *p)
{
    return (int16_t)(uint16_t)(((uint16_t)p[1] << 8) | p[0]);
}

/* ---- Pins, bus clear, controller set-up --------------------------- */

static void
pins_to_sio(void)
{
    out_w(GPIO_OE_CLR, IMU_SDA_MASK | IMU_SCL_MASK);   /* released     */
    out_w(GPIO_OUT_CLR, IMU_SDA_MASK | IMU_SCL_MASK);  /* low when on  */
    out_w(GPIO_CTRL(IMU_PIN_SDA), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(IMU_PIN_SCL), GPIO_CTRL_FUNCSEL_SIO);
}

static void
pins_to_i2c(void)
{
    out_w(GPIO_CTRL(IMU_PIN_SDA), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO_CTRL(IMU_PIN_SCL), GPIO_CTRL_FUNCSEL_I2C);
}

/*
 * A sensor reset in the middle of a read can be left holding SDA low.
 * Clock SCL (open-drain style: drive low or release) until it lets go,
 * then send a STOP.
 */
static void
bus_clear(void)
{
    uint8_t i;

    pins_to_sio();
    delay_us(CLEAR_HALF_US);
    for (i = 0u; (i < CLEAR_PULSES) &&
                 (0u == (in_w(GPIO_IN) & IMU_SDA_MASK)); i++)
    {
        out_w(GPIO_OE_SET, IMU_SCL_MASK);
        delay_us(CLEAR_HALF_US);
        out_w(GPIO_OE_CLR, IMU_SCL_MASK);
        delay_us(CLEAR_HALF_US);
    }

    /* STOP: SDA rises while SCL is high */
    out_w(GPIO_OE_SET, IMU_SCL_MASK);
    delay_us(CLEAR_HALF_US);
    out_w(GPIO_OE_SET, IMU_SDA_MASK);
    delay_us(CLEAR_HALF_US);
    out_w(GPIO_OE_CLR, IMU_SCL_MASK);
    delay_us(CLEAR_HALF_US);
    out_w(GPIO_OE_CLR, IMU_SDA_MASK);
    delay_us(CLEAR_HALF_US);

    pins_to_i2c();
}

/* Timing as the Pico SDK computes it: 400 kHz -> HCNT 126, LCNT 187,
   spike length 11, SDA hold 38 clocks (the kernel I2C driver uses the
   same numbers). */
static void
controller_setup(void)
{
    uint32_t const period = (I2C_CLK_HZ + (I2C_BAUD_HZ / 2u)) / I2C_BAUD_HZ;
    uint32_t const lcnt = (period * 3u) / 5u;
    uint32_t const hcnt = period - lcnt;
    uint32_t const hold = ((I2C_CLK_HZ * 3u) / 10000000uL) + 1u;
    uint32_t hold_reg;

    set_w(RESETS_RESET, RESETS_RESET_I2C1);
    clr_w(RESETS_RESET, RESETS_RESET_I2C1);
    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C1))
    {
        /* spin: the block leaves reset within a few clocks */
    }

    out_w(I2C_REG(I2Cx_ENABLE), 0u);
    out_w(I2C_REG(I2Cx_CON), I2C_CON_SPEED_FAST | I2C_CON_MASTER_MODE |
          I2C_CON_SLAVE_DISABLE | I2C_CON_RESTART_EN |
          I2C_CON_TX_EMPTY_CTRL);
    out_w(I2C_REG(I2Cx_TX_TL), 0u);
    out_w(I2C_REG(I2Cx_RX_TL), 0u);
    out_w(I2C_REG(I2Cx_INTR_MASK), 0u);         /* polled              */
    out_w(I2C_REG(I2Cx_FS_SCL_HCNT), hcnt);
    out_w(I2C_REG(I2Cx_FS_SCL_LCNT), lcnt);
    out_w(I2C_REG(I2Cx_FS_SPKLEN), (lcnt < 16u) ? 1u : (lcnt / 16u));
    hold_reg = (uint32_t)in_w(I2C_REG(I2Cx_SDA_HOLD));
    out_w(I2C_REG(I2Cx_SDA_HOLD), (hold_reg & ~I2C_SDA_TX_HOLD_MASK) | hold);
    out_w(I2C_REG(I2Cx_ENABLE), I2C_ENABLE_ENABLE);
    g_tar = I2C_NO_TARGET;
}

/* Bus clear and controller reset.  The chip may have reset as well, so
   its identity is checked again before it is next used. */
static void
bus_reset(void)
{
    count(&g_stats.recoveries);
    out_w(I2C_REG(I2Cx_ENABLE), 0u);
    bus_clear();
    controller_setup();
    g_b_verify = true;
}

EXPORT void
drv_imu_init(void)
{
    out_w(GPIO(IMU_PIN_SDA), IMU_PAD_CFG);
    out_w(GPIO(IMU_PIN_SCL), IMU_PAD_CFG);
    bus_clear();
    controller_setup();
    g_addr = DRV_IMU_NO_ADDR;
}

EXPORT void
drv_imu_recover(void)
{
    bus_reset();
    g_addr = DRV_IMU_NO_ADDR;           /* find it again before use     */
}

/* ---- Transfers ---------------------------------------------------- */

static bool
stop_seen(void)
{
    return (0u != (in_w(I2C_REG(I2Cx_RAW_INTR_STAT)) & I2C_INT_STOP_DET));
}

static void
wait_stop(void)
{
    uint32_t const t0 = drv_imu_now_us();

    while (!stop_seen() && ((drv_imu_now_us() - t0) < STOP_WAIT_US))
    {
        /* spin: the STOP follows within one byte time */
    }
    (void)in_w(I2C_REG(I2Cx_CLR_STOP_DET));
}

/*
 * Write n_wr bytes, then (with a repeated START) read n_rd bytes.  Either
 * count may be zero, not both.
 */
static ER
xfer_ex(uint8_t addr, uint8_t const *p_wr, uint32_t n_wr, uint8_t *p_rd,
        uint32_t n_rd, bool b_stats)
{
    uint32_t const total = n_wr + n_rd;
    uint32_t const limit = XFER_BASE_US +
                           ((total + 2u) * I2C_BYTE_US * XFER_MARGIN);
    uint32_t const t0 = drv_imu_now_us();
    uint32_t issued = 0u;
    uint32_t got = 0u;
    uint32_t took;
    ER er = E_OK;

    if (0u == total)
    {
        return E_PAR;
    }
    count(b_stats ? &g_stats.xfers : &g_stats.probes);

    if (addr != g_tar)
    {
        out_w(I2C_REG(I2Cx_ENABLE), 0u);
        out_w(I2C_REG(I2Cx_TAR), addr);
        out_w(I2C_REG(I2Cx_ENABLE), I2C_ENABLE_ENABLE);
        g_tar = addr;
    }
    (void)in_w(I2C_REG(I2Cx_CLR_TX_ABRT));
    (void)in_w(I2C_REG(I2Cx_CLR_STOP_DET));

    for (;;)
    {
        /* Queue commands while TX has room and RX cannot overflow */
        while ((issued < total) &&
               (0u != (in_w(I2C_REG(I2Cx_STATUS)) & I2C_STATUS_TFNF)) &&
               ((issued < n_wr) ||
                (((issued - n_wr) - got) < I2C_HW_FIFO_DEPTH)))
        {
            uint32_t cmd;

            if (issued < n_wr)
            {
                cmd = p_wr[issued];
            }
            else
            {
                cmd = I2C_DATA_CMD_CMD_READ;
                if ((issued == n_wr) && (n_wr > 0u))
                {
                    cmd |= I2C_DATA_CMD_RESTART;
                }
            }
            if ((issued + 1u) == total)
            {
                cmd |= I2C_DATA_CMD_STOP;
            }
            out_w(I2C_REG(I2Cx_DATA_CMD), cmd);
            issued++;
        }

        while ((got < n_rd) && (0u != in_w(I2C_REG(I2Cx_RXFLR))))
        {
            p_rd[got] = (uint8_t)(in_w(I2C_REG(I2Cx_DATA_CMD)) &
                                  I2C_DATA_CMD_DAT);
            got++;
        }

        if (0u != (in_w(I2C_REG(I2Cx_RAW_INTR_STAT)) & I2C_INT_TX_ABRT))
        {
            /* No acknowledge: the controller flushes TX and sends STOP */
            (void)in_w(I2C_REG(I2Cx_TX_ABRT_SOURCE));
            (void)in_w(I2C_REG(I2Cx_CLR_TX_ABRT));
            wait_stop();
            while (0u != in_w(I2C_REG(I2Cx_RXFLR)))
            {
                (void)in_w(I2C_REG(I2Cx_DATA_CMD));     /* discard     */
            }
            if (b_stats)
            {
                count(&g_stats.nacks);
            }
            er = E_IO;
            break;
        }
        if ((issued == total) && (got == n_rd) && stop_seen())
        {
            (void)in_w(I2C_REG(I2Cx_CLR_STOP_DET));
            break;
        }
        if ((drv_imu_now_us() - t0) > limit)
        {
            count(&g_stats.timeouts);
            bus_reset();
            er = E_TMOUT;
            break;
        }
    }

    took = drv_imu_now_us() - t0;
    if (b_stats && (took > g_stats.max_xfer_us))
    {
        g_stats.max_xfer_us = took;
    }
    if (b_stats && (E_OK != er))
    {
        count(&g_stats.fails);
    }
    return er;
}

static ER
xfer(uint8_t addr, uint8_t const *p_wr, uint32_t n_wr, uint8_t *p_rd,
     uint32_t n_rd)
{
    return xfer_ex(addr, p_wr, n_wr, p_rd, n_rd, true);
}

static ER
write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t const buf[2] = { reg, val };

    return xfer(addr, buf, 2u, NULL, 0u);
}

static ER
read_regs(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint32_t n)
{
    return xfer(addr, &reg, 1u, p_buf, n);
}

/* ---- Public API --------------------------------------------------- */

EXPORT bool
drv_imu_probe(uint8_t addr)
{
    uint8_t dummy;

    /* A one-byte read: harmless (it reads whatever register the chip's
       pointer is on).  Counted as a probe, not as a transfer: an empty
       address is the expected answer, not a failure.  Timeouts still
       count, since they mean a bus problem. */
    return (E_OK == xfer_ex(addr, NULL, 0u, &dummy, 1u, false));
}

EXPORT ER
drv_imu_whoami_at(uint8_t addr, uint8_t *p_id)
{
    uint8_t const reg = REG_WHO_AM_I;

    /* Part of the search, so counted as a probe: no answer at the other
       candidate address is expected, not a failure */
    *p_id = 0u;
    return xfer_ex(addr, &reg, 1u, p_id, 1u, false);
}

EXPORT ER
drv_imu_find(uint8_t *p_addr)
{
    static uint8_t const candidates[2] =
    {
        DRV_IMU_ADDR_SA0_HIGH, DRV_IMU_ADDR_SA0_LOW
    };
    uint8_t id;
    uint32_t i;

    g_addr = DRV_IMU_NO_ADDR;
    for (i = 0u; i < 2u; i++)
    {
        if ((E_OK == drv_imu_whoami_at(candidates[i], &id)) &&
            (DRV_IMU_WHO_AM_I == id))
        {
            g_addr = candidates[i];
            g_b_verify = false;
            break;
        }
    }
    *p_addr = g_addr;
    return (DRV_IMU_NO_ADDR != g_addr) ? E_OK : E_IO;
}

EXPORT uint8_t
drv_imu_addr(void)
{
    return g_addr;
}

/* The identified address, re-checked after a bus reset.  E_OBJ if there
   is none: nothing is ever read from or written to an unknown device. */
static ER
chip(uint8_t *p_addr)
{
    uint8_t id;

    if (DRV_IMU_NO_ADDR == g_addr)
    {
        return E_OBJ;
    }
    if (g_b_verify)
    {
        g_b_verify = false;
        if ((E_OK != drv_imu_whoami_at(g_addr, &id)) ||
            (DRV_IMU_WHO_AM_I != id))
        {
            g_addr = DRV_IMU_NO_ADDR;
            return E_OBJ;
        }
    }
    *p_addr = g_addr;
    return E_OK;
}

EXPORT ER
drv_imu_read_reg(uint8_t reg, uint8_t *p_val)
{
    uint8_t addr;
    ER er = chip(&addr);

    return (E_OK == er) ? read_regs(addr, reg, p_val, 1u) : er;
}

EXPORT ER
drv_imu_read_regs(uint8_t reg, uint8_t *p_buf, uint8_t n)
{
    uint8_t addr;
    ER er = chip(&addr);

    return (E_OK == er) ?
           read_regs(addr, (n > 1u) ? (reg | SUB_AUTO_INC) : reg, p_buf, n) :
           er;
}

EXPORT ER
drv_imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t addr;
    ER er = chip(&addr);

    return (E_OK == er) ? write_reg(addr, reg, val) : er;
}

EXPORT ER
drv_imu_configure(void)
{
    static uint8_t const ctrl[CTRL_REGS] =
    {
        CTRL0_VAL, CTRL1_VAL, CTRL2_VAL, CTRL3_VAL,
        CTRL4_VAL, CTRL5_VAL, CTRL6_VAL, CTRL7_VAL
    };
    uint8_t rb[CTRL_REGS];
    uint8_t val;
    uint8_t addr;
    uint32_t i;
    ER er;

    er = chip(&addr);
    if (E_OK != er)
    {
        return er;                      /* never write to an unknown device */
    }
    for (i = 0u; i < CTRL_REGS; i++)
    {
        er = write_reg(addr, (uint8_t)(REG_CTRL0 + i), ctrl[i]);
        if (E_OK != er)
        {
            return er;
        }
    }
    er = write_reg(addr, REG_FIFO_CTRL, FIFO_BYPASS);  /* empty      */
    if (E_OK == er)
    {
        er = write_reg(addr, REG_FIFO_CTRL, FIFO_STREAM);
    }
    if (E_OK != er)
    {
        return er;
    }

    /* Read-back: every register written must hold its value (FR20) */
    er = read_regs(addr, REG_CTRL0 | SUB_AUTO_INC, rb, CTRL_REGS);
    if (E_OK != er)
    {
        return er;
    }
    for (i = 0u; i < CTRL_REGS; i++)
    {
        if (rb[i] != ctrl[i])
        {
            return E_IO;
        }
    }
    er = read_regs(addr, REG_FIFO_CTRL, &val, 1u);
    if ((E_OK == er) && (FIFO_STREAM != (val & 0xE0u)))
    {
        er = E_IO;
    }
    return er;
}

static ER
fifo_level(uint8_t addr, uint8_t *p_n, bool *p_b_ovr)
{
    uint8_t src;
    ER er;

    er = read_regs(addr, REG_FIFO_SRC, &src, 1u);
    if (E_OK != er)
    {
        return er;
    }
    /* FSS counts unread samples 0..31; a full FIFO shows as OVRN */
    *p_b_ovr = (0u != (src & SRC_OVRN));
    if (*p_b_ovr)
    {
        *p_n = (uint8_t)DRV_IMU_FIFO_DEPTH;
    }
    else if (0u != (src & SRC_EMPTY))
    {
        *p_n = 0u;
    }
    else
    {
        *p_n = (uint8_t)(src & SRC_FSS);
    }
    return E_OK;
}

static void
unpack(uint8_t const *p_bytes, drv_imu_raw_t *p_raw)
{
    uint32_t k;

    for (k = 0u; k < 3u; k++)
    {
        p_raw->v[k] = le16(&p_bytes[2u * k]);
    }
}

EXPORT ER
drv_imu_read_acc(drv_imu_raw_t *p_buf, uint8_t max, uint8_t *p_n,
                 bool *p_b_ovr)
{
    uint8_t avail;
    uint8_t addr;
    uint32_t n;
    uint32_t i;
    ER er;

    *p_n = 0u;
    *p_b_ovr = false;
    er = chip(&addr);
    if (E_OK == er)
    {
        er = fifo_level(addr, &avail, p_b_ovr);
    }
    if (E_OK != er)
    {
        return er;
    }
    if (*p_b_ovr)
    {
        count(&g_stats.fifo_overruns);
    }
    n = avail;
    if (n > max)
    {
        n = max;
    }
    if (n > BURST_MAX_SAMPLES)
    {
        n = BURST_MAX_SAMPLES;
    }
    if (0u == n)
    {
        return E_OK;
    }

#if S4_CFG_FIFO_BURST
    /* All samples in one transfer: needs the 2Dh -> 28h wrap that the
       Stage 2 burst check confirms */
    er = read_regs(addr, REG_OUT_X_L_A | SUB_AUTO_INC, g_burst,
                   n * SAMPLE_BYTES);
#else
    /* One 6-byte burst per sample; each read of OUT_*_A pops the oldest
       sample (section 4.3, "Retrieving data from FIFO") */
    for (i = 0u; (i < n) && (E_OK == er); i++)
    {
        er = read_regs(addr, REG_OUT_X_L_A | SUB_AUTO_INC,
                       &g_burst[i * SAMPLE_BYTES], SAMPLE_BYTES);
    }
#endif
    if (E_OK != er)
    {
        return er;
    }
    for (i = 0u; i < n; i++)
    {
        unpack(&g_burst[i * SAMPLE_BYTES], &p_buf[i]);
    }
    *p_n = (uint8_t)n;
    return E_OK;
}

EXPORT ER
drv_imu_burst_check(drv_imu_raw_t *p_buf, uint8_t n, uint8_t *p_before,
                    uint8_t *p_after)
{
    bool b_ovr;
    uint8_t addr;
    uint32_t i;
    ER er;

    *p_before = 0u;
    *p_after = 0u;
    if ((0u == n) || (n > BURST_MAX_SAMPLES))
    {
        return E_PAR;
    }
    er = chip(&addr);
    if (E_OK == er)
    {
        er = fifo_level(addr, p_before, &b_ovr);
    }
    if ((E_OK == er) && (*p_before < n))
    {
        er = E_PAR;                 /* not enough samples waiting        */
    }
    if (E_OK == er)
    {
        er = read_regs(addr, REG_OUT_X_L_A | SUB_AUTO_INC, g_burst,
                       (uint32_t)n * SAMPLE_BYTES);
    }
    if (E_OK == er)
    {
        er = fifo_level(addr, p_after, &b_ovr);
    }
    if (E_OK == er)
    {
        for (i = 0u; i < n; i++)
        {
            unpack(&g_burst[i * SAMPLE_BYTES], &p_buf[i]);
        }
    }
    return er;
}

EXPORT ER
drv_imu_read_mag(drv_imu_raw_t *p_raw)
{
    uint8_t b[SAMPLE_BYTES];
    uint8_t addr;
    ER er;

    /* X, Y, Z, each low byte first (Table 16) */
    er = chip(&addr);
    if (E_OK == er)
    {
        er = read_regs(addr, REG_OUT_X_L_M | SUB_AUTO_INC, b, SAMPLE_BYTES);
    }
    if (E_OK == er)
    {
        unpack(b, p_raw);
    }
    return er;
}

EXPORT void
drv_imu_get_stats(drv_imu_stats_t *p_stats)
{
    *p_stats = g_stats;
}
