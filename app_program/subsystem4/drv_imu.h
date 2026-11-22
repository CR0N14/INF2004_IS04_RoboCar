/*
 *  drv_imu.h
 *  LSM303D driver (GY-511 board) on I2C1: GP2 = SDA, GP3 = SCL.
 *
 *  The accelerometer and the magnetometer share one I2C address, set by
 *  the SA0 pin: 0x1D (high) or 0x1E (low) (datasheet DocID023312 Rev 2,
 *  section 6.1.1).  Our first board answered at 0x1D, the second at 0x1E,
 *  so drv_imu_find() looks at both and accepts only an address whose
 *  WHO_AM_I reads 49h.  Until then every access to the chip, and every
 *  write in particular, is refused (E_OBJ).  After a bus timeout the
 *  driver re-reads WHO_AM_I before it next touches the chip.
 *
 *  Polled, register-level.  Only the IMU task calls it: the task is the
 *  sole owner of I2C1 (TEAM.md section 3).  The kernel's own I2C driver
 *  stays off (DEVCNF_USE_IIC 0), so I2C0 never starts and GP8/GP9 stay
 *  with the motor driver.
 *
 *  Every transfer has a timeout.  A timed-out transfer clears the bus
 *  (up to 9 SCL pulses and a STOP) and re-initialises I2C1 before
 *  returning.  Return codes: E_OK, E_IO (no acknowledge), E_TMOUT,
 *  E_PAR, E_OBJ (no LSM303D identified).
 *
 *  Chip set-up (drv_imu_configure):
 *    accelerometer  400 Hz, +-4 g (0.122 mg/LSB), anti-alias filter
 *                   50 Hz, block data update, FIFO in stream mode
 *    magnetometer   100 Hz continuous, +-2 gauss (0.080 mgauss/LSB),
 *                   high resolution
 */

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <tk/tkernel.h>
#include <stdint.h>
#include <stdbool.h>

#define DRV_IMU_ADDR_SA0_HIGH   0x1Du   /* 0011101b (Table 11)           */
#define DRV_IMU_ADDR_SA0_LOW    0x1Eu   /* 0011110b                      */
#define DRV_IMU_NO_ADDR         0x00u   /* not identified                */
#define DRV_IMU_WHO_AM_I        0x49u   /* register 0Fh (Table 19)       */
#define DRV_IMU_FIFO_DEPTH      32u     /* FSS[4:0] 0..31, OVRN = full   */

/* Raw sample: sensor axes X, Y, Z (not yet mapped to the car) */
typedef struct
{
    int16_t v[3];
} drv_imu_raw_t;

/* Running counters since boot (saturate at UINT32_MAX) */
typedef struct
{
    uint32_t xfers;         /* transfers attempted (bus scans excluded)  */
    uint32_t fails;         /* failed transfers (any reason)             */
    uint32_t nacks;         /* ... of which no acknowledge               */
    uint32_t timeouts;      /* ... of which timed out                    */
    uint32_t recoveries;    /* bus clears + re-initialisations           */
    uint32_t fifo_overruns; /* accelerometer FIFO was full when read     */
    uint32_t max_xfer_us;   /* longest single transfer                   */
    uint32_t probes;        /* bus-scan probes (an empty address is not  */
                            /*   a failure, so they are counted apart)   */
} drv_imu_stats_t;

/* Set up I2C1 and the two pins, clearing the bus first. */
void    drv_imu_init(void);

/* Bus clear and re-initialisation after a lost sensor.  Also forgets
   the address: call drv_imu_find() before using the chip again. */
void    drv_imu_recover(void);

/* true if a device acknowledges at 7-bit address addr (bus scan). */
bool    drv_imu_probe(uint8_t addr);

/* Read WHO_AM_I (0Fh) at any address: a read only, for the scan
   readout.  E_IO if nothing answers there. */
ER      drv_imu_whoami_at(uint8_t addr, uint8_t *p_id);

/* Look for the LSM303D at 0x1D, then 0x1E.  E_OK and *p_addr set if an
   address answers with WHO_AM_I 49h; E_IO and DRV_IMU_NO_ADDR if not. */
ER      drv_imu_find(uint8_t *p_addr);

/* The identified address, or DRV_IMU_NO_ADDR */
uint8_t drv_imu_addr(void);

/* Accelerometer and magnetometer set-up, then read-back check.
   E_IO if a register does not read back as written; E_OBJ before
   drv_imu_find() has succeeded. */
ER      drv_imu_configure(void);

/* Read up to max samples from the accelerometer FIFO.  *p_n = samples
   read (0 if none new), *p_b_ovr = the FIFO had overflowed. */
ER      drv_imu_read_acc(drv_imu_raw_t *p_buf, uint8_t max, uint8_t *p_n,
                         bool *p_b_ovr);

/* Latest magnetometer sample, X, Y, Z. */
ER      drv_imu_read_mag(drv_imu_raw_t *p_raw);

/* Register access for the bring-up and diagnostic readouts only.  Never
   use them on reserved registers (00h-04h, 0Eh, 10h-11h). */
ER      drv_imu_read_reg(uint8_t reg, uint8_t *p_val);
ER      drv_imu_read_regs(uint8_t reg, uint8_t *p_buf, uint8_t n);
ER      drv_imu_write_reg(uint8_t reg, uint8_t val);

/* FIFO burst check for the bring-up readout: reads n samples in ONE
   transfer and reports the FIFO level before and after.  If the output
   pointer wraps from 2Dh to 28h, the level drops by n; if not, by 1. */
ER      drv_imu_burst_check(drv_imu_raw_t *p_buf, uint8_t n,
                            uint8_t *p_before, uint8_t *p_after);

void    drv_imu_get_stats(drv_imu_stats_t *p_stats);

/* Free-running microsecond counter (RP2040 TIMER, wraps every 71 min). */
uint32_t drv_imu_now_us(void);

#endif /* DRV_IMU_H */
