/*
 *  imu_console.h
 *  Subsystem 4 console output (imu_cfg.h S4_CFG_CONSOLE).
 *
 *  Called only from the IMU task.  tm_printf() never blocks: with no
 *  terminal attached the USB console drops the text and counts it.
 */

#ifndef IMU_CONSOLE_H
#define IMU_CONSOLE_H

#include "../app.h"
#include "imu_logic.h"
#include "imu_task.h"

/* Stage 2 bring-up (S4_CONSOLE_RAW).  Never returns.  tick() must block
   until the next 10 ms cycle. */
void    imu_console_raw(void (*tick)(void));

/* Accelerometer diagnosis (S4_CONSOLE_ACCDIAG).  Never returns. */
void    imu_console_accdiag(void (*tick)(void));

/* Always printed (one line each, rare): self-test result, IMU lost */
void    imu_console_selftest(bool b_ok, uint8_t cal_result, int32_t g_mg,
                             uint8_t addr, uint32_t now_ms);
void    imu_console_fault(ER er, uint32_t now_ms);

/* Bench readout only (S4_CONSOLE_BENCH) */
void    imu_console_status(imu_view_t const *p_view, uint8_t status,
                           uint16_t spike_mg, imu_task_dbg_t const *p_dbg,
                           uint32_t now_ms);
void    imu_console_hump(imu_hump_t const *p_hump, uint32_t now_ms);
void    imu_console_collision(uint16_t spike_mg, bool b_sent,
                              uint32_t now_ms);
void    imu_console_tipover(int32_t pitch_cdeg, int32_t roll_cdeg,
                            bool b_sent, uint32_t now_ms);

#endif /* IMU_CONSOLE_H */
