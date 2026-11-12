/*
 *  imu_task.h
 *  IMU task: kernel objects and the 10 ms task.  Internal to Subsystem
 *  4; other subsystems use subsystem4.h (s4_get_tlm()).
 */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

#define IMU_CYCLE_MS            10u     /* NFR14: accelerometer every 10 ms */

/* Task modes: the top level of the IMU statechart (design review 6.4).
   SUDDEN IMPACT is a 500 ms latch inside MONITORING (imu_logic.c). */
typedef enum
{
    IMU_MODE_SELFTEST = 0,      /* SELF-TEST & CALIBRATE                 */
    IMU_MODE_IDLE,              /* SENSORS_ENABLED clear: no reads        */
    IMU_MODE_MONITOR,           /* MONITORING                            */
    IMU_MODE_FAULT              /* SENSOR FAULT: data invalid, retrying  */
} imu_mode_t;

/* Counters for the on-target tests and the bench readout */
typedef struct
{
    uint8_t  mode;              /* imu_mode_t                            */
    uint8_t  addr;              /* I2C address found (0x1D / 0x1E), 0    */
    bool     b_selftest_ok;     /* last self-test passed                 */
    bool     b_ready_sent;      /* SM_EVT_READY posted                   */
    bool     b_calibrated;      /* at-rest alignment done                */
    int32_t  g_mg;              /* |g| measured at rest                  */
    uint8_t  cal_result;        /* imu_cal_result_t of the last attempt  */
    uint32_t cycles;            /* 10 ms cycles run                      */
    uint32_t missed;            /* cycles lost to overload               */
    uint32_t read_fails;        /* failed read cycles since boot         */
    uint32_t faults;            /* times SENSOR FAULT was entered        */
    uint32_t logic_us_max;      /* NFR20: processing, I2C excluded       */
    uint32_t bus_us_max;        /* I2C time in one cycle                 */
    uint32_t sm_posts_lost;     /* control events dropped (queue full)   */
} imu_task_dbg_t;

/* Create the cyclic handler and the snapshot mutex (subsystem4_init). */
ER      imu_task_init(void);

/* Create and start the IMU task and its cyclic handler
   (subsystem4_start). */
ER      imu_task_start(void);

/* Copy of the counters (tests, console) */
void    imu_task_debug(imu_task_dbg_t *p_dbg);

#endif /* IMU_TASK_H */
