/*
 *  subsystem4.c
 *  Subsystem 4 -- IMU-Based Motion and Terrain Monitoring.
 *
 *  One task (imu_task.c, priority PRI_IMU) owns the GY-511 on I2C1
 *  (drv_imu.c).  Its pure processing (tilt, heading, turn rate, motion
 *  class, humps, collision, tip-over) is in imu_logic.c, with integer
 *  maths in imu_fixmath.c, so it can be unit-tested on a PC
 *  (testing/subsystem4/host/).
 *
 *  Kernel objects: 1 task, 1 cyclic handler (10 ms), 1 mutex (snapshot).
 *  Build switches: imu_cfg.h.  Values that need the real car:
 *  imu_params.h.
 */

#include "subsystem4.h"
#include "imu_task.h"

EXPORT ER
subsystem4_init(void)
{
    return imu_task_init();
}

EXPORT ER
subsystem4_start(void)
{
    return imu_task_start();
}
