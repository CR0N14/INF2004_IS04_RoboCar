#ifndef DEMO_IMU_H
#define DEMO_IMU_H
#include "subsystem4.h"
#include <stdbool.h>
#include <stdint.h>
/* heading_cdeg: tilt-compensated magnetometer heading, 0..35999, clockwise
   from magnetic north. heading_ok: a heading arrived within 100 ms. */
typedef struct {uint32_t time_ms;int16_t mg[3];uint16_t peak_mm;int32_t heading_cdeg;bool ready,latched,height_valid,heading_ok;} demo_imu_t;
ER demo_imu_read(demo_imu_t*);
bool demo_imu_ready(void);
bool demo_imu_latched(void);
ER demo_imu_zero(void);
#endif
