#ifndef DEMO_IMU_H
#define DEMO_IMU_H
#include "subsystem4.h"
#include <stdbool.h>
#include <stdint.h>
typedef struct {uint32_t time_ms;int16_t mg[3];uint16_t peak_mm;bool ready,latched,height_valid;} demo_imu_t;
ER demo_imu_read(demo_imu_t*);
bool demo_imu_ready(void);
bool demo_imu_latched(void);
ER demo_imu_zero(void);
#endif
