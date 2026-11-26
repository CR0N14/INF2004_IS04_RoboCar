/* S5 sonar access for the S2 obstacle bypass (DEMO=1 only).
   S5 owns the servo and sensor; S2 only requests an aim and reads results. */
#ifndef S5_DEMO_SONAR_H
#define S5_DEMO_SONAR_H
#include "../app.h"
#include <stdint.h>
/* Servo angle wanted during the bypass (0..180, 90 = ahead). */
void s5_demo_set_aim(uint8_t angle_deg);
/* Latest reading: distance (511 = no echo), the angle it was taken at, and
   the S5 clock time (ms) it was taken. */
ER s5_demo_reading(uint16_t *distance_cm,uint8_t *angle_deg,uint32_t *time_ms);
#endif
