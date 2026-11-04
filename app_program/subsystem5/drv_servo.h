/* Local SG90 driver. Call only from the S5 Ultrasound task. */
#ifndef S5_DRV_SERVO_H
#define S5_DRV_SERVO_H

#include "../app.h"

ER s5_servo_init(void);
ER s5_servo_set_angle(UINT angle_deg);

#endif /* S5_DRV_SERVO_H */
