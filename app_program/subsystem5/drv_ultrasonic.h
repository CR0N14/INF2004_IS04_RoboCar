/* Local ultrasonic driver. Call only from the S5 Ultrasound task. */
#ifndef S5_DRV_ULTRASONIC_H
#define S5_DRV_ULTRASONIC_H

#include "subsystem5.h"

ER s5_ultrasonic_init(void);
/* Returns 1..400 cm, or S5_NO_ECHO_CM on timeout/invalid measurement.
   Caller must leave at least 60 ms between triggers. */
UINT s5_ultrasonic_read_cm(void);

#endif /* S5_DRV_ULTRASONIC_H */
