/*
 *  drv_encoder.h
 *  Count the two quadrature (A/B) wheel encoders.
 *
 *  The interrupt handler is the ONLY writer of the counters.  Each counter
 *  is its own full 32-bit word (team decision, Tribunal 1), so a read on
 *  either core can never catch it half-written, and no packed neighbour can
 *  overwrite it.
 *
 *  Ported from z_motion_control/encoder.c.  The decoding is the same; only
 *  the way the interrupt is installed changed (see drv_encoder.c).
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>
#include "../app.h"
#include "motion_config.h"

/* drv_encoder_qdec() result for an impossible step (both bits changed) */
#define QDEC_ERROR      2

/* Set up the encoder pins and install the GPIO interrupt.  Call once at
 * start-up, from task context (it is called from subsystem2_init()). */
ER drv_encoder_init(void);

/* Total counts since start-up.  Forward = counts go up.  x4 decoding: every
 * edge on A and on B adds or subtracts 1.  Wraps after 2^31 counts, so
 * always work with differences: (now - before) is right even across a
 * wrap.  There is deliberately no reset: a reset from a task could race
 * with the interrupt running on the other core. */
int32_t drv_encoder_count(wheel_t wheel);

/* Impossible transitions seen (A and B changed at once).  Above zero means
 * edges came faster than the interrupt could handle, or there is noise:
 * pulses may have been missed (NFR7). */
uint32_t drv_encoder_errors(wheel_t wheel);

/* One quadrature step: prev and now are 2-bit states (A << 1 | B).
 * Returns +1, -1, 0 (no change) or QDEC_ERROR.  Pure function; public so
 * the decoding table can be unit-tested. */
INT drv_encoder_qdec(uint8_t prev, uint8_t now);

/* TRUE if the interrupt is enabled for all four encoder pins.  For tests. */
BOOL drv_encoder_irq_enabled(void);

#endif /* DRV_ENCODER_H */
