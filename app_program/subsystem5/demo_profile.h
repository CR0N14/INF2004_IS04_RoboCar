#ifndef DEMO_PROFILE_H
#define DEMO_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Obstacle width from the servo-mounted ultrasonic sensor.
 *
 * A coarse sweep finds each side of the obstacle to within 10 degrees, then a
 * fine scan inside that bracket places the corner to within 2 degrees. Width
 * is the lateral distance between the two corners.
 *
 * The caller owns the hardware: demo_range_fn must point the servo at
 * angle_deg, wait for it to settle, trigger the sensor and return the range in
 * cm. 0 or any value above 400 means no echo. 90 degrees is straight ahead and
 * lower angles are the left.
 */
typedef uint16_t (*demo_range_fn)(void *p_ctx, unsigned int angle_deg);

typedef struct
{
    uint16_t width_cm;      /* corner to corner                              */
    uint16_t ahead_cm;      /* range straight ahead                          */
    uint8_t  left_deg;      /* servo angle of the left corner                */
    uint8_t  right_deg;     /* servo angle of the right corner               */
    uint8_t  bearing_deg;   /* servo angle midway between the corners        */
    uint8_t  readings;      /* sensor readings used, including retries       */
} demo_width_t;

/* Sweep limits and step sizes, in degrees from straight ahead. */
#define DEMO_SCAN_HALF_DEG      60u
#define DEMO_COARSE_STEP_DEG    10u
#define DEMO_FINE_STEP_DEG      2u

/* A reading belongs to the obstacle when its depth (range x cos(angle)) is no
   more than this much beyond the depth straight ahead. */
#define DEMO_HIT_TOL_CM         8u

/* The obstacle must be this close straight ahead to be measured. */
#define DEMO_AHEAD_MAX_CM       35u

/*
 * Half the sensor's beam angle, in whole degrees. The beam is a cone, so an
 * edge shows up while the axis is still this far outside the real corner and
 * both corners read too wide. PROVISIONAL: 4 is a middle value that was not
 * measured on this car. To calibrate, scan a box of known width and read
 * left_deg and right_deg from the scan log. The value to use is
 * atan((measured - true) / 2 / range) in degrees, added to this one.
 */
#ifndef DEMO_BEAM_HALF_DEG
#define DEMO_BEAM_HALF_DEG      4u
#endif

/* Returns false if the obstacle cannot be bracketed: nothing close ahead, a
   corner beyond the sweep, or no usable readings. p_out is then unchanged. */
bool demo_measure_width(demo_range_fn read, void *p_ctx, demo_width_t *p_out);

#endif /* DEMO_PROFILE_H */
