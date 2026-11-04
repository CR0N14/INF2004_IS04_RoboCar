/** @file code39.h
 * @brief One uppercase Code 39 character between start/stop guards.
 */
#ifndef CODE39_H
#define CODE39_H
#include "barcode_decoder.h"

#define CODE39_RUN_COUNT 29u
/** Decode either scan orientation; output is A=0 through Z=25. */
bool code39_recognise (void * p_context, uint16_t const * p_runs, uint8_t count,
                       barcode_direction_t direction, uint8_t * p_symbol);
#endif
/* End of file. */

