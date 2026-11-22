#ifndef DEMO_PROFILE_H
#define DEMO_PROFILE_H
#include <stdbool.h>
#include <stdint.h>
/* 25 samples at 30..150 degrees, 90 forward. 0/511 invalid. */
bool demo_profile_width(uint16_t const distances[25],uint16_t *width);
#endif
