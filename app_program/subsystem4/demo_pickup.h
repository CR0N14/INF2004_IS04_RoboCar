#ifndef DEMO_PICKUP_H
#define DEMO_PICKUP_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {uint32_t since;bool active,latched;} demo_pickup_t;
bool demo_pickup_step(demo_pickup_t*,uint32_t,int32_t,int32_t,bool,bool);
#endif
