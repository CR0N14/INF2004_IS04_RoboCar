#include "demo_pickup.h"
#include "../demo_config.h"
bool demo_pickup_step(demo_pickup_t *s,uint32_t now,int32_t pitch,int32_t roll,bool running,bool impact){
 if(running){
  bool tilted=pitch>DEMO_PICKUP_TILT_DEG*100||pitch< -DEMO_PICKUP_TILT_DEG*100||roll>DEMO_PICKUP_TILT_DEG*100||roll< -DEMO_PICKUP_TILT_DEG*100;
  if(tilted){if(!s->active){s->active=true;s->since=now;}if(now-s->since>=DEMO_PICKUP_HOLD_MS)s->latched=true;}else s->active=false;
  if(impact)s->latched=true;
 }else s->active=false;
 return s->latched;
}
