#ifndef DEMO_STEERING_H
#define DEMO_STEERING_H
#include "../demo_config.h"
/* Darkness 0..4095. Rightward error is inferred, not directly measured.
   Both-black remains a junction input and both-white uses bounded recovery. */
static inline int demo_analogue_steer(unsigned left,unsigned centre,unsigned mask){
 if((mask&3u)==3u || (mask&3u)==0u)return 0;
 if((mask&3u)==1u)return DEMO_LINE_CORRECTION;
 if(left>600u){unsigned v=(left-600u)*DEMO_LINE_CORRECTION/1800u;
  return (int)(v>DEMO_LINE_CORRECTION?DEMO_LINE_CORRECTION:v);}
 if(centre<3000u){unsigned v=(3000u-centre)*20u/1400u;return -(int)(v>20u?20u:v);}
 return 0;
}
/* Smooth tracking: hold the centre sensor at DEMO_TRACK_TARGET (the line
   edge), so drift either way gives a signed error; code-left darkness means
   the line is moving towards physical right and calls for a right turn. */
static inline int demo_track_steer(unsigned left,unsigned centre,unsigned mask){
 if((mask&3u)==1u)return DEMO_TRACK_MAX_STEER;
 if((mask&3u)!=2u)return 0;
 int v=((int)centre-(int)DEMO_TRACK_TARGET)*DEMO_TRACK_KP/1000;
 if(left>DEMO_TRACK_LEFT_ON)v+=(int)(left-DEMO_TRACK_LEFT_ON)*DEMO_TRACK_KL/1000;
 return v>DEMO_TRACK_MAX_STEER?DEMO_TRACK_MAX_STEER:(v< -DEMO_TRACK_MAX_STEER?-DEMO_TRACK_MAX_STEER:v);
}
#endif
