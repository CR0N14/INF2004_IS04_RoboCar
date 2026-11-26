/* Refined demo obstacle profile (DEMO_SONAR_REFINED): simulated servo and
   ultrasonic sensor (nearest echo in a +-DEMO_BEAM_HALF_DEG cone, +-1 cm
   noise, optional dropouts) facing a flat box, with optional side walls
   parallel to the line. Compares the refined width with the 25-point sweep. */
#define _GNU_SOURCE
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../app_program/subsystem5/demo_profile.h"
#include "../../app_program/subsystem1/sm_types.h"
#include "../../app_program/demo_config.h"
static unsigned checks;
#define CHECK(x) do{assert(x);++checks;}while(0)
typedef struct {double depth,width,centre,low_wall,high_wall,back,dropout;unsigned seed,reads,last,ms;} world_t;
static unsigned rnd(world_t *w){w->seed=w->seed*1664525u+1013904223u;return w->seed>>8;}
/* off: degrees from straight ahead, negative = low servo angles (x < 0). */
static double ray(world_t const *w,double off){
 double a=off*M_PI/180.0,c=cos(a),s=sin(a),best=1e9;
 if(c>1e-6){double r=w->depth/c,x=r*s;
  if(x>=w->centre-w->width/2&&x<=w->centre+w->width/2)best=r;
  if(w->back>0&&w->back/c<best)best=w->back/c;}
 if(s< -1e-6&&w->low_wall>0&&w->low_wall/-s<best)best=w->low_wall/-s;
 if(s>1e-6&&w->high_wall>0&&w->high_wall/s<best)best=w->high_wall/s;
 return best;
}
static uint16_t sense(world_t *w,double off){
 double best=1e9;
 for(double d=-(double)DEMO_BEAM_HALF_DEG;d<=(double)DEMO_BEAM_HALF_DEG+1e-9;d+=0.25){double r=ray(w,off+d);if(r<best)best=r;}
 if(best>400.0)return 511u;
 if(w->dropout>0&&(rnd(w)%1000u)<(unsigned)(w->dropout*1000))return 511u;
 return (uint16_t)lround(best+(double)((int)(rnd(w)%3u)-1));
}
/* Time as on the car: servo settle 100 ms + 3 ms/deg on a move, 60 ms on a
   repeat (60 ms), then the echo (round trip, or the 30 ms timeout). */
static uint16_t read_world(void *p,unsigned angle){world_t *w=(world_t*)p;w->reads++;
 unsigned sw=angle>w->last?angle-w->last:w->last-angle;w->ms+=sw?100u+3u*sw:60u;w->last=angle;
 uint16_t r=sense(w,(double)angle-90.0);w->ms+=r>400u?30u:(unsigned)(r*58u/1000u);return r;}
static world_t box(double depth,double width){world_t w={depth,width,0,0,0,0,0,7u,0,90u,0};return w;}
static int old_width(world_t w,uint16_t *out){
 uint16_t d[25];for(int i=0;i<25;i++)d[i]=sense(&w,(double)(30+5*i)-90.0);
 return demo_profile_width(d,out);
}
int main(void){
 demo_profile_t p;demo_width_t r;setvbuf(stdout,NULL,_IONBF,0);
 /* 15 cm box at 25 cm, open space: width close, both sides far, tie -> left. */
 world_t w=box(25,15);
 CHECK(demo_profile_measure(read_world,&w,&p,&r));
 printf("15 cm box at 25 cm: refined %u cm (corners %u..%u, %u reads)",p.width_cm,r.left_deg,r.right_deg,w.reads);
 uint16_t ow=0;if(old_width(box(25,15),&ow))printf(", 25-point sweep %u cm\n",ow);else printf(", 25-point sweep failed\n");
 CHECK(abs((int)p.width_cm-15)<=3);
 CHECK(p.left_cm==400&&p.right_cm==400&&demo_profile_choose(&p)==SM_AVOID_LEFT);
 CHECK(p.closest_cm>=24&&p.closest_cm<=27&&r.bearing_deg>=88&&r.bearing_deg<=92);
 /* 25-point sweep: 90->30 (280 ms), 24 x 5-deg steps (115 ms), back to 90 (280 ms), echoes. */
 w.ms+=100u+3u*(w.last>90u?w.last-90u:90u-w.last); /* aim back ahead */
 printf("scan time estimate: refined %u ms, 25-point sweep about %u ms\n",w.ms,280u+24u*115u+280u+25u*30u);
 CHECK(w.reads<=40u&&w.ms<=280u+24u*115u+280u+25u*30u); /* not slower */
 /* Several sizes and ranges stay within 3 cm. */
 double const sz[][2]={{20,10},{25,20},{20,30},{30,25},{18,12}};
 for(unsigned i=0;i<sizeof sz/sizeof sz[0];i++){
  world_t v=box(sz[i][0],sz[i][1]);
  CHECK(demo_profile_measure(read_world,&v,&p,NULL));
  printf("box %2.0f cm at %2.0f cm: refined %u cm",sz[i][1],sz[i][0],p.width_cm);
  if(old_width(box(sz[i][0],sz[i][1]),&ow))printf(", 25-point %u cm\n",ow);else printf(", 25-point failed\n");
  CHECK(abs((int)p.width_cm-(int)sz[i][1])<=3);
 }
 /* Wall 25 cm to the low-angle side: that side is not clear. */
 w=box(25,15);w.low_wall=25;
 CHECK(demo_profile_measure(read_world,&w,&p,NULL));
 uint16_t low=DEMO_SERVO_LEFT_LOW?p.left_cm:p.right_cm,high=DEMO_SERVO_LEFT_LOW?p.right_cm:p.left_cm;
 CHECK(low>=26&&low<=32&&high==400);
 CHECK(demo_profile_choose(&p)==(DEMO_SERVO_LEFT_LOW?SM_AVOID_RIGHT:SM_AVOID_LEFT));
 /* Walls close on both sides: no way round. */
 w=box(25,15);w.low_wall=22;w.high_wall=22;
 CHECK(demo_profile_measure(read_world,&w,&p,NULL));
 CHECK(demo_profile_choose(&p)==SM_AVOID_STOP);
 /* Background wall far behind still counts as clear beyond the corners. */
 w=box(25,15);w.back=120;
 CHECK(demo_profile_measure(read_world,&w,&p,NULL)&&p.left_cm>=DEMO_SIDE_CLEAR_CM&&p.right_cm>=DEMO_SIDE_CLEAR_CM);
 /* 10% dropped echoes: retries keep the result for most seeds. */
 unsigned ok=0;
 for(unsigned s=1;s<=20;s++){world_t v=box(25,15);v.dropout=0.10;v.seed=s;
  if(demo_profile_measure(read_world,&v,&p,NULL)&&abs((int)p.width_cm-15)<=4)ok++;}
 printf("10%% dropouts: %u/20 within 4 cm\n",ok);
 CHECK(ok>=17u);
 /* Refused: nothing close ahead, a corner beyond the sweep, no echoes. */
 w=box(60,15);CHECK(!demo_profile_measure(read_world,&w,&p,NULL));
 w=box(20,120);CHECK(!demo_profile_measure(read_world,&w,&p,NULL));
 w=box(25,15);w.dropout=1.0;CHECK(!demo_profile_measure(read_world,&w,&p,NULL));
 CHECK(!demo_profile_measure(NULL,&w,&p,NULL));
 printf("Refined sonar profile: %u checks passed\n",checks);
 return 0;
}
