/* Smooth tracking (DEMO_SMOOTH_TRACK=1): proportional steering on the line,
   slowing with the steer, damping per IR sample, and a ramped arc that ends
   on either tracing sensor. Synthetic darkness; no physical claims. */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem2/demo_motion.h"
#include "../../app_program/subsystem2/demo_steering.h"
static unsigned checks;
#define CHECK(x) do {assert(x);++checks;}while(0)
static uint32_t st(sm_nav_mode_t nav){return SM_STATE_NAVIGATING|((uint32_t)nav<<3);}
static demo_output_t at(demo_motion_t *c,uint32_t now,uint32_t sample,uint8_t mask,uint16_t left,uint16_t centre){
 demo_line_t s={.time_ms=sample,.mask=mask,.calibrated=true,.valid=true,.dark={left,centre,0}};
 return demo_motion_step(c,now,st(SM_NAV_TRACKING),&s,0,false);
}
static demo_output_t step(demo_motion_t *c,uint32_t t,uint8_t mask,uint16_t left,uint16_t centre){return at(c,t,t,mask,left,centre);}
int main(void){
 demo_motion_t c; demo_output_t o;
 /* Pure steer: signs, ignored masks, code-left only at the limit. */
 CHECK(demo_track_steer(0,DEMO_TRACK_TARGET,2)==0);
 CHECK(demo_track_steer(0,4095,2)>0);
 CHECK(demo_track_steer(0,1800,2)<0);
 CHECK(demo_track_steer(3000,2500,2)>0);
 CHECK(demo_track_steer(4095,4095,3)==0&&demo_track_steer(0,0,0)==0);
 CHECK(demo_track_steer(4095,0,1)==DEMO_TRACK_MAX_STEER);
 for(unsigned l=0;l<=4095;l+=455)for(unsigned m=0;m<=4095;m+=455){
  int v=demo_track_steer(l,m,2);CHECK(v>=-DEMO_TRACK_MAX_STEER&&v<=DEMO_TRACK_MAX_STEER);
 }
 /* On the edge: straight at base duty. */
 c=(demo_motion_t){0};o=step(&c,0,2,0,DEMO_TRACK_TARGET);CHECK(o.right==14&&o.left==14&&o.forward);
 /* Deep on the line: line moving towards code-left, slow the right wheel. */
 c=(demo_motion_t){0};o=step(&c,0,2,0,4095);CHECK(o.right==11&&o.left==14);
 /* Centre fading: slow the left wheel and the car. */
 c=(demo_motion_t){0};o=step(&c,0,2,0,1800);CHECK(o.right==14&&o.left==10);
 /* Code-left darkening wins over a fading centre. */
 c=(demo_motion_t){0};o=step(&c,0,2,3000,2500);CHECK(o.right==5&&o.left==14&&o.forward);
 /* Code-left only: steering limit, minimum duty, still forward. */
 c=(demo_motion_t){0};o=step(&c,0,1,4095,0);CHECK(o.right==4&&o.left==DEMO_TRACK_MIN_DUTY&&o.forward&&!o.event);
 /* Damping adds the P change once per IR sample and holds between samples. */
 c=(demo_motion_t){0};step(&c,0,2,0,3000);
 o=step(&c,10,2,0,2500);CHECK(o.left==14*85/100); /* P 10 + D 5 */
 o=at(&c,15,10,2,0,2500);CHECK(o.left==14*85/100);
 o=step(&c,20,2,0,2500);CHECK(o.left==14*90/100);  /* P 10, D 0 */
 /* Ramped arc after a centre-side loss, LINE_LOST only after 720 ms. */
 c=(demo_motion_t){0};step(&c,0,2,0,1800);
 o=step(&c,10,0,0,0);CHECK(o.right==14&&o.left==14&&!o.event);
 o=step(&c,30,0,0,0);CHECK(o.right==14&&o.left==14*67/100);
 o=step(&c,230,0,0,0);CHECK(o.left==14*39/100&&o.forward);
 o=step(&c,430,0,0,0);CHECK(o.left==1&&o.right==14&&o.forward&&!o.event);
 o=step(&c,729,0,0,0);CHECK(!o.event&&o.forward);
 o=step(&c,730,0,0,0);CHECK(o.event==SM_EVT_LINE_LOST&&!o.right&&!o.left);
 /* Either sensor ends the arc; the code-left catch latches the next arc. */
 c=(demo_motion_t){0};step(&c,0,2,0,1800);step(&c,10,0,0,0);
 o=step(&c,200,0,0,0);CHECK(o.right>o.left);
 o=step(&c,210,1,4095,0);CHECK(o.right==4&&o.left==14&&!o.event);
 o=step(&c,220,0,0,0);CHECK(o.right==14&&o.left==14);
 o=step(&c,240,0,0,0);CHECK(o.right==14*67/100&&o.left==14);
 /* Drift towards code-left before a loss arcs that way too. */
 c=(demo_motion_t){0};step(&c,0,2,2000,2600);step(&c,10,0,0,0);
 o=step(&c,30,0,0,0);CHECK(o.right<o.left);
 /* Crossings stay straight and bounded; RECOVERY is unchanged. */
 c=(demo_motion_t){0};o=step(&c,0,3,4095,4095);CHECK(o.right==14&&o.left==14);
 o=step(&c,1000,3,4095,4095);CHECK(o.fault);
 /* Neutral/deadband steering retains either previous direction. */
 c=(demo_motion_t){0};step(&c,0,2,0,3300);CHECK(c.last_direction==1);
 step(&c,20,2,0,3100);CHECK(c.last_direction==1);
 step(&c,40,2,0,3000);CHECK(c.last_direction==1);
 step(&c,60,0,0,0);o=step(&c,80,0,0,0);CHECK(o.right<o.left);
 c=(demo_motion_t){0};step(&c,0,2,0,2700);CHECK(c.last_direction==-1);
 step(&c,20,2,0,2900);CHECK(c.last_direction==-1);
 step(&c,40,2,0,3000);CHECK(c.last_direction==-1);
 step(&c,60,0,0,0);o=step(&c,80,0,0,0);CHECK(o.right>o.left);
 /* Crossing discards previous P/D history; derivative resumes next frame. */
 c=(demo_motion_t){0};step(&c,0,2,0,4000);
 step(&c,20,3,4095,4095);CHECK(!c.track_valid&&c.track_d==0);
 o=step(&c,40,2,0,2500);CHECK(c.track_d==0&&o.right==14&&o.left==12);
 step(&c,60,2,0,2000);CHECK(c.track_d==-5);
 /* Barcode priority also resets history, even when it lasts many samples. */
 c=(demo_motion_t){0};step(&c,0,2,0,4000);
 demo_line_t scan={.time_ms=20,.mask=2,.valid=true,.calibrated=true,.barcode_capturing=true,.dark={0,3000,0}};
 o=demo_motion_step(&c,20,st(SM_NAV_TRACKING),&scan,0,false);CHECK(!c.track_valid&&c.track_d==0&&o.right==14&&o.left==14);
 scan.time_ms=500;demo_motion_step(&c,500,st(SM_NAV_TRACKING),&scan,0,false);
 o=step(&c,520,2,0,2500);CHECK(c.track_d==0&&o.right==14&&o.left==12);
 step(&c,540,2,0,2000);CHECK(c.track_d==-5);
 /* Observed floor cases: code-left -> right; fading centre -> left.
    Verify the sign persists into gentle and wide recovery. */
 for(unsigned mask=1;mask<=2;mask++){
  int dir=mask==1?1:-1;
  c=(demo_motion_t){0};step(&c,0,mask,mask==1?4095:0,mask==2?1800:0);
  step(&c,20,0,0,0);o=step(&c,740,0,0,0);CHECK(o.event==SM_EVT_LINE_LOST);
  demo_line_t lost={.time_ms=760,.valid=true,.calibrated=true};
  o=demo_motion_step(&c,760,st(SM_NAV_RECOVERY),&lost,0,false);CHECK(o.left==dir*10&&o.right==-dir*10);
  lost.time_ms=1110;o=demo_motion_step(&c,1110,st(SM_NAV_RECOVERY),&lost,0,false);CHECK(o.left==dir*18&&o.right==-dir*18);
 }
 /* Barcode-commanded left/right turns retain their motor mapping. */
 for(unsigned right=0;right<2;right++){
  c=(demo_motion_t){0};demo_line_t j={.valid=true,.calibrated=true,.mask=3};
  demo_motion_step(&c,0,st(SM_NAV_MANOEUVRE),&j,SM_NAV_CMD_PACK(SM_NAV_CMD_TURN,right?SM_ACTION_RIGHT:SM_ACTION_LEFT),false);
  j.time_ms=260;j.mask=0;o=demo_motion_step(&c,260,st(SM_NAV_MANOEUVRE),&j,0,false);
  CHECK(o.left==(right?14:-14)&&o.right==(right?-14:14));
 }
 printf("Smooth tracking: %u checks passed\n",checks);
 return 0;
}
