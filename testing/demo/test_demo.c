#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../app_program/subsystem4/demo_pickup.h"
#include "../../app_program/subsystem2/demo_motion.h"
#include "../../app_program/subsystem1/sm_logic.h"
#include "../../app_program/subsystem5/demo_profile.h"
static unsigned checks;
#define CHECK(x) do {assert(x);++checks;}while(0)
static uint32_t status(sm_nav_mode_t nav){return SM_STATE_NAVIGATING|((uint32_t)nav<<3);}
static demo_output_t step(demo_motion_t *c,uint32_t t,sm_nav_mode_t nav,uint8_t mask,uint8_t cmd){demo_line_t s={.time_ms=t,.mask=mask,.calibrated=true,.valid=true,.steer=mask==1?-DEMO_LINE_CORRECTION:0};return demo_motion_step(c,t,status(nav),&s,cmd,false);}
int main(void){
 demo_motion_t c={0};demo_output_t o=step(&c,0,SM_NAV_TRACKING,2,0);CHECK(o.left==14&&o.right==14);
 o=step(&c,10,SM_NAV_TRACKING,1,0);CHECK(o.left==7&&o.right==14);
 o=step(&c,20,SM_NAV_HOLD,2,0);CHECK(o.left==0&&o.right==0);
 demo_line_t stale={.calibrated=true,.valid=true,.time_ms=0,.mask=2};c=(demo_motion_t){0};o=demo_motion_step(&c,100,status(SM_NAV_TRACKING),&stale,0,false);CHECK(o.fault&&o.arg==SM_FAULT_IR_SENSOR&&!o.left&&!o.right);
 c=(demo_motion_t){0};step(&c,0,SM_NAV_TRACKING,0,0);o=step(&c,100,SM_NAV_TRACKING,0,0);CHECK(o.event==SM_EVT_LINE_LOST&&!o.left&&!o.right);
 o=step(&c,110,SM_NAV_RECOVERY,0,0);CHECK(o.left<0&&o.right>0);
 o=step(&c,359,SM_NAV_RECOVERY,0,0);CHECK(o.left<0&&o.right>0);
 o=step(&c,360,SM_NAV_RECOVERY,0,0);CHECK(o.left>0&&o.right<0);
 o=step(&c,1200,SM_NAV_RECOVERY,0,0);CHECK(o.left>0&&o.right<0);
 step(&c,1210,SM_NAV_RECOVERY,2,0);o=step(&c,1310,SM_NAV_RECOVERY,2,0);CHECK(o.event==SM_EVT_LINE_REACQUIRED);
 c=(demo_motion_t){0};step(&c,0,SM_NAV_RECOVERY,0,0);o=step(&c,3000,SM_NAV_RECOVERY,0,0);CHECK(o.fault&&!o.left&&!o.right);
 for(unsigned a=0;a<4;a++){
  c=(demo_motion_t){0};step(&c,0,SM_NAV_MANOEUVRE,3,SM_NAV_CMD_PACK(SM_NAV_CMD_TURN,a));
  o=step(&c,260,SM_NAV_MANOEUVRE,0,0);CHECK(a==SM_ACTION_STRAIGHT?(o.left>0&&o.right>0):(o.left*o.right<0));
  step(&c,1100,SM_NAV_MANOEUVRE,2,0);o=step(&c,1200,SM_NAV_MANOEUVRE,2,0);CHECK(o.event==SM_EVT_TURN_DONE&&!o.left&&!o.right);
 }
 c=(demo_motion_t){0};step(&c,0,SM_NAV_TRACKING,3,0);o=step(&c,1000,SM_NAV_TRACKING,3,0);CHECK(o.fault);
 c=(demo_motion_t){0};step(&c,0,SM_NAV_MANOEUVRE,0,SM_NAV_CMD_PACK(SM_NAV_CMD_TURN,0));o=step(&c,3000,SM_NAV_MANOEUVRE,0,0);CHECK(o.fault);
 for(unsigned a=SM_AVOID_LEFT;a<=SM_AVOID_RIGHT;a++){
  c=(demo_motion_t){0};step(&c,0,SM_NAV_BYPASS,0,SM_NAV_CMD_PACK(SM_NAV_CMD_AVOID,a));
  unsigned times[]={450,1450,1900,3400,3850};for(unsigned i=0;i<5;i++)step(&c,times[i],SM_NAV_BYPASS,0,0);
  step(&c,4100,SM_NAV_BYPASS,2,0);CHECK(c.segment==6);
  step(&c,4200,SM_NAV_BYPASS,0,0);step(&c,4500,SM_NAV_BYPASS,2,0);o=step(&c,4600,SM_NAV_BYPASS,2,0);CHECK(o.event==SM_EVT_BYPASS_DONE&&!o.left&&!o.right);
 }
 c=(demo_motion_t){0};demo_line_t good={.valid=true,.calibrated=true,.mask=2};o=demo_motion_step(&c,0,status(SM_NAV_TRACKING),&good,0,true);CHECK(o.fault&&o.arg==SM_FAULT_COLLISION&&!o.left&&!o.right);
 o=demo_motion_step(&c,10,SM_STATE_STANDBY,&good,0,true);CHECK(!o.event&&!o.left&&!o.right);
 c=(demo_motion_t){0};step(&c,UINT32_MAX-50,SM_NAV_TRACKING,0,0);o=step(&c,50,SM_NAV_TRACKING,0,0);CHECK(o.event==SM_EVT_LINE_LOST);
 uint16_t d[25],width=0;for(int i=0;i<25;i++)d[i]=100;for(int i=8;i<=16;i++)d[i]=20;
 CHECK(demo_profile_width(d,&width)&&width==14);d[0]=511;CHECK(!demo_profile_width(d,&width));for(int i=0;i<25;i++)d[i]=20;CHECK(!demo_profile_width(d,&width));
 sm_ctx_t sm;sm_actions_t act;sm_logic_init(&sm,&act);sm_logic_step(&sm,SM_EVT_CHECK,0,&act);sm_logic_step(&sm,SM_EVT_CMD_START,0,&act);CHECK(sm.state==SM_STATE_NAVIGATING);
 sm_logic_step(&sm,SM_EVT_BARCODE,SM_BARCODE_ARG(0,SM_ACTION_LEFT),&act);CHECK(sm.b_pending);
 sm_logic_step(&sm,SM_EVT_JUNCTION,0,&act);CHECK(sm.nav_mode==SM_NAV_MANOEUVRE&&act.b_nav_cmd);
 sm_logic_step(&sm,SM_EVT_TURN_DONE,0,&act);CHECK(sm.nav_mode==SM_NAV_TRACKING);
 sm_logic_step(&sm,SM_EVT_OBSTACLE,0,&act);CHECK(sm.nav_mode==SM_NAV_HOLD);
 sm_logic_step(&sm,SM_EVT_PROFILE_READY,SM_AVOID_LEFT,&act);CHECK(sm.nav_mode==SM_NAV_BYPASS);
 sm_logic_step(&sm,SM_EVT_BYPASS_DONE,0,&act);CHECK(sm.nav_mode==SM_NAV_RECOVERY);
 sm_logic_step(&sm,SM_EVT_LINE_REACQUIRED,0,&act);CHECK(sm.state==SM_STATE_NAVIGATING);
 sm_logic_step(&sm,SM_EVT_FAULT,SM_FAULT_COLLISION,&act);CHECK(sm.state==SM_STATE_ERROR);
 demo_pickup_t pickup={0};
 CHECK(!demo_pickup_step(&pickup,0,5000,0,false,false));
 CHECK(!demo_pickup_step(&pickup,0,5000,0,true,false));
 CHECK(!demo_pickup_step(&pickup,149,5000,0,true,false));
 CHECK(demo_pickup_step(&pickup,150,5000,0,true,false));
 CHECK(demo_pickup_step(&pickup,160,0,0,false,false));
 pickup=(demo_pickup_t){0};CHECK(demo_pickup_step(&pickup,0,0,0,true,true));
 /* All injected A bindings are stored until junction and consumed once. */
 for(unsigned action=0;action<4;action++){
  sm_logic_init(&sm,&act);sm_logic_step(&sm,SM_EVT_CHECK,0,&act);
  sm_logic_step(&sm,SM_EVT_CMD_START,0,&act);
  sm_logic_step(&sm,SM_EVT_BARCODE,SM_BARCODE_ARG(0,action),&act);
  CHECK(sm.b_pending&&sm.pending==action&&sm.nav_mode==SM_NAV_TRACKING);
  sm_logic_step(&sm,SM_EVT_JUNCTION,0,&act);
  CHECK(!sm.b_pending&&sm.nav_mode==SM_NAV_MANOEUVRE&&act.b_nav_cmd);
  sm_logic_step(&sm,SM_EVT_TURN_DONE,0,&act);
  sm_logic_step(&sm,SM_EVT_JUNCTION,0,&act);
  CHECK(sm.nav_mode==SM_NAV_TRACKING&&!act.b_nav_cmd);
  sm_logic_step(&sm,SM_EVT_CMD_STOP,0,&act);
  CHECK(sm.state==SM_STATE_STANDBY&&!sm.b_pending);
 }
 printf("Encoder-free demo: %u assertions passed\n",checks);
}
