#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem2/demo_steering.h"
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
 o=step(&c,10,SM_NAV_TRACKING,1,0);CHECK(o.left==14&&o.right==14);
 o=step(&c,20,SM_NAV_HOLD,2,0);CHECK(o.left==0&&o.right==0);
 demo_line_t stale={.calibrated=true,.valid=true,.time_ms=0,.mask=2};c=(demo_motion_t){0};o=demo_motion_step(&c,100,status(SM_NAV_TRACKING),&stale,0,false);CHECK(o.fault&&o.arg==SM_FAULT_IR_SENSOR&&!o.left&&!o.right);
 c=(demo_motion_t){0};step(&c,0,SM_NAV_TRACKING,0,0);o=step(&c,320,SM_NAV_TRACKING,0,0);CHECK(o.event==SM_EVT_LINE_LOST&&!o.left&&!o.right);
 o=step(&c,110,SM_NAV_RECOVERY,0,0);CHECK(o.left<0&&o.right>0);
 o=step(&c,309,SM_NAV_RECOVERY,0,0);CHECK(o.left<0&&o.right>0);
 o=step(&c,710,SM_NAV_RECOVERY,0,0);CHECK(o.left<0&&o.right>0);
 o=step(&c,1200,SM_NAV_RECOVERY,0,0);CHECK(o.left<0&&o.right>0);
 step(&c,1210,SM_NAV_RECOVERY,2,0);o=step(&c,1310,SM_NAV_RECOVERY,2,0);CHECK(o.event==SM_EVT_LINE_REACQUIRED);
 c=(demo_motion_t){0};step(&c,0,SM_NAV_RECOVERY,0,0);o=step(&c,DEMO_RECOVERY_MS,SM_NAV_RECOVERY,0,0);CHECK(o.fault&&!o.left&&!o.right);
 for(unsigned a=0;a<4;a++){
  c=(demo_motion_t){0};step(&c,0,SM_NAV_MANOEUVRE,3,SM_NAV_CMD_PACK(SM_NAV_CMD_TURN,a));
  o=step(&c,260,SM_NAV_MANOEUVRE,0,0);CHECK(a==SM_ACTION_STRAIGHT?(o.left>0&&o.right>0):(o.left*o.right<0));
  CHECK((o.left==DEMO_BASE_DUTY||o.left==-DEMO_BASE_DUTY)&&(o.right==DEMO_BASE_DUTY||o.right==-DEMO_BASE_DUTY));
  step(&c,1950,SM_NAV_MANOEUVRE,2,0);o=step(&c,2050,SM_NAV_MANOEUVRE,2,0);CHECK(o.event==SM_EVT_TURN_DONE&&!o.left&&!o.right);
 }
 c=(demo_motion_t){0};step(&c,0,SM_NAV_TRACKING,3,0);o=step(&c,1000,SM_NAV_TRACKING,3,0);CHECK(o.fault);
 c=(demo_motion_t){0};step(&c,0,SM_NAV_MANOEUVRE,0,SM_NAV_CMD_PACK(SM_NAV_CMD_TURN,0));o=step(&c,3000,SM_NAV_MANOEUVRE,0,0);CHECK(o.fault);
 /* Gentle-to-wide boundary, retained direction, fresh confirmation and U-turn deadline. */
 c=(demo_motion_t){0};o=step(&c,0,SM_NAV_RECOVERY,0,0);CHECK(o.left==-10);
 o=step(&c,349,SM_NAV_RECOVERY,0,0);CHECK(o.left==-10);
 o=step(&c,350,SM_NAV_RECOVERY,0,0);CHECK(o.left==-18&&!o.event);
 o=step(&c,599,SM_NAV_RECOVERY,0,0);CHECK(o.left==-18);
 o=step(&c,600,SM_NAV_RECOVERY,0,0);CHECK(o.left==-18);
 step(&c,620,SM_NAV_RECOVERY,2,0);
 o=step(&c,640,SM_NAV_RECOVERY,2,0);CHECK(o.event==SM_EVT_LINE_REACQUIRED);
 c=(demo_motion_t){0};step(&c,0,SM_NAV_MANOEUVRE,3,SM_NAV_CMD_PACK(SM_NAV_CMD_TURN,SM_ACTION_UTURN));
 o=step(&c,1849,SM_NAV_MANOEUVRE,2,0);CHECK(o.left==-14&&!o.event);
 o=step(&c,1850,SM_NAV_MANOEUVRE,0,0);CHECK(o.left==-18&&!o.event);
 o=step(&c,2100,SM_NAV_MANOEUVRE,0,0);CHECK(o.left==-18);
 o=step(&c,4850,SM_NAV_MANOEUVRE,0,0);CHECK(o.fault&&!o.left&&!o.right);
 /* Mounting-aware loss direction survives both search stages, even when
    the other sensor briefly appears during the search. */
 for(unsigned mask=1;mask<=2;mask++){
  int dir=mask==1?1:-1;
  c=(demo_motion_t){0};step(&c,0,SM_NAV_TRACKING,mask,0);
  step(&c,20,SM_NAV_TRACKING,0,0);
  o=step(&c,40,SM_NAV_TRACKING,0,0);CHECK(o.forward&&(dir<0?o.left<o.right:o.left>o.right));
  o=step(&c,340,SM_NAV_TRACKING,0,0);CHECK(o.event==SM_EVT_LINE_LOST);
  o=step(&c,360,SM_NAV_RECOVERY,0,0);CHECK(o.left==dir*10&&o.right==-dir*10);
  o=step(&c,710,SM_NAV_RECOVERY,0,0);CHECK(o.left==dir*18&&o.right==-dir*18);
  step(&c,730,SM_NAV_RECOVERY,1,0);
  o=step(&c,1500,SM_NAV_RECOVERY,0,0);CHECK(o.left==dir*18&&o.right==-dir*18);
  o=step(&c,360+DEMO_RECOVERY_MS,SM_NAV_RECOVERY,0,0);CHECK(o.fault&&!o.left&&!o.right);
 }
 /* Barcode candidate suppresses correction for at most four seconds. */
 c=(demo_motion_t){0};
 demo_line_t scan={.valid=true,.calibrated=true,.barcode_capturing=true,.mask=0};
 o=demo_motion_step(&c,0,status(SM_NAV_TRACKING),&scan,0,false);CHECK(o.left==14&&o.right==14);
 scan.time_ms=DEMO_BARCODE_PRIORITY_MS-1;o=demo_motion_step(&c,DEMO_BARCODE_PRIORITY_MS-1,status(SM_NAV_TRACKING),&scan,0,false);CHECK(o.left==14&&o.right==14&&!o.event);
 scan.time_ms=DEMO_BARCODE_PRIORITY_MS;o=demo_motion_step(&c,DEMO_BARCODE_PRIORITY_MS,status(SM_NAV_TRACKING),&scan,0,false);CHECK(o.event==SM_EVT_LINE_LOST&&!o.left&&!o.right);
 scan.time_ms=DEMO_BARCODE_PRIORITY_MS+20;o=demo_motion_step(&c,DEMO_BARCODE_PRIORITY_MS+20,status(SM_NAV_RECOVERY),&scan,0,false);CHECK(o.left==-10&&o.right==10);
 /* Decoder completion releases the window early; STOP and stale data win. */
 c=(demo_motion_t){0};scan.time_ms=0;demo_motion_step(&c,0,status(SM_NAV_TRACKING),&scan,0,false);
 scan.time_ms=400;scan.barcode_capturing=false;o=demo_motion_step(&c,400,status(SM_NAV_TRACKING),&scan,0,false);CHECK(o.event==SM_EVT_LINE_LOST);
 c=(demo_motion_t){0};scan.time_ms=0;scan.barcode_capturing=true;demo_motion_step(&c,0,status(SM_NAV_TRACKING),&scan,0,false);
 o=demo_motion_step(&c,70,status(SM_NAV_TRACKING),&scan,0,false);CHECK(o.fault&&o.arg==SM_FAULT_IR_SENSOR);
 o=demo_motion_step(&c,80,SM_STATE_STANDBY,&scan,0,false);CHECK(!o.left&&!o.right);
 /* Guided bypass: testing/demo/test_bypass.c. */
 c=(demo_motion_t){0};demo_line_t good={.valid=true,.calibrated=true,.mask=2};o=demo_motion_step(&c,0,status(SM_NAV_TRACKING),&good,0,true);CHECK(o.fault&&o.arg==SM_FAULT_COLLISION&&!o.left&&!o.right);
 o=demo_motion_step(&c,10,SM_STATE_STANDBY,&good,0,true);CHECK(!o.event&&!o.left&&!o.right);
 c=(demo_motion_t){0};step(&c,UINT32_MAX-50,SM_NAV_TRACKING,0,0);o=step(&c,270,SM_NAV_TRACKING,0,0);CHECK(o.event==SM_EVT_LINE_LOST);
 uint16_t d[25],width=0;for(int i=0;i<25;i++)d[i]=100;for(int i=8;i<=16;i++)d[i]=20;
 CHECK(demo_profile_width(d,&width)&&width==14);d[0]=511;CHECK(demo_profile_width(d,&width)&&width==14); /* no echo = far */for(int i=0;i<25;i++)d[i]=20;CHECK(!demo_profile_width(d,&width));
 sm_ctx_t sm;sm_actions_t act;sm_logic_init(&sm,&act);sm_logic_step(&sm,SM_EVT_CHECK,0,&act);sm_logic_step(&sm,SM_EVT_CMD_START,0,&act);CHECK(sm.state==SM_STATE_NAVIGATING);
 sm_logic_step(&sm,SM_EVT_BARCODE,SM_BARCODE_ARG(0,SM_ACTION_LEFT),&act);CHECK(sm.b_pending);
 sm_logic_step(&sm,SM_EVT_JUNCTION,0,&act);CHECK(sm.nav_mode==SM_NAV_MANOEUVRE&&act.b_nav_cmd);
 sm_logic_step(&sm,SM_EVT_TURN_DONE,0,&act);CHECK(sm.nav_mode==SM_NAV_TRACKING);
 sm_logic_step(&sm,SM_EVT_OBSTACLE,0,&act);CHECK(sm.nav_mode==SM_NAV_HOLD);
 sm_logic_step(&sm,SM_EVT_PROFILE_READY,SM_AVOID_LEFT,&act);CHECK(sm.nav_mode==SM_NAV_BYPASS);
 sm_logic_step(&sm,SM_EVT_BYPASS_DONE,0,&act);CHECK(sm.nav_mode==SM_NAV_RECOVERY);
 sm_logic_step(&sm,SM_EVT_LINE_REACQUIRED,0,&act);CHECK(sm.state==SM_STATE_NAVIGATING);
 sm_logic_step(&sm,SM_EVT_FAULT,SM_FAULT_COLLISION,&act);CHECK(sm.state==SM_STATE_ERROR);
 sm_logic_init(&sm,&act);sm_logic_step(&sm,SM_EVT_CHECK,0,&act);
 sm_logic_step(&sm,SM_EVT_CMD_START,0,&act);
 sm_logic_step(&sm,SM_EVT_FAULT,SM_FAULT_NO_ECHO,&act);CHECK(sm.state==SM_STATE_ERROR);
 c=(demo_motion_t){0};o=demo_motion_step(&c,0,SM_STATE_ERROR,&good,0,false);CHECK(!o.left&&!o.right);
 CHECK(demo_analogue_steer(0,4095,2)==0);
 CHECK(demo_analogue_steer(1500,2500,2)>0);
 CHECK(demo_analogue_steer(0,2000,2)<0);
 CHECK(demo_analogue_steer(4095,4095,3)==0);
 CHECK(demo_analogue_steer(0,0,0)==0);
 c=(demo_motion_t){0};o=step(&c,0,SM_NAV_TRACKING,1,0);CHECK(o.left==14&&o.right==14);
 o=step(&c,200,SM_NAV_TRACKING,1,0);CHECK(o.left==14&&o.right==14);
 o=step(&c,220,SM_NAV_TRACKING,0,0);CHECK(o.forward&&!o.event&&o.left>0&&o.right>0);
 o=step(&c,239,SM_NAV_TRACKING,0,0);CHECK(o.forward&&!o.event);
 o=step(&c,240,SM_NAV_TRACKING,0,0);CHECK(o.left==14&&o.right==9&&o.forward&&!o.event);
 o=step(&c,260,SM_NAV_TRACKING,1,0);CHECK(o.left==14&&o.right==9);
 o=step(&c,280,SM_NAV_TRACKING,2,0);CHECK(o.left==14&&o.right==14&&!o.event);
 o=step(&c,300,SM_NAV_TRACKING,0,0);CHECK(o.forward);
 o=step(&c,620,SM_NAV_TRACKING,0,0);CHECK(o.event==SM_EVT_LINE_LOST&&!o.left&&!o.right);
 o=step(&c,370,SM_NAV_RECOVERY,0,0);CHECK(o.left==-10&&o.right==10);
 /* Repeated reads of one ADC frame cannot confirm loss or reacquisition. */
 c=(demo_motion_t){0};demo_line_t repeated={.time_ms=100,.calibrated=true,.valid=true,.mask=0};
 o=demo_motion_step(&c,100,status(SM_NAV_TRACKING),&repeated,0,false);CHECK(o.forward);
 o=demo_motion_step(&c,120,status(SM_NAV_TRACKING),&repeated,0,false);CHECK(!o.event&&!o.left&&!o.right);
 repeated.time_ms=120;o=demo_motion_step(&c,120,status(SM_NAV_TRACKING),&repeated,0,false);CHECK(o.forward&&o.left==9&&o.right==14&&!o.event);
 o=step(&c,130,SM_NAV_RECOVERY,1,0);CHECK(!o.left&&!o.right&&!o.event);
 repeated.mask=2;repeated.time_ms=140;o=demo_motion_step(&c,140,status(SM_NAV_RECOVERY),&repeated,0,false);CHECK(!o.left&&!o.right&&!o.event);
 o=demo_motion_step(&c,160,status(SM_NAV_RECOVERY),&repeated,0,false);CHECK(!o.event);
 repeated.time_ms=160;o=demo_motion_step(&c,160,status(SM_NAV_RECOVERY),&repeated,0,false);CHECK(o.event==SM_EVT_LINE_REACQUIRED);
 /* Either single sensor confirms recovery; a reused ADC frame cannot. */
 for(unsigned mask=1;mask<=2;mask++){
  c=(demo_motion_t){0};step(&c,0,SM_NAV_RECOVERY,0,0);
  demo_line_t hit={.valid=true,.calibrated=true,.time_ms=400,.mask=mask};
  o=demo_motion_step(&c,400,status(SM_NAV_RECOVERY),&hit,0,false);CHECK(!o.event&&!o.left&&!o.right);
  o=demo_motion_step(&c,420,status(SM_NAV_RECOVERY),&hit,0,false);CHECK(!o.event);
  hit.time_ms=420;o=demo_motion_step(&c,420,status(SM_NAV_RECOVERY),&hit,0,false);CHECK(o.event==SM_EVT_LINE_REACQUIRED);
  CHECK(c.last_direction==(mask==1?1:-1));
  o=step(&c,440,SM_NAV_TRACKING,mask,0);CHECK(o.left==14&&o.right==14);
 }
 c=(demo_motion_t){0};step(&c,0,SM_NAV_RECOVERY,1,0);
 step(&c,20,SM_NAV_RECOVERY,0,0);
 o=step(&c,40,SM_NAV_RECOVERY,1,0);CHECK(!o.event);
 o=step(&c,60,SM_NAV_RECOVERY,1,0);CHECK(o.event==SM_EVT_LINE_REACQUIRED);
 c=(demo_motion_t){0};step(&c,0,SM_NAV_RECOVERY,3,0);
 o=step(&c,20,SM_NAV_RECOVERY,3,0);CHECK(!o.event&&o.left*o.right<0);
 c=(demo_motion_t){0};step(&c,0,SM_NAV_MANOEUVRE,3,SM_NAV_CMD_PACK(SM_NAV_CMD_TURN,SM_ACTION_UTURN));
 step(&c,1850,SM_NAV_MANOEUVRE,1,0);
 o=step(&c,1870,SM_NAV_MANOEUVRE,1,0);CHECK(!o.event&&o.left==-18);
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
