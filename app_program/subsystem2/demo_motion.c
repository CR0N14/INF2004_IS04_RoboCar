#include "demo_motion.h"
static int clamp(int x){return x>DEMO_MAX_DUTY?DEMO_MAX_DUTY:(x< -DEMO_MAX_DUTY?-DEMO_MAX_DUTY:x);}
static void straight(demo_output_t *o,int steer){
 int l=DEMO_BASE_DUTY+DEMO_LEFT_TRIM,r=DEMO_BASE_DUTY+DEMO_RIGHT_TRIM;
 if(steer>0)r=r*(100-steer)/100;else l=l*(100+steer)/100;
 o->left=clamp(l);o->right=clamp(r);o->forward=l>0&&r>0;
}
static void spin(demo_output_t *o,int dir,int duty){o->left=dir*duty;o->right=-dir*duty;}
static demo_output_t fail(demo_motion_t *c,sm_fault_t reason){
 demo_output_t o={0};c->failed=true;c->failure=reason;o.fault=true;o.event=SM_EVT_FAULT;o.arg=reason;return o;
}
demo_output_t demo_motion_step(demo_motion_t *c,uint32_t now,uint32_t status,demo_line_t const *s,uint8_t cmd,bool pickup){
 demo_output_t o={0};sm_state_t state=SM_STATUS_STATE(status);sm_nav_mode_t nav=SM_STATUS_NAV(status);
 if(DEMO_BARCODE_BENCH)return o; /* Stationary decoder test: never drive. */
 bool running=state==SM_STATE_NAVIGATING||state==SM_STATE_OBSTACLE_AVOIDANCE;
 if(!running){*c=(demo_motion_t){0};return o;}
 if(pickup)return fail(c,SM_FAULT_COLLISION);
 if(c->failed)return fail(c,(sm_fault_t)c->failure);
 if(!c->running){c->running=true;c->run_start=now;c->last_direction=-1;c->nav=255;}
 if(now-c->run_start>=DEMO_RUN_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
 if(SM_STATUS_FAULTS(status)&~SM_FAULT_NON_FATAL_MASK)return o;
 if(!s->calibrated||!s->valid||now-s->time_ms>DEMO_STALE_MS)return fail(c,SM_FAULT_IR_SENSOR);
 if(c->nav!=nav){c->nav=nav;c->entered=now;c->found_active=false;c->loss_active=false;c->cross_active=false;c->left_line=false;c->done=false;c->segment=0;}
 if(SM_NAV_CMD_CODE(cmd)==SM_NAV_CMD_TURN||SM_NAV_CMD_CODE(cmd)==SM_NAV_CMD_AVOID){c->action=SM_NAV_CMD_ARG(cmd);c->entered=now;c->segment=0;c->done=false;}
 if(DEMO_BARCODE_DRIVE_MS){
  /* Timed straight pass over a barcode, then stop. Brake in any other mode. */
  if(nav!=SM_NAV_TRACKING||c->done)return o;
  if(now-c->run_start>=DEMO_BARCODE_DRIVE_MS){c->done=true;o.event=SM_EVT_CMD_STOP;return o;}
  straight(&o,0);return o;
 }
 bool line=(s->mask&3u)==1u||(s->mask&3u)==2u;
 bool centred=(s->mask&3u)==2u;
 if(nav==SM_NAV_HOLD||nav==SM_NAV_DISABLED)return o;
 if(c->done)return o; /* Keep braking while the completion event is retried. */
 if(nav==SM_NAV_TRACKING){
  if(line){c->loss_active=false;c->cross_active=false;if(s->steer)c->last_direction=s->steer>0?1:-1;straight(&o,s->steer);}
  else if((s->mask&3u)==3u){
   c->loss_active=false;if(!c->cross_active){c->cross_active=true;c->cross_since=now;}
   if(now-c->cross_since>=DEMO_CROSS_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
   straight(&o,0); /* bounded passage over an uncommanded crossing */
  }else{
   c->cross_active=false;if(!c->loss_active){c->loss_active=true;c->loss_since=now;}
   if(now-c->loss_since>=DEMO_LOSS_MS)o.event=SM_EVT_LINE_LOST;
  }
 }else if(nav==SM_NAV_RECOVERY){
  if(now-c->entered>=DEMO_RECOVERY_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
  if(centred){if(!c->found_active){c->found_active=true;c->found_since=now;}if(now-c->found_since>=DEMO_FOUND_MS){o.event=SM_EVT_LINE_REACQUIRED;c->done=true;}}
  else{c->found_active=false;int dir=c->last_direction;if(now-c->entered>=DEMO_SEARCH_FIRST_MS)dir=-dir;spin(&o,dir,DEMO_SEARCH_DUTY);}
 }else if(nav==SM_NAV_MANOEUVRE){
  uint32_t t=now-c->entered;
  if(t>=DEMO_TURN_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
  if(t<DEMO_PRETURN_MS){straight(&o,0);return o;}
  if(!centred)c->left_line=true;
  uint32_t minimum=c->action==SM_ACTION_UTURN?DEMO_UTURN_MIN_MS:DEMO_TURN_MIN_MS;
  if(centred&&t>=DEMO_PRETURN_MS+minimum&&(c->left_line||c->action==SM_ACTION_STRAIGHT)){
   if(!c->found_active){c->found_active=true;c->found_since=now;}
   if(now-c->found_since>=DEMO_FOUND_MS){o.event=SM_EVT_TURN_DONE;c->done=true;}
  }else{c->found_active=false;if(c->action==SM_ACTION_STRAIGHT)straight(&o,0);else spin(&o,c->action==SM_ACTION_RIGHT?1:-1,DEMO_TURN_DUTY);}
 }else if(nav==SM_NAV_BYPASS){
  /* A rectangular timed detour, then bounded line search. No distance claim. */
  static uint32_t const durations[5]={DEMO_BYPASS_SPIN_MS,DEMO_BYPASS_SIDE_MS,DEMO_BYPASS_SPIN_MS,DEMO_BYPASS_PASS_MS,DEMO_BYPASS_SPIN_MS};
  if(c->action!=SM_AVOID_LEFT&&c->action!=SM_AVOID_RIGHT)return fail(c,SM_FAULT_SCAN);
  if(c->segment<5u&&now-c->entered>=durations[c->segment]){++c->segment;c->entered=now;}
  int dir=c->action==SM_AVOID_LEFT?-1:1;
  if(c->segment==5u){
   if(now-c->entered>=DEMO_RECOVERY_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
   if(line&&now-c->entered>=200u){c->segment=6u;c->entered=now;c->left_line=false;c->found_active=false;return o;}
   straight(&o,0);return o;
  }
  if(c->segment==6u){
   if(now-c->entered>=DEMO_TURN_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
   if(!centred)c->left_line=true;
   if(centred&&c->left_line&&now-c->entered>=DEMO_TURN_MIN_MS){
    if(!c->found_active){c->found_active=true;c->found_since=now;}
    if(now-c->found_since>=DEMO_FOUND_MS){o.event=SM_EVT_BYPASS_DONE;c->done=true;}
   }else{c->found_active=false;spin(&o,dir,DEMO_TURN_DUTY);}
   return o;
  }
  if(c->segment==0)spin(&o,dir,DEMO_TURN_DUTY);
  else if(c->segment==2||c->segment==4)spin(&o,-dir,DEMO_TURN_DUTY);
  else straight(&o,0);
 }
 return o;
}
