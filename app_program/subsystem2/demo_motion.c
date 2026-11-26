#include "demo_motion.h"
#include "demo_steering.h"
static int clamp(int x){return x>DEMO_MAX_DUTY?DEMO_MAX_DUTY:(x< -DEMO_MAX_DUTY?-DEMO_MAX_DUTY:x);}
static void straight(demo_output_t *o,int steer){
 int l=DEMO_BASE_DUTY+DEMO_LEFT_TRIM,r=DEMO_BASE_DUTY+DEMO_RIGHT_TRIM;
 if(steer>0)r=r*(100-steer)/100;else l=l*(100+steer)/100;
 o->left=clamp(l);o->right=clamp(r);o->forward=l>0&&r>0;
}
/* Forward at duty, inner wheel slowed by |steer| % (sign as straight()). */
static void drive(demo_output_t *o,int duty,int steer){
 int l=duty+DEMO_LEFT_TRIM,r=duty+DEMO_RIGHT_TRIM;
 if(steer>0)r=r*(100-steer)/100;else l=l*(100+steer)/100;
 o->left=clamp(l);o->right=clamp(r);o->forward=l>0&&r>0;
}
/* Proportional line following with damping on fresh IR samples, slowing as
   the steer grows. Latches the side the line was last drifting to. */
static void track(demo_motion_t *c,demo_output_t *o,demo_line_t const *s){
 int p=demo_track_steer(s->dark[0],s->dark[1],s->mask);
 if(!c->track_valid||s->time_ms!=c->track_sample){
  c->track_d=c->track_valid?(p-c->track_p)*DEMO_TRACK_KD/10:0;
  c->track_p=p;c->track_sample=s->time_ms;c->track_valid=true;
 }
 int steer=p+c->track_d;
 if(steer>DEMO_TRACK_MAX_STEER)steer=DEMO_TRACK_MAX_STEER;
 if(steer< -DEMO_TRACK_MAX_STEER)steer=-DEMO_TRACK_MAX_STEER;
 /* Keep the last meaningful direction while steering is near zero. */
 if((s->mask&3u)==1u||steer>=DEMO_TRACK_DIR_MIN)c->last_direction=1;
 else if(steer<=-DEMO_TRACK_DIR_MIN)c->last_direction=-1;
 int mag=steer<0?-steer:steer;
 int duty=DEMO_BASE_DUTY-mag*(DEMO_BASE_DUTY-DEMO_TRACK_FLOOR)/DEMO_TRACK_MAX_STEER;
 drive(o,duty,steer);
}
/* Arc after loss: the fixed 33% arc, or (smooth) a ramp towards a pivot. */
static int arc_steer(uint32_t arc){
 if(!DEMO_SMOOTH_TRACK)return DEMO_ARC_STEER;
 if(arc>=DEMO_SMOOTH_ARC_RAMP_MS)return DEMO_SMOOTH_ARC_MAX_STEER;
 return DEMO_ARC_STEER+(int)(arc*(uint32_t)(DEMO_SMOOTH_ARC_MAX_STEER-DEMO_ARC_STEER)/DEMO_SMOOTH_ARC_RAMP_MS);
}
#define TRACK_ARC_MS (DEMO_SMOOTH_TRACK?DEMO_SMOOTH_ARC_MS:DEMO_ARC_MS)
static void spin(demo_output_t *o,int dir,int duty){o->left=dir*duty;o->right=-dir*duty;}
static demo_output_t fail(demo_motion_t *c,sm_fault_t reason){
 demo_output_t o={0};c->failed=true;c->failure=reason;o.fault=true;o.event=SM_EVT_FAULT;o.arg=reason;return o;
}
/* Both search stages retain the direction latched before line loss.
 * Success needs two fresh IR frames; the deadline bounds the whole search. */
static demo_output_t search(demo_motion_t *c,uint32_t elapsed,
 demo_line_t const *s,int dir,bool wide_only,sm_evt_code_t success){
 demo_output_t o={0};
 uint32_t gentle=wide_only?0u:DEMO_SEARCH_FIRST_MS;
 if(elapsed>=gentle+DEMO_WIDE_SEARCH_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
 unsigned mask=s->mask&3u;
 /* Recovery accepts either tracing sensor; turns still require centre-only.
  * Both black remains a crossing, not a confirmed tracing position. */
 bool found=mask==2u||(success==SM_EVT_LINE_REACQUIRED&&mask==1u);
 if(found){
  if(!c->found_active){c->found_active=true;c->found_since=s->time_ms;}
  if(s->time_ms-c->found_since>=DEMO_REACQUIRE_MS){
   if(success==SM_EVT_LINE_REACQUIRED)c->last_direction=mask==1u?1:-1;
   o.event=success;c->done=true;
  }
 }else{
  c->found_active=false;
  int duty=DEMO_SEARCH_DUTY;
  if(elapsed>=gentle){
   duty=DEMO_WIDE_SEARCH_DUTY;
  }
  spin(&o,dir,duty);
 }
 return o;
}
/* ---- Obstacle bypass (heading + sonar guided, no encoders) ----------
 * dir: -1 bypass on the left, +1 on the right. rel: heading change since
 * the bypass began, cdeg, positive = turned right. Phases:
 * 0 turn out 45, 1 out leg until the sonar loses the obstacle edge,
 * 2 margin, 3 turn back to the start heading, 4 pass leg until the sonar
 * loses the obstacle side, 5 margin, 6 turn in 45, 7 approach until the
 * line, 8 turn back to the start heading, then BYPASS_DONE (S1: RECOVERY). */
enum {BP_TURN_OUT,BP_OUT,BP_OUT_MARGIN,BP_TURN_BACK,BP_PASS,BP_PASS_MARGIN,BP_TURN_IN,BP_APPROACH,BP_ALIGN};
static int32_t wrap_cdeg(int32_t a){while(a>18000)a-=36000;while(a<=-18000)a+=36000;return a;}
static void leg_reset(demo_motion_t *c,uint32_t now){c->reads=0;c->gone=0;c->seen=false;c->last_sonar=now;}
static void next_phase(demo_motion_t *c,uint32_t now,uint8_t phase){c->segment=phase;c->entered=now;leg_reset(c,now);}
/* Servo angle pointing offset degrees towards the obstacle (side -dir). */
static uint8_t aim_at_obstacle(int dir,unsigned offset){
 bool obstacle_right=dir<0;
 bool up=obstacle_right==(DEMO_SERVO_LEFT_LOW!=0);
 return (uint8_t)(up?90u+offset:90u-offset);
}
/* Straight at bypass duty, steering to hold target_rel (cdeg). */
static void hold(demo_output_t *o,int32_t rel,int32_t target_rel){
 int steer=(int)(wrap_cdeg(target_rel-rel)/100)*DEMO_BYPASS_HOLD_GAIN;
 if(steer>40)steer=40;
 if(steer<-40)steer=-40;
 int l=DEMO_BYPASS_DUTY,r=DEMO_BYPASS_DUTY;
 if(steer>0)r=r*(100-steer)/100;else l=l*(100+steer)/100;
 o->left=l;o->right=r;o->forward=true;
}
/* Count new sonar readings taken at the wanted angle after this leg began.
   Returns true once the obstacle has been seen and then lost, or was never
   seen across enough readings and time. */
static bool leg_clear(demo_motion_t *c,uint32_t now,uint8_t aim){
 demo_sense_t const *x=&c->sense;
 if(x->sonar_ok&&x->sonar_deg==aim&&(int32_t)(x->sonar_time-c->last_sonar)>0){
  c->last_sonar=x->sonar_time;
  if(c->reads<255u)++c->reads;
  if(x->sonar_cm<=DEMO_BYPASS_SEEN_CM){c->seen=true;c->gone=0;}
  else if(x->sonar_cm>=DEMO_BYPASS_GONE_CM){if(c->seen&&c->gone<255u)++c->gone;}
  else c->gone=0;
 }
 if(c->seen)return c->gone>=DEMO_BYPASS_GONE_READS;
 return c->reads>=DEMO_BYPASS_MIN_READS&&now-c->entered>=DEMO_BYPASS_UNSEEN_MS;
}
static demo_output_t bypass(demo_motion_t *c,uint32_t now,demo_line_t const *s,demo_output_t o){
 if(c->action!=SM_AVOID_LEFT&&c->action!=SM_AVOID_RIGHT)return fail(c,SM_FAULT_SCAN);
 if(!c->sense.heading_ok)return fail(c,SM_FAULT_IMU);
 if(!c->h0_set){c->h0=c->sense.heading_cdeg;c->h0_set=true;next_phase(c,now,BP_TURN_OUT);}
 int dir=c->action==SM_AVOID_LEFT?-1:1;
 int32_t rel=DEMO_HEADING_SIGN*wrap_cdeg(c->sense.heading_cdeg-c->h0);
 int32_t turned=dir*rel; /* positive: turned towards the bypass side */
 uint32_t t=now-c->entered;
 o.aim=true;o.aim_deg=aim_at_obstacle(dir,DEMO_BYPASS_AIM_OUT_DEG);
 switch(c->segment){
 case BP_TURN_OUT:
  if(t>=DEMO_BYPASS_TURN_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
  if(turned>=DEMO_BYPASS_OUT_CDEG){next_phase(c,now,BP_OUT);return o;}
  spin(&o,dir,DEMO_BYPASS_TURN_DUTY);return o;
 case BP_OUT:
  if(t>=DEMO_BYPASS_LEG_MAX_MS)return fail(c,SM_FAULT_SCAN);
  if(leg_clear(c,now,o.aim_deg)){next_phase(c,now,BP_OUT_MARGIN);return o;}
  hold(&o,rel,dir*DEMO_BYPASS_OUT_CDEG);return o;
 case BP_OUT_MARGIN:
  if(t>=DEMO_BYPASS_MARGIN_MS){next_phase(c,now,BP_TURN_BACK);return o;}
  hold(&o,rel,dir*DEMO_BYPASS_OUT_CDEG);return o;
 case BP_TURN_BACK:
  o.aim_deg=aim_at_obstacle(dir,DEMO_BYPASS_AIM_PASS_DEG);
  if(t>=DEMO_BYPASS_TURN_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
  if(turned<=DEMO_BYPASS_ALIGN_CDEG){next_phase(c,now,BP_PASS);return o;}
  spin(&o,-dir,DEMO_BYPASS_TURN_DUTY);return o;
 case BP_PASS:
  o.aim_deg=aim_at_obstacle(dir,DEMO_BYPASS_AIM_PASS_DEG);
  if(t>=DEMO_BYPASS_LEG_MAX_MS)return fail(c,SM_FAULT_SCAN);
  if(leg_clear(c,now,o.aim_deg)){next_phase(c,now,BP_PASS_MARGIN);return o;}
  hold(&o,rel,0);return o;
 case BP_PASS_MARGIN:
  o.aim_deg=90u;
  if(t>=DEMO_BYPASS_LENGTH_MS){next_phase(c,now,BP_TURN_IN);return o;}
  hold(&o,rel,0);return o;
 case BP_TURN_IN:
  o.aim_deg=90u;
  if(t>=DEMO_BYPASS_TURN_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
  if(turned<=-DEMO_BYPASS_IN_CDEG){next_phase(c,now,BP_APPROACH);return o;}
  spin(&o,-dir,DEMO_BYPASS_TURN_DUTY);return o;
 case BP_APPROACH:{
  demo_sense_t const *x=&c->sense;
  o.aim_deg=90u;
  if(t>=DEMO_BYPASS_APPROACH_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
  if(x->sonar_ok&&x->sonar_deg==90u&&(int32_t)(x->sonar_time-c->entered)>0&&
     x->sonar_cm<DEMO_BYPASS_BLOCK_CM)return fail(c,SM_FAULT_SCAN);
  if((s->mask&3u)!=0u&&t>=200u){next_phase(c,now,BP_ALIGN);return o;}
  hold(&o,rel,-dir*DEMO_BYPASS_IN_CDEG);return o;}
 case BP_ALIGN:
  o.aim_deg=90u;
  if(t>=DEMO_BYPASS_TURN_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
  if(turned>=-DEMO_BYPASS_ALIGN_CDEG){o.event=SM_EVT_BYPASS_DONE;c->done=true;return o;}
  spin(&o,dir,DEMO_BYPASS_TURN_DUTY);return o;
 default:
  return fail(c,SM_FAULT_SCAN);
 }
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
 if(c->nav!=nav){c->nav=nav;c->barcode_seen=false;c->entered=now;c->found_active=false;c->loss_active=false;c->cross_active=false;c->left_line=false;c->done=false;c->segment=0;c->h0_set=false;c->track_valid=false;leg_reset(c,now);}
 if(SM_NAV_CMD_CODE(cmd)==SM_NAV_CMD_TURN||SM_NAV_CMD_CODE(cmd)==SM_NAV_CMD_AVOID){c->action=SM_NAV_CMD_ARG(cmd);c->entered=now;c->segment=0;c->done=false;c->h0_set=false;leg_reset(c,now);}
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
  /* One window per decoder capture, not one window per bar/IR sample.
   * All state, pickup, stale-data and fault gates above still take priority. */
  if(!s->barcode_capturing)c->barcode_seen=false;
  else if(!c->barcode_seen){c->barcode_seen=true;c->barcode_since=now;}
  if(s->barcode_capturing&&now-c->barcode_since<DEMO_BARCODE_PRIORITY_MS){
   c->track_valid=false;c->track_d=0; /* Resume without pre-scan history. */
   if((s->mask&3u)==0u){
    if(!c->loss_active){c->loss_active=true;c->loss_since=now;c->loss_sample=s->time_ms;}
   }else{
    c->loss_active=false;
    if(line)c->last_direction=(s->mask&3u)==1u?1:-1;
   }
   /* Do not hide a persistent crossing fault during the scan. */
   if((s->mask&3u)==3u){
    if(!c->cross_active){c->cross_active=true;c->cross_since=now;}
    if(now-c->cross_since>=DEMO_CROSS_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
   }else c->cross_active=false;
   straight(&o,0);return o;
  }
  /* Smooth: either tracing sensor ends a loss and steering resumes at once. */
  if(line&&(DEMO_SMOOTH_TRACK||!c->loss_active||centred)){
   c->loss_active=false;c->cross_active=false;
   if(DEMO_SMOOTH_TRACK){track(c,&o,s);return o;}
   /* Physical mounting, viewed facing forward: code left is on the right;
    * centre is to its left. Search towards the last tracing sensor.
    * This is latched through loss/arc/pivot, not changed by search samples. */
   c->last_direction=(s->mask&3u)==1u?1:-1;
   straight(&o,0);
  }
  else if((s->mask&3u)==3u){
   c->track_valid=false;c->track_d=0; /* Crossing interrupts tracking. */
   c->loss_active=false;if(!c->cross_active){c->cross_active=true;c->cross_since=now;}
   if(now-c->cross_since>=DEMO_CROSS_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
   straight(&o,0); /* bounded passage over an uncommanded crossing */
  }else{
   c->cross_active=false;c->track_valid=false;
   if(!c->loss_active){c->loss_active=true;c->loss_since=now;c->loss_sample=s->time_ms;}
   if(now-c->loss_since<DEMO_LOSS_MS)straight(&o,0);
   else if(s->time_ms-c->loss_sample>=DEMO_LOSS_MS){
    if(now-c->loss_since>=DEMO_LOSS_MS+TRACK_ARC_MS)o.event=SM_EVT_LINE_LOST;
    else straight(&o,c->last_direction*arc_steer(now-c->loss_since-DEMO_LOSS_MS));
   }
   /* Brake while waiting for a fresh confirming sample, never extend blindly. */
  }
 }else if(nav==SM_NAV_RECOVERY){
  return search(c,now-c->entered,s,c->last_direction,false,SM_EVT_LINE_REACQUIRED);
 }else if(nav==SM_NAV_MANOEUVRE){
  uint32_t t=now-c->entered;
  if(c->action==SM_ACTION_UTURN){
   if(t<DEMO_PRETURN_MS){straight(&o,0);return o;}
   if(t<DEMO_PRETURN_MS+DEMO_UTURN_MIN_MS){spin(&o,-1,DEMO_TURN_DUTY);return o;}
   /* Remain MANOEUVRE until confirmed: do not claim TURN_DONE on a timer. */
   c->last_direction=-1;
   return search(c,t-DEMO_PRETURN_MS-DEMO_UTURN_MIN_MS,s,-1,true,SM_EVT_TURN_DONE);
  }
  if(t>=DEMO_TURN_MAX_MS)return fail(c,SM_FAULT_LINE_RECOVERY);
  if(t<DEMO_PRETURN_MS){straight(&o,0);return o;}
  if(!centred)c->left_line=true;
  uint32_t minimum=c->action==SM_ACTION_UTURN?DEMO_UTURN_MIN_MS:DEMO_TURN_MIN_MS;
  if(centred&&t>=DEMO_PRETURN_MS+minimum&&(c->left_line||c->action==SM_ACTION_STRAIGHT)){
   if(!c->found_active){c->found_active=true;c->found_since=now;}
   if(now-c->found_since>=DEMO_FOUND_MS){o.event=SM_EVT_TURN_DONE;c->done=true;}
  }else{c->found_active=false;if(c->action==SM_ACTION_STRAIGHT)straight(&o,0);else spin(&o,c->action==SM_ACTION_RIGHT?1:-1,DEMO_TURN_DUTY);}
 }else if(nav==SM_NAV_BYPASS){
  return bypass(c,now,s,o);
 }
 return o;
}
