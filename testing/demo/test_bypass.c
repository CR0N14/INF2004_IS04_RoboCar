/* Guided obstacle bypass and sweep analysis: host simulation, no hardware.
   The model spins the heading at 90 deg/s while the wheels counter-rotate,
   returns a sonar reading every 70 ms at the angle requested one cycle
   earlier, and shows the line once the approach leg has run 500 ms. */
#include <stdio.h>
#include "../../app_program/subsystem2/demo_motion.h"
#include "../../app_program/subsystem5/demo_profile.h"
#define CHECK(x) do{++checks;if(!(x)){printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#x);return 1;}}while(0)
static unsigned checks;
static uint32_t avoid(sm_nav_mode_t n){return SM_STATE_OBSTACLE_AVOIDANCE|((uint32_t)n<<3);}

typedef struct {
 int32_t heading;        /* cdeg, 0..35999                              */
 uint32_t sonar_time;    /* last reading time                           */
 uint8_t servo;          /* angle the servo reached (one cycle lag)     */
 uint16_t sonar_cm;
 bool heading_ok,sonar_dead,block_ahead;
 unsigned out_seen_ms,pass_seen_ms; /* how long each leg sees the obstacle */
} sim_t;

/* Distance the sonar reports, from the phase the car is in. */
static uint16_t world(sim_t const *w,demo_motion_t const *c,uint32_t now){
 uint32_t t=now-c->entered;
 if(c->segment==1)return t<w->out_seen_ms?25u:150u;
 if(c->segment==4)return t<w->pass_seen_ms?20u:200u;
 if(c->segment==7)return w->block_ahead?8u:120u;
 return 100u;
}

/* Run a bypass to completion or fault. Returns the final output. */
static demo_output_t run(sim_t *w,uint8_t action,uint32_t *end,uint8_t *aims,int *first_turn){
 demo_motion_t c={0};demo_output_t o={0};uint8_t last_phase=255;
 uint8_t cmd=SM_NAV_CMD_PACK(SM_NAV_CMD_AVOID,action);
 *first_turn=0;
 for(uint32_t now=1000;now<40000;now+=10){
  demo_line_t s={.time_ms=now,.calibrated=true,.valid=true,.mask=0};
  if(c.segment==7&&now-c.entered>=500u)s.mask=2;
  if(!w->sonar_dead&&now-w->sonar_time>=70u){w->sonar_time=now;w->sonar_cm=world(w,&c,now);}
  c.sense=(demo_sense_t){.heading_ok=w->heading_ok,.heading_cdeg=w->heading,
   .sonar_ok=!w->sonar_dead,.sonar_cm=w->sonar_cm,.sonar_deg=w->servo,.sonar_time=w->sonar_time};
  o=demo_motion_step(&c,now,avoid(SM_NAV_BYPASS),&s,cmd,false);cmd=0;
  if(o.aim)w->servo=o.aim_deg;
  if(c.segment!=last_phase&&c.segment<9){aims[c.segment]=o.aim_deg;last_phase=c.segment;}
  if(o.left&&o.right&&(o.left>0)!=(o.right>0)){
   int dir=o.left>0?1:-1; /* left wheel forward: turning right */
   if(!*first_turn)*first_turn=dir;
   w->heading=(w->heading+dir*90+36000)%36000;
  }
  if(o.event!=SM_EVT_NONE){*end=now;return o;}
 }
 *end=0;return o;
}

int main(void){
 uint8_t aims[9];uint32_t end;int first;demo_output_t o;
 /* Both sides, including a start heading next to the 0/360 wrap. */
 for(uint8_t action=SM_AVOID_LEFT;action<=SM_AVOID_RIGHT;action++){
  sim_t w={.heading=35900,.heading_ok=true,.servo=90,.out_seen_ms=700,.pass_seen_ms=900};
  o=run(&w,action,&end,aims,&first);
  CHECK(o.event==SM_EVT_BYPASS_DONE&&!o.fault&&end>0);
  CHECK(first==(action==SM_AVOID_LEFT?-1:1)); /* first spin is towards the bypass side */
  int32_t err=(w.heading-35900+54000)%36000-18000;
  CHECK(err>=-DEMO_BYPASS_ALIGN_CDEG-100&&err<=DEMO_BYPASS_ALIGN_CDEG+100); /* back on the start heading */
  /* Servo looks at the obstacle side: right (high angles) when bypassing left. */
  CHECK(aims[1]==(action==SM_AVOID_LEFT?150:30));
  CHECK(aims[4]==(action==SM_AVOID_LEFT?180:0));
  CHECK(aims[7]==90);
 }
 /* An obstacle never seen on a leg still ends the leg once enough readings arrived. */
 {sim_t w={.heading=9000,.heading_ok=true,.servo=90,.out_seen_ms=0,.pass_seen_ms=0};
  o=run(&w,SM_AVOID_LEFT,&end,aims,&first);CHECK(o.event==SM_EVT_BYPASS_DONE);}
 /* No heading: refuse to move. */
 {sim_t w={.heading=0,.heading_ok=false,.servo=90,.out_seen_ms=700,.pass_seen_ms=900};
  o=run(&w,SM_AVOID_LEFT,&end,aims,&first);CHECK(o.fault&&o.arg==SM_FAULT_IMU&&!o.left&&!o.right);}
 /* Sonar silent: the out leg times out instead of guessing. */
 {sim_t w={.heading=0,.heading_ok=true,.servo=90,.sonar_dead=true};
  o=run(&w,SM_AVOID_LEFT,&end,aims,&first);CHECK(o.fault&&o.arg==SM_FAULT_SCAN&&!o.left&&!o.right);}
 /* Obstacle that never ends on the pass leg: leg timeout. */
 {sim_t w={.heading=0,.heading_ok=true,.servo=90,.out_seen_ms=700,.pass_seen_ms=60000};
  o=run(&w,SM_AVOID_RIGHT,&end,aims,&first);CHECK(o.fault&&o.arg==SM_FAULT_SCAN);}
 /* Something close ahead while approaching the line. */
 {sim_t w={.heading=0,.heading_ok=true,.servo=90,.out_seen_ms=700,.pass_seen_ms=900,.block_ahead=true};
  o=run(&w,SM_AVOID_LEFT,&end,aims,&first);CHECK(o.fault&&o.arg==SM_FAULT_SCAN&&!o.left&&!o.right);}
 /* Heading frozen (spin not registering): the turn times out. */
 {demo_motion_t c={0};demo_line_t s={.calibrated=true,.valid=true};
  c.sense=(demo_sense_t){.heading_ok=true,.heading_cdeg=1000};
  uint8_t cmd=SM_NAV_CMD_PACK(SM_NAV_CMD_AVOID,SM_AVOID_LEFT);
  for(uint32_t now=0;now<=DEMO_BYPASS_TURN_MAX_MS+20;now+=10){s.time_ms=now;o=demo_motion_step(&c,now,avoid(SM_NAV_BYPASS),&s,cmd,false);cmd=0;if(o.fault)break;}
  CHECK(o.fault&&o.arg==SM_FAULT_LINE_RECOVERY);}
 /* Unknown action (REVERSE) is refused. */
 {demo_motion_t c={0};demo_line_t s={.calibrated=true,.valid=true};c.sense.heading_ok=true;
  o=demo_motion_step(&c,0,avoid(SM_NAV_BYPASS),&s,SM_NAV_CMD_PACK(SM_NAV_CMD_AVOID,SM_AVOID_REVERSE),false);
  CHECK(o.fault&&o.arg==SM_FAULT_SCAN);}

 /* ---- Sweep analysis ---- */
 uint16_t d[25];demo_profile_t p;
 for(int i=0;i<25;i++)d[i]=100;for(int i=8;i<=16;i++)d[i]=20;
 CHECK(demo_profile_analyse(d,&p)&&p.width_cm==14&&p.left_cm==100&&p.right_cm==100);
 CHECK(demo_profile_choose(&p)==SM_AVOID_LEFT); /* tie: left */
 /* No echo beyond the edges is open space, not a failed scan. */
 for(int i=0;i<8;i++)d[i]=511;
 CHECK(demo_profile_analyse(d,&p)&&p.width_cm==14&&(DEMO_SERVO_LEFT_LOW?p.left_cm:p.right_cm)==400);
 /* Low-angle side blocked at 35 cm: choose the other side. */
 for(int i=0;i<8;i++)d[i]=35;
 CHECK(demo_profile_analyse(d,&p)&&demo_profile_choose(&p)==(DEMO_SERVO_LEFT_LOW?SM_AVOID_RIGHT:SM_AVOID_LEFT));
 /* Both sides blocked: STOP. */
 for(int i=17;i<25;i++)d[i]=35;
 CHECK(demo_profile_analyse(d,&p)&&demo_profile_choose(&p)==SM_AVOID_STOP);
 /* Edge not inside the sweep, or no echo straight ahead: no profile. */
 for(int i=0;i<25;i++)d[i]=20;CHECK(!demo_profile_analyse(d,&p));
 for(int i=0;i<25;i++)d[i]=100;d[12]=511;CHECK(!demo_profile_analyse(d,&p));
 printf("Obstacle bypass and sweep: %u checks passed\n",checks);
 return 0;
}
