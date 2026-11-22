/* Timed barcode drive: straight at base duty, then the car posts STOP. */
#include <stdio.h>
#include "../../app_program/subsystem2/demo_motion.h"
#define CHECK(x) do{++checks;if(!(x)){printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#x);return 1;}}while(0)
static unsigned checks;
static uint32_t nav(sm_nav_mode_t n){return SM_STATE_NAVIGATING|((uint32_t)n<<3);}
int main(void){
 demo_motion_t c={0};demo_output_t o;
 demo_line_t s={.calibrated=true,.valid=true,.mask=0};
 /* All-white or all-black under the trackers must not stop the pass. */
 for(uint32_t t=0;t<DEMO_BARCODE_DRIVE_MS;t+=10){
  s.time_ms=t;s.mask=(uint8_t)((t/200u)%2u?7u:0u);
  o=demo_motion_step(&c,t,nav(SM_NAV_TRACKING),&s,0,false);
  CHECK(o.left==DEMO_BASE_DUTY&&o.right==DEMO_BASE_DUTY&&!o.event&&!o.fault);
 }
 s.time_ms=DEMO_BARCODE_DRIVE_MS;
 o=demo_motion_step(&c,DEMO_BARCODE_DRIVE_MS,nav(SM_NAV_TRACKING),&s,0,false);
 CHECK(!o.left&&!o.right&&o.event==SM_EVT_CMD_STOP);
 o=demo_motion_step(&c,DEMO_BARCODE_DRIVE_MS+10,nav(SM_NAV_TRACKING),&s,0,false);
 CHECK(!o.left&&!o.right&&!o.event); /* STOP is posted once */
 o=demo_motion_step(&c,DEMO_BARCODE_DRIVE_MS+20,SM_STATE_STANDBY,&s,0,false);
 CHECK(!o.left&&!o.right&&!o.event);
 /* A second START gets a fresh timer. */
 s.time_ms=50000;o=demo_motion_step(&c,50000,nav(SM_NAV_TRACKING),&s,0,false);
 CHECK(o.left==DEMO_BASE_DUTY&&o.right==DEMO_BASE_DUTY);
 /* Any other nav mode brakes; stale IR and pickup still fault. */
 o=demo_motion_step(&c,50010,nav(SM_NAV_MANOEUVRE),&s,0,false);CHECK(!o.left&&!o.right);
 c=(demo_motion_t){0};s.time_ms=0;
 o=demo_motion_step(&c,100,nav(SM_NAV_TRACKING),&s,0,false);CHECK(o.fault&&o.arg==SM_FAULT_IR_SENSOR&&!o.left);
 c=(demo_motion_t){0};s.time_ms=0;
 o=demo_motion_step(&c,0,nav(SM_NAV_TRACKING),&s,0,true);CHECK(o.fault&&o.arg==SM_FAULT_COLLISION&&!o.left);
 printf("Timed barcode drive: %u checks passed\n",checks);return 0;
}
