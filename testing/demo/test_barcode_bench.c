#include <assert.h>
#include "../../app_program/subsystem2/demo_motion.h"
int main(void) {
 demo_motion_t c={0};demo_line_t s={.calibrated=true,.valid=true,.mask=2};
 for(unsigned nav=0;nav<8;nav++) {
  demo_output_t o=demo_motion_step(&c,0,SM_STATE_NAVIGATING|(nav<<3),&s,0,false);
  assert(o.left==0&&o.right==0&&!o.event);
 }
 return 0;
}
