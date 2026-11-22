#ifndef DEMO_MOTION_H
#define DEMO_MOTION_H
#include <stdbool.h>
#include <stdint.h>
#include "../subsystem1/sm_types.h"
#include "../demo_config.h"
typedef struct {uint32_t time_ms; uint16_t raw[3],white[3],black[3]; uint8_t mask,cal_phase; bool calibrated,valid; int steer;} demo_line_t;
typedef struct {int left,right; sm_evt_code_t event; uint16_t arg; bool fault,forward;} demo_output_t;
typedef struct {
 uint32_t entered,run_start,loss_since,found_since,cross_since;
 uint8_t nav,action,segment,failure; int last_direction;
 bool running,loss_active,found_active,cross_active,left_line,failed,done;
} demo_motion_t;
demo_output_t demo_motion_step(demo_motion_t*,uint32_t,uint32_t,demo_line_t const*,uint8_t,bool);
#endif
