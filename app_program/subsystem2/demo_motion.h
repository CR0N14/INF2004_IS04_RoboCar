#ifndef DEMO_MOTION_H
#define DEMO_MOTION_H
#include <stdbool.h>
#include <stdint.h>
#include "../subsystem1/sm_types.h"
#include "../demo_config.h"
typedef struct {uint32_t time_ms; uint16_t raw[3],white[3],black[3]; uint16_t dark[3]; uint8_t mask,cal_phase; bool calibrated,valid,barcode_capturing; int steer;} demo_line_t;
/* aim: during the bypass, the servo angle S5 should measure at (else 90). */
typedef struct {int left,right; sm_evt_code_t event; uint16_t arg; bool fault,forward,aim; uint8_t aim_deg;} demo_output_t;
/* Latest heading (S4) and sonar reading (S5), refreshed by the caller every
   cycle before demo_motion_step(). Times are on the shared kernel ms clock. */
typedef struct {bool heading_ok,sonar_ok; int32_t heading_cdeg; uint16_t sonar_cm; uint8_t sonar_deg; uint32_t sonar_time;} demo_sense_t;
typedef struct {
 uint32_t entered,run_start,loss_since,found_since,cross_since,loss_sample,barcode_since;
 uint8_t nav,action,segment,failure; int last_direction;
 bool barcode_seen;
 bool running,loss_active,found_active,cross_active,left_line,failed,done;
 /* Bypass: start heading, sonar edge tracking for the current leg. */
 demo_sense_t sense; int32_t h0; uint32_t last_sonar;
 uint8_t reads,gone; bool h0_set,seen;
 /* Smooth tracking: last P term and damping, per fresh IR sample. */
 uint32_t track_sample; int track_p,track_d; bool track_valid;
} demo_motion_t;
demo_output_t demo_motion_step(demo_motion_t*,uint32_t,uint32_t,demo_line_t const*,uint8_t,bool);
#endif
