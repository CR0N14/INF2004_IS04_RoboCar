#ifndef DEMO_IR_H
#define DEMO_IR_H
#include "subsystem3.h"
#include "../subsystem2/demo_motion.h"
ER demo_ir_diag (uint8_t * out);
ER demo_ir_start (void);
ER demo_ir_read (demo_line_t *);
ER demo_ir_calibrate (UINT surface);
ER demo_ir_tlm (s3_tlm_t *);
#endif
/* End of file. */

