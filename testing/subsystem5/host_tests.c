/* Host compatibility types only. The tests include the real subsystem5.h,
   sm_types.h and production implementation; no driver implementation runs. */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_H
#define STATE_MACHINE_H
#define __TM_TMONITOR_H__
#define APP_TEST 1
#define EXPORT
#define E_OK 0
#define E_SYS (-5)
#define E_NOMEM (-33)
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_QOVR (-43)
#define E_TMOUT (-50)
#define TA_INHERIT 2u
#define TMO_FEVR (-1)
#define PRI_ULTRASOUND 14

typedef int INT, ER, ID, PRI, TMO;
typedef unsigned int UINT;
typedef uint32_t UW, RELTIM;
typedef unsigned char UB;
typedef void (*FP)(void);
typedef struct { UW mtxatr; } T_CMTX;
#include "../../app_program/subsystem1/sm_types.h"

static unsigned host_failures;
static INT tm_printf(const UB *format, ...)
{
    va_list args;
    INT result;
    if (strstr((const char *)format, "[TEST] %s FAIL") != NULL) ++host_failures;
    va_start(args, format);
    result = vprintf((const char *)format, args);
    va_end(args);
    return result;
}
#define CHECK(er, what) do { INT check_er_ = (INT)(er); \
    if (check_er_ < E_OK) tm_printf((UB*)"ERROR %s: %d\n", (what), check_er_); \
    } while (0)

#include "../../app_program/subsystem5/subsystem5_test.c"

int main(void)
{
    subsystem5_test();
    return host_failures ? 1 : 0;
}
