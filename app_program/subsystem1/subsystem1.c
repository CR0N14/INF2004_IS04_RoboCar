/*
 *  subsystem1.c
 *  Subsystem 1 -- WiFi Communication, Command and Telemetry + State
 *  Machine: creates its kernel objects and starts its tasks.
 *
 *  Tasks: sm_task (PRI_STATE_MACHINE)
 */

#include "subsystem1.h"

EXPORT ER
subsystem1_init(void)
{
    return sm_init();
}

EXPORT ER
subsystem1_start(void)
{
    return sm_start();
}
