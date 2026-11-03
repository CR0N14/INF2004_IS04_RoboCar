/*
 *  subsystem1.c
 *  Subsystem 1 -- WiFi Communication, Command and Telemetry + State
 *  Machine: creates its kernel objects and starts its tasks.
 *
 *  Tasks: sm_task (PRI_STATE_MACHINE), net_task (PRI_NETWORKING)
 */

#include "subsystem1.h"

EXPORT ER
subsystem1_init(void)
{
    ER const er = sm_init();

    return (E_OK != er) ? er : net_init();
}

EXPORT ER
subsystem1_start(void)
{
    ER const er = sm_start();

    return (E_OK != er) ? er : net_start();
}
