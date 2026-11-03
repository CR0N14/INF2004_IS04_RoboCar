/*
 *	subsystem1.c
 *	Subsystem 1 -- WiFi Communication, Command and Telemetry + State Machine.  EMPTY TEMPLATE.
 *
 *	subsystem1_init():  create this subsystem's kernel objects
 *	                    (semaphores, mailboxes, ...).  Do not start
 *	                    tasks here.
 *	subsystem1_start(): create and start this subsystem's tasks, e.g.
 *	    ID tid = app_start_task(my_task, S1_PRI_MAIN, 0, "my_task");
 *	    return (tid < E_OK) ? tid : E_OK;
 *
 *	Drivers for this subsystem's hardware go in this folder as
 *	drv_WiFi Communication, Command and Telemetry + State Machine.c / drv_WiFi Communication, Command and Telemetry + State Machine.h, called only from the task that owns
 *	the peripheral (TEAM.md section 3).
 *
 *	Record every pin, task, object and interface in TEAM.md.
 */

#include "subsystem1.h"

EXPORT ER subsystem1_init(void)
{
	return E_OK;
}

EXPORT ER subsystem1_start(void)
{
	return E_OK;
}
