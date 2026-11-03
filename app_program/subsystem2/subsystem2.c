/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control.  EMPTY TEMPLATE.
 *
 *	subsystem2_init():  create this subsystem's kernel objects
 *	                    (semaphores, mailboxes, ...).  Do not start
 *	                    tasks here.
 *	subsystem2_start(): create and start this subsystem's tasks, e.g.
 *	    ID tid = app_start_task(my_task, S2_PRI_MAIN, 0, "my_task");
 *	    return (tid < E_OK) ? tid : E_OK;
 *
 *	Drivers for this subsystem's hardware go in this folder as
 *	drv_Motion Control.c / drv_Motion Control.h, called only from the task that owns
 *	the peripheral (TEAM.md section 3).
 *
 *	Record every pin, task, object and interface in TEAM.md.
 */

#include "subsystem2.h"

EXPORT ER subsystem2_init(void)
{
	return E_OK;
}

EXPORT ER subsystem2_start(void)
{
	return E_OK;
}
