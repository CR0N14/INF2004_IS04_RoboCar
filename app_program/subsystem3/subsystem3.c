/*
 *	subsystem3.c
 *	Subsystem 3 -- Barcode Decoding and IR Line Following.  EMPTY TEMPLATE.
 *
 *	subsystem3_init():  create this subsystem's kernel objects
 *	                    (semaphores, mailboxes, ...).  Do not start
 *	                    tasks here.
 *	subsystem3_start(): create and start this subsystem's tasks, e.g.
 *	    ID tid = app_start_task(my_task, S3_PRI_MAIN, 0, "my_task");
 *	    return (tid < E_OK) ? tid : E_OK;
 *
 *	Drivers for this subsystem's hardware go in this folder as
 *	drv_Barcode Decoding and IR Line Following.c / drv_Barcode Decoding and IR Line Following.h, called only from the task that owns
 *	the peripheral (TEAM.md section 3).
 *
 *	Record every pin, task, object and interface in TEAM.md.
 */

#include "subsystem3.h"

EXPORT ER subsystem3_init(void)
{
	return E_OK;
}

EXPORT ER subsystem3_start(void)
{
	return E_OK;
}
