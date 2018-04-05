#ifndef CONTROL_INCLUSION_GUARD
#define CONTROL_INCLUSION_GUARD
#include "sensors.h"

/// \file control.h header for control.c

///Main control thread
/**
	This is the thread that actually performs the control operations. Note that it runs fairly slow (~5Hz), this is mostly because
	the thermocouple sensors are unable to run faster than that (max conversion time of 0.22s, average of 0.15s) and if polled faster
	don't ever update (conversions restart before finishing so value is never updated).

	\param pvParameters unused
 */
void vControlTask(void *pvParameters);
void vPollingTask(void *pvParameters);

///wake up the control thread
/**
	wakes up the control thread if it has started (that is reported the threadhandle)
 */
void wake_control_thread(void);

///Checks if the group is stable
/**
	Checks if the given group has been in the "stable" band for long enough.

	\param g group to check
	\return 1 if stable, 0 if not
*/
unsigned char is_stable(group *g);
//signed int count=0;
void flowMeterOptional_clear(unsigned int channel);
void flowMeterOptional_set(unsigned int channel);
unsigned int flowMeterOptional_get(unsigned int channel);
void reset(void);
#endif

