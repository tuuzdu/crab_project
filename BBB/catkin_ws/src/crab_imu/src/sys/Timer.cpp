/*
 * Timer.cpp
 *
 *  Created on: 04.10.2013
 *      Author: tuuzdu
 */

#include "Timer.h"
#include <time.h>

unsigned long millis(void) {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000L);
}
