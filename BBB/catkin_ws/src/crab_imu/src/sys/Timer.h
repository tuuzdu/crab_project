/*
 * Timer.h
 *
 *  Created on: 04.10.2013
 *      Author: tuuzdu
 */

#ifndef TIMER_H_
#define TIMER_H_

#define CLOCK_REALTIME				0
#define CLOCK_MONOTONIC				1
#define CLOCK_PROCESS_CPUTIME_ID	2
#define CLOCK_THREAD_CPUTIME_ID		3
#define CLOCK_MONOTONIC_RAW			4
#define CLOCK_REALTIME_COARSE		5
#define CLOCK_MONOTONIC_COARSE		6

unsigned long millis();

#endif /* TIMER_H_ */
