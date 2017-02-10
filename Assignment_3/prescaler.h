/*
 * 	File:prescaler.c
 *
 *  Created on: Feb 8, 2017
 *  Author: Vignesh Jayaram
 *
 *  prescaler_set()
 *  This routine is used to calculate the prescaler.
 *
 *
 */


#ifndef PRESCALER_H
#define PRESCALER_H

#include "sleep.h"

uint32_t period;
uint32_t on_period;
uint8_t letimer_prescaler;

#define LFXO_COUNT  32768			//ideal Count for 1 sec for LFXO
#define ULFRCO_COUNT  1000			//ideal count for 1 sec for ULFRCO

#define ONE_CYCLE 2.5				//one time cycle is 1.75 seconds
#define ON_TIME 0.004     			//LED shld be on for 4 milliseconds
#define LETIMER_MAX_COUNT 65535 	//Max count of Letimer


void prescaler_set(void);

#endif /* SRC_PRESCALER_H_ */
