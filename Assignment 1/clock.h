/*
 * 	File:clock.h
 *
 *  Created on: Jan 31, 2017
 *  Author: Vignesh Jayaram
 *
 *  CLOCK_SETUP()
 *  This routine is used to enable clocks to the peripheral.
 *
 */

#ifndef SRC_CLOCK_H_
#define SRC_CLOCK_H_

#include"sleep.h"

#define ENERGY_MIN 3			// Macro to define minimum energy mode

void CLOCK_SETUP(void);

#endif /* SRC_CLOCK_H_ */
