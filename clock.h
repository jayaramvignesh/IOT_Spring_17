/*
 * 	File:clock.h
 *
 *  Created on: Feb 8, 2017
 *  Author: Vignesh Jayaram
 *
 *  CLOCK_SETUP()
 *  This routine is used to enable clocks to the peripheral.
 *
 */

#ifndef CLOCK_H
#define CLOCK_H

#include "sleep.h"

void CLOCK_SETUP(CMU_Osc_TypeDef osc);

#endif /* SRC_CLOCK_H_ */
