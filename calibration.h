/*
 *  File: calibration.c
 *
 *  Created on: Feb 8,2017
 *  Author: Vignesh Jayaram
 *
 *  LETIMER0_CALIB_SETUP(uint32_t count)
 *  This routine is used to set the LETIMER required for calibration
 *
 *	LETIMER_CALIBRATION()
 *	This routine is used to calibrate the  ULFRCO clock
 *
 *
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "sleep.h"
#include "timer.h"
#include "prescaler.h"



void LETIMER_CALIBRATION();
void LETIMER0_CALIB_SETUP(uint32_t count);


#endif /* SRC_CALIBRATION_H_ */
