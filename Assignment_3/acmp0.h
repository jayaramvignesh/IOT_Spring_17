/*
 * 	File:acmp0.h
 *
 *  Created on: Feb 8, 2017
 *  Author: Vignesh Jayaram
 *
 *
 *  The routine is used to setup the analog comparator
 *
 */

#ifndef ACMP0_H
#define ACMP0_H

#include "sleep.h"

#define HIGH_LEVEL 61					//Macro to define High Threshold
#define LOW_LEVEL 2					//Macro to define Low Threshold


/*********************************************
 * Structure to configure ACMP0
 *
 * Input variable:  none
 * Global variable: none
 * Return type: none
 * Macro: LOW_LEVEL
 *
 *********************************************/
static ACMP_Init_TypeDef acmpInit =
  {
	    .fullBias                 = false,
	    .halfBias                 = true,
	    .biasProg                 = 0x0,
	    .interruptOnFallingEdge   = false,
	    .interruptOnRisingEdge    = false,
	    .warmTime                 = acmpWarmTime256,
	    .hysteresisLevel          = acmpHysteresisLevel4,
	    .inactiveValue            = false,
	    .lowPowerReferenceEnabled = false,
	    .vddLevel                 = LOW_LEVEL,
	    .enable                   = false
  };

void ACMP0_SETUP();


#endif /* SRC_ACMP0_H_ */
