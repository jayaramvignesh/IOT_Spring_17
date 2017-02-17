/*
 * 	File: LETIMER_0.h
 *
 *  Created on: feb 8,2017
 *  Author: Vignesh Jayaram
 *
 *  LETIMER0_SETUP()
 *  This function is used to setup the LETIMER0.
 *
 *  LETIMER0_IRQHandler()
 *  This interrupt handler is used to clear the interrupts and then toggle the LED based on the ambient light.
 *
 *
 */

#ifndef LETIMER0_H
#define LETIMER0_H

#include "sleep.h"
#include "acmp0.h"
#include "prescaler.h"
#include "gpio_setup.h"
#include "led.h"
#include "adc0.h"
#include "dma.h"

extern float calib_ratio;

#define CALIBRATION 1 			//Macro for calibration
#define ENERGY_MIN 2			// Macro to define minimum energy mode
//#define DMA_REQD

uint32_t on_period_EM3;
uint32_t period_EM3;
unsigned int acmp_value;

unsigned int letimer_interrupt_flag;

void LETIMER0_SETUP();
void LETIMER0_IRQHandler(void);


#endif /* SRC_LETIMER0_H_ */
