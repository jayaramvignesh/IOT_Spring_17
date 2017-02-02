/*
 * 	File: LETIMER_0.h
 *
 *  Created on: Jan 31, 2017
 *  Author: Vignesh Jayaram
 *
 *  LETIMER0_SETUP()
 *  This function is used to setup the LETIMER0.
 *
 *  LETIMER0_IRQHandler()
 *  This interrupt handler is used to clear the interrupts and then toggle the LED.
 *
 *
 */

#ifndef SRC_LETIMER_0_H_
#define SRC_LETIMER_0_H_

#include "em_letimer.h"
#include "clock.h"

#define PERIOD 1.75		// Time period of 1.75 seconds
#define ON_TIME 0.03		// LED will be on for 30 ms in every 1.75 seconds
#define INT_CLR_ALL 0x11111111	// Macro to clear all interrupts



uint16_t period = (PERIOD * 32768);		// Time period calculation for modes 0-2 {32768 because of 32.768 Khz clock}
uint16_t led_on = (ON_TIME * 32768);		// LED ON TIME period calculation for modes 0-2
uint16_t period_em3 = (PERIOD * 1000);	// Time period calculation for mode 3 { 1000 because of 1Khz clock}
uint16_t led_on_em3 = (ON_TIME * 1000);	// LED ON TIME period calculation for modes 3

/*structure defining the configurations of LETIMER0*/
const LETIMER_Init_TypeDef letimerInit =
{
  .enable         = true,                   // Start counting when init completed.
  .debugRun       = false,                  // Counter shall not keep running during debug halt.
  .bufTop         = false,                  // Don't load COMP1 into COMP0 when REP0 reaches 0.
  .out0Pol        = 0,                      // Idle value for output 0.
  .out1Pol        = 0,                      // Idle value for output 1.
  .ufoa0          = letimerUFOANone,        // No output action
  .ufoa1          = letimerUFOANone,        // No output action
  .repMode        = letimerRepeatFree,       // Count until stopped by SW
  .rtcComp0Enable = false,                  // Don't start counting on RTC COMP0 match.
  .rtcComp1Enable = false,                  // Don't start counting on RTC COMP1 match.
  .comp0Top       = true,                   // Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP
};

void LETIMER0_SETUP(void);
void LETIMER0_IRQHandler(void);

#endif /* SRC_LETIMER_0_H_ */
