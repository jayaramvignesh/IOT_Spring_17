/*
 *  File: timer.c
 *
 *  Created on: Feb 8,2017
 *  Author: Vignesh Jayaram
 *
 *
 *  Permission is granted to anyone to use this software for any purpose,
 *  including commercial applications, and to alter it and redistribute it
 *  freely, subject to the following restrictions:
 *
 *  1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 *  2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *  3. This notice may not be removed or altered from any source distribution.
 *
 *  DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 *  obligation to support this Software. Silicon Labs is providing the
 *  Software "AS IS", with no express or implied warranties of any kind,
 *  including, but not limited to, any implied warranties of merchantability
 *  or fitness for any particular purpose or warranties against infringement
 *  of any proprietary rights of a third party.
 *
 *  Silicon Labs will not be liable for any consequential, incidental, or
 *  special damages, or any other relief, or for any claim by any third party,
 *  arising from your use of this Software.
 *
 *  TIMER0_SETUP()
 * 	Setting up TIMER0
 *
 * 	TIMER1_SETUP
 *	Setting up TIMER1
 *
 *
 *
 ********************************************************************************/
#include "timer.h"



/* ****************************************************
 * Function to set up TIMER0
 *
 * Input variable:  none
 * Local variable: none
 * Global variable: none
 * Return type: none
 * macro: none
 *******************************************************/
void TIMER0_SETUP()
{
	CMU_ClockEnable(cmuClock_HFPER,true);				// Enabling High Frequency[14MHz] clock
	CMU_ClockEnable(cmuClock_TIMER0,true);				// Enabling clock to TIMER0

	/* Creating struct to set the TIMER0 parameters */
	TIMER_Init_TypeDef timer0Init =
	  {
	    .enable     = true, 							//Start counting when init is completed
	    .debugRun   = false, 							//Counter stops counting during debug
	    .prescale   = timerPrescale1,					//No prescaler required
	    .mode       = timerModeUp, 						//Timer to be used in up counter mode
	    .dmaClrAct  = false,							// No DMA clear request
	    .quadModeX4 = false, 							//No quad mode used
		.clkSel     = timerClkSelHFPerClk, 				//Select clock to High frequency
		.fallAction = timerInputActionNone, 			//No action on falling input edge
	    .riseAction = timerInputActionNone, 			//No action on rising input edge
	    .oneShot    = false, 							//Continuous counting.
	    .sync       = true, 							//In synchronization with other timers
	  };
	TIMER_Init(TIMER0, &timer0Init);					//Initializing TIMER0
}

/* ****************************************************
 * Function to set up TIMER1 in cascade mode
 *
 * Input variable:  none
 * Local variable: none
 * Global variable: none
 * Return type: none
 * macro: none
 *******************************************************/
void TIMER1_SETUP()
{
	CMU_ClockEnable(cmuClock_HFPER,true);				// Enabling High Frequency[14MHz] clock
	CMU_ClockEnable(cmuClock_TIMER1,true);				// Enabling clock to TIMER1

	/* Creating struct to set the TIMER1 parameters */
	TIMER_Init_TypeDef timer1Init =
	  {
	    .enable     = true, 							//Start counting when init is completed
	    .debugRun   = false, 							//Counter stops counting during debug
	    .prescale   = timerPrescale1,					//No prescaler required
	    .mode       = timerModeUp, 						//Timer to be used in up counter mode
	    .dmaClrAct  = false,							//No DMA clear request
	    .quadModeX4 = false, 							//No quad mode used
		.clkSel     = timerClkSelCascade, 				//Select clock to Cascade mode. Clock gets triggered on UF/OF of TIMER0
		.fallAction = timerInputActionNone, 			//No action on falling input edge
	    .riseAction = timerInputActionNone, 			//No action on rising input edge
	    .oneShot    = false, 							//Continuous counting.
	    .sync       = true, 							//In synchronization with other timers
	  };

	TIMER_Init(TIMER1, &timer1Init);					//Initializing TIMER1
}

