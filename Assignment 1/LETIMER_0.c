/*
 *
 *      File: LETIMER_0.c
 *
 *      Created on: Jan 31, 2017
 *      Author: Vignesh Jayaram
 *
 *      Permission is granted to anyone to use this software for any purpose,
 *  	including commercial applications, and to alter it and redistribute it
 *  	freely, subject to the following restrictions:
 *
 *  	1. The origin of this software must not be misrepresented; you must not
 *    	claim that you wrote the original software.
 *  	2. Altered source versions must be plainly marked as such, and must not be
 *   	 misrepresented as being the original software.
 *  	3. This notice may not be removed or altered from any source distribution.
 *
 *  	DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 *  	obligation to support this Software. Silicon Labs is providing the
 *  	Software "AS IS", with no express or implied warranties of any kind,
 *  	including, but not limited to, any implied warranties of merchantability
 *  	or fitness for any particular purpose or warranties against infringement
 *  	of any proprietary rights of a third party.
 *
 *  	Silicon Labs will not be liable for any consequential, incidental, or
 *  	special damages, or any other relief, or for any claim by any third party,
 *  	arising from your use of this Software.
 *
 *      LETIMER0_SETUP()
 *  	This function is used to setup the LETIMER0.
 *
 *  	LETIMER0_IRQHandler()
 *  	This interrupt handler is used to clear the interrupts and then toggle the LED.
 *
 */


#include "LETIMER_0.h"


/*********************************************
 * Interrupt Handler for  peripheral LETIMER0
 *
 * Input variable:  none
 * Global variable: none
 * Return type: none
 *
 *********************************************/
void LETIMER0_IRQHandler(void)
{
	INT_Disable();						// Disable all interrupts
	LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0 | LETIMER_IFC_COMP1); 	//clear the set interrupts
	GPIO_PinOutToggle(gpioPortE, 2);	//Toggle the onboard LED
	INT_Enable();						//Enable all interrupts
}



/* ****************************************************
 * Function is used to setup the LETIMER0 peripheral
 *
 * Input variable:  none
 * Local variable: none
 * Global variable:period,led_on,period_em3,led_on_em3
 * Return type: none
 * MACRO: ENERGY_MIN
 *
 *******************************************************/

void LETIMER0_SETUP(void)
{
    /*Set initial compare values for COMP0 and COMP1
	COMP1 keeps it's value and is used as TOP value
	for the LETIMER.
	COMP1 gets decremented through the program execution
	to generate a different PWM duty cycle */

	if(ENERGY_MIN==3)
	{
		LETIMER_CompareSet(LETIMER0, 0, period_em3);
		LETIMER_CompareSet(LETIMER0, 1, led_on_em3);
	}
	else
	{
		LETIMER_CompareSet(LETIMER0, 0, period);
	    LETIMER_CompareSet(LETIMER0, 1, led_on);
	}
	 LETIMER_IntClear(LETIMER0, INT_CLR_ALL); 	// Clears all interrupts LETIMER0_IFC_UF
	 LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP0|LETIMER_IEN_COMP1); // Enabling COMP0 and COMP1 interrupts
	 LETIMER_Init(LETIMER0, &letimerInit);		// Initialize LETIMER0
	 blockSleepMode(ENERGY_MIN);
	 NVIC_EnableIRQ(LETIMER0_IRQn);
	 LETIMER_Enable(LETIMER0, true);
}



