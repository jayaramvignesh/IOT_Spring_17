/*
 *  File: calibration.c
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
 *  LETIMER0_CALIB_SETUP(uint32_t count)
 *  This routine is used to set the LETIMER reqruired for calibration
 *
 *	LETIMER_CALIBRATION()
 *	This routine is used to calibrate the  ULFRCO clock
 *
 *
 *
 ********************************************************************************/

#include "calibration.h"

uint32_t total_count_LFXO = 0;
uint32_t total_count_ULFRCO = 0;
float calib_ratio=0;

/* ****************************************************
 * Function to set up LETIMER0 for calibration
 *
 * Input variable:  count
 * Local variable: none
 * Global variable: none
 * Return type: none
 * macro: none
 *******************************************************/
void LETIMER0_CALIB_SETUP(uint32_t count)
{
	/*structure defining the configurations of LETIMER0*/
	const LETIMER_Init_TypeDef letimer_calib_Init =
	{
	  .enable         = true,                   // Start counting when init completed.
	  .debugRun       = false,                  // Counter shall not keep running during debug halt.
	  .rtcComp0Enable = false,                  // Don't start counting on RTC COMP0 match.
	  .rtcComp1Enable = false,                  // Don't start counting on RTC COMP1 match.
	  .comp0Top       = true,                   // Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP
	  .bufTop         = false,                  // Don't load COMP1 into COMP0 when REP0 reaches 0.
	  .out0Pol        = 0,                      // Idle value for output 0.
	  .out1Pol        = 0,                      // Idle value for output 1.
	  .ufoa0          = letimerUFOANone,        // No changes on underflow
	  .ufoa1          = letimerUFOANone,        // No changes on underflow
	  .repMode        = letimerRepeatOneshot    // will run for only one time
	};
	 LETIMER_Init(LETIMER0, &letimer_calib_Init);	// Initialize LETIMER0
	 LETIMER0->CNT = count;
	 LETIMER_Enable(LETIMER0, true);
}

/***********************************************************************************
 * Routine for calibration
 *
 * Input variable:  none
 * Global variable: total_count_LFXO,total_count_ULFRCO,calib_ratio
 * Local variable:  none
 * Return type: none
 * Macros: ULFRCO_COUNT
 *
 ***********************************************************************************/
void LETIMER_CALIBRATION()
{
	while((LETIMER0->CNT)!=0);								//wait till LETIMER count reaches 0
	LETIMER_IntClear(LETIMER0, LETIMER_IFC_UF);				//clear interrupts
	total_count_LFXO = TIMER0->CNT;							//get the TIMER0 count
	total_count_LFXO |= (TIMER1->CNT << 16);				//Get the TIMER1 count and then calculate total count
	TIMER0->CNT =0;											//Reset the TIMER0 count
	TIMER1->CNT =0;											//Reset the TIMER1 count
	LETIMER_Enable(LETIMER0,false);							//Disable LETIMER0
	CLOCK_SETUP(cmuSelect_ULFRCO);							//Switch the clock to ULFRCO
	LETIMER0_CALIB_SETUP(ULFRCO_COUNT);						//Setup the count in LETIMER0 to 1000
	while((LETIMER0->CNT)!=0);								//wait till LETIMER count reaches 0
	LETIMER_IntClear(LETIMER0, LETIMER_IFC_UF);				//clear interrupts
	total_count_ULFRCO = TIMER0->CNT;						//get the TIMER0 count
	total_count_ULFRCO |= (TIMER1->CNT << 16);				//Get the TIMER1 count and then calculate total count
	calib_ratio = ((float)(total_count_LFXO))/((float)(total_count_ULFRCO));		//calculate the calibration ratio
	CMU_ClockEnable(cmuClock_TIMER0,false);								//Disable TIMER0
	CMU_ClockEnable(cmuClock_TIMER1,false);								//Disable TIMER1
}


