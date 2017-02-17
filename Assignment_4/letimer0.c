/*
 *
 *      File: LETIMER_0.c
 *
 *      Created on: feb 8,2017
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
 *  	This interrupt handler is used to clear the interrupts and then toggle the LED based on ambient light.
 *
 */

#include "letimer0.h"

/* **************************************************************************
 * Function is used to setup the LETIMER0 peripheral
 *
 * Input variable:  none
 * Local variable: none
 * Global variable:letimer_prescaler,period_EM3,on_period_EM3,calib_ratio
 * Return type: none
 * MACRO: ENERGY_MIN,ULFRCO_COUNT,CALIBRATION,ONE_CYCLE
 *
 ***************************************************************************/
void LETIMER0_SETUP()
{
  /*Set initial compare values for COMP0 and COMP1
	COMP1 keeps it's value and is used as TOP value
	for the LETIMER.
	COMP1 gets decremented through the program execution
	to generate a different PWM duty cycle */

	/*structure defining the configurations of LETIMER0*/

	const LETIMER_Init_TypeDef letimerInit =
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
	  .repMode        = letimerRepeatFree       // Free running
	};

	prescaler_set();

	if(CALIBRATION == 1)
	{
		period_EM3 = (calib_ratio * ULFRCO_COUNT * ONE_CYCLE);		//calculate the new period for EM3 using calibration ratio
		on_period_EM3 = (calib_ratio * ULFRCO_COUNT * ON_TIME);		//calculate the new on-time for EM3 using calibration ratio
	}
	else
	{
		period_EM3 = (ULFRCO_COUNT * ONE_CYCLE);			//calculate the new period for EM3 using calibration ratio
		on_period_EM3 = (ULFRCO_COUNT * ON_TIME);			//calculate the new on-time for EM3 using calibration ratio
	}

	if(ENERGY_MIN == 3)										//Check for minimum energy mode
	{
		LETIMER_CompareSet(LETIMER0, 0, period_EM3);
		LETIMER_CompareSet(LETIMER0, 1, on_period_EM3);
	}
	else
	{
		CMU->LFAPRESC0 = (letimer_prescaler << 8);			//set prescalar
		LETIMER_CompareSet(LETIMER0, 0, period);
	    LETIMER_CompareSet(LETIMER0, 1, on_period);
	}

	 LETIMER_IntClear(LETIMER0, INT_CLR_ALL); 							// Clears all interrupts LETIMER0_IFC_UF
	 LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP0|LETIMER_IEN_COMP1);  // Enabling COMP0 and COMP1 interrupts
	 LETIMER_Init(LETIMER0, &letimerInit);								// Initialize LETIMER0
	 NVIC_EnableIRQ(LETIMER0_IRQn);										//Enable NVIC
	 blockSleepMode(ENERGY_MIN);										//Set energy mode for sleep routine
	 LETIMER_Enable(LETIMER0, true);
}
/*********************************************
 * Interrupt Handler for  peripheral LETIMER0
 *
 * Input variable:  none
 * Global variable: letimer_interrupt_flag,acmp_value
 * Return type: none
 * Macros: HIGH_LEVEL, LOW_LEVEL ,HIGH_TEMP_LIMIT_C, LOW_TEMP_LIMIT_C
 *********************************************/
void LETIMER0_IRQHandler(void)
{
	INT_Disable();									//Disable interrupts
	letimer_interrupt_flag = LETIMER_IntGet(LETIMER0);		//Check if COMP0 and COMP1 interrupt
	if(letimer_interrupt_flag & LETIMER_IF_COMP0)			//check for COMP0 interrupt
	{
		acmp_value = (ACMP0->STATUS & ACMP_STATUS_ACMPOUT);		//read the ACMP
		ACMP0->CTRL &= ~ACMP_CTRL_EN;							//Disable ACMP0
		GPIO_PinOutClear(LIGHT_EXCITE_PORT,LIGHT_EXCITE_PIN);	//Disable the excitation
		LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0);			//Clear COMP0 interrupt
		if(acmp_value)
		{
			if(acmpInit.vddLevel == LOW_LEVEL)							//Check for LOW threshold
			{
				acmpInit.vddLevel = HIGH_LEVEL;							//Change VDDlevel to HIGH THRESHOLD
				ACMP_Init(ACMP0,&acmpInit);								//Initialize ACMP0
				ACMP_ChannelSet(ACMP0, ACMP_REF, ACMP_CHANNEL);			//Set the channel
				LED_ON(0);
			}
			else
			{
				acmpInit.vddLevel = LOW_LEVEL;							//Change VDDlevel to LOW THRESHOLD
				ACMP_Init(ACMP0,&acmpInit);								//Initialize ACMP0
				ACMP_ChannelSet(ACMP0, ACMP_CHANNEL, acmpChannelVDD);	//Set channel
				GPIO_PinOutClear(LED0_PORT,LED0_PIN);					//Clear LED
				LED_OFF(0);
			}
		}

#ifdef DMA_REQD										//Check if DMA transfer or ADC polling
		DMA_Setup();								//Setup DMA
		blockSleepMode(ADC_EM);							//Enter Block Sleep Mode 1
		ADC_Start(ADC0, ADC_MODE);					//Enable ADC
#else
		float temp_celsius;
		temp_celsius = TEMP_CALC();					//Calculate the temperature
		if((temp_celsius > HIGH_TEMP_LIMIT_C) || (temp_celsius < LOW_TEMP_LIMIT_C))	//check if tempereature is withtin threshold
		{
			LED_ON(1);
		}
		else
		{
			LED_OFF(1);
		}
#endif
	}
	else
	{
		ACMP0->CTRL |= ACMP_CTRL_EN;						//Enable ACMP0
		GPIO_PinOutSet(LIGHT_EXCITE_PORT,LIGHT_EXCITE_PIN);	//Enable the excitation
		LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP1);		//Clear COMP1 interrupt
		while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;	//Check if warm-up is complete
	}
	INT_Enable();							//Enable interrupts
}
