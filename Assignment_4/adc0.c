/*
 *
 *    	File:adc0.c
 *
 *  	Created on: Feb 14, 2017
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
 *      void ADC0_SETUP ()
 *      This routine is used to set up the ADC0
 *
 *      float convertToCelsius(int32_t adcSample)
 *		This routine is used to set convert the ADC sample to temtperature in celsius
 *
 *		float TEMP_CALC()
 *  	Function to read the ADC values and calculate the temperature in celsius
 *
 */


#include "adc0.h"

uint32_t frequency_value =0;			//variable to store frequency of clock

/************************************************************************************
 * Function is used to setup the ADC
 *
 * Input variable:  none
 * Global variable: frequency_value
 * Local variable:  none
 * Return type: none
 * Macros:ADC_PRESCALER,ADC_ACQ_TIME,ADC_INPUT,ADC_REFERENCE_VOLTAGE,ADC_RESOLUTION
 *
 ***********************************************************************************/
void ADC_SETUP ()
{
		uint8_t time_base = 0;
		frequency_value = CMU_ClockFreqGet(cmuClock_HFPER);
		time_base = ADC_TimebaseCalc(frequency_value);
		const ADC_Init_TypeDef adc0_init =
		{
			.ovsRateSel = _ADC_CTRL_PRESC_DEFAULT,			//Set oversampling to 0
			.lpfMode  = adcLPFilterBypass,					//No filter
			.warmUpMode = adcWarmupNormal,					// Normal Warm-up
			.prescale = ADC_PRESCALER,						// Prescaler of 50 required for Acquisition time of 7.14 useconds
			.tailgate = false,								//No tailgating required
			.timebase = time_base,							//Setting time base for warmup
		};

		ADC_Init(ADC0, &adc0_init);							//init ADC0

		const ADC_InitSingle_TypeDef adc0_single_init =
		{
				.acqTime = ADC_ACQ_TIME,						//Setting Acquisition time of 7.14 usecs ie 2 clock cycles
				.diff = false,								// Single ended input mode
				.input = ADC_INPUT,				//Temperature reference
				.leftAdjust = false,						//left adjust not required
				.prsEnable = false,							//prs enable not required
				.reference = ADC_REFERENCE_VOLTAGE,					//Reference Voltage of 1.25V
				.rep = true,								//repetition required
				.resolution = ADC_RESOLUTION,					//12 bit resolution
		};

		ADC_InitSingle(ADC0, &adc0_single_init);			//init ADC0 for Single Scan

		ADC0->IFC = INT_CLR_ALL;							//Clear all interrupts
		ADC0->IEN = ADC_IEN_SINGLE;							//enable single interrupt
		//NVIC_EnableIRQ(ADC0_IRQn);							//enable NVIC
}


/* **************************************************************************
 * Function is used to calculate temperature in degree celsius
 *
 * Input variable:  adcSample
 * Local variable: none
 * Global variable:none
 * Return type: float temp
 * MACRO: none
 *
 *COPYRIGHT: The below used routine has been taken from Silicon Labs
 ***************************************************************************/
float convertToCelsius(int32_t adcSample)
{
	float temp;
	float cal_temp_0 = (float)((DEVINFO->CAL& _DEVINFO_CAL_TEMP_MASK)>> _DEVINFO_CAL_TEMP_SHIFT);
	float cal_value_0 = (float)((DEVINFO->ADC0CAL2& _DEVINFO_ADC0CAL2_TEMP1V25_MASK)>> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
	float t_grad = -6.27;
	temp = (cal_temp_0 -((cal_value_0 - adcSample)  / t_grad));
	return temp;
}


/* **************************************************************************
 * Function to read the ADC values and calculate the temperature in celsius
 *
 * Input variable: none
 * Local variable: none
 * Global variable: conversion_count
 * Return type: float temp_C
 * MACRO:  TOTAL_CONNVERSION
 *
 *  ***************************************************************************/
float TEMP_CALC()
{
	uint16_t conversion_count = 0;			//variable to store the no of ADC conversions
	blockSleepMode(ADC_EM);									//Energy Mode 1
	float temp_C = 0;
	conversion_count = 0;
	uint32_t sum = 0;
	uint32_t average_value = 0;
	uint32_t timer_count = 0;
	uint32_t time_taken = 0;
	CMU_ClockEnable(cmuClock_TIMER0,true);
	TIMER_Enable(TIMER0,true);
	ADC_Start(ADC0,ADC_MODE);					//Start ADC
	while(conversion_count != TOTAL_CONVERSION)			//Check for 750 conversions
	{
		while(!(ADC0->IF & ADC_IFS_SINGLE));
		sum += ADC0->SINGLEDATA;						//Add the newest read value to take the sum
		conversion_count++;								//increment conversion count
		ADC_IntClear(ADC0,ADC_IFC_SINGLE);
	}
	TIMER_Enable(TIMER0,false);
	ADC0->CMD = ADC_CMD_SINGLESTOP;						//Disable ADC
	timer_count = TIMER0->CNT;
	time_taken = (timer_count )/frequency_value;
	unblockSleepMode(ADC_EM);								//Come out of Energy Mode 1
	average_value = sum / TOTAL_CONVERSION;				//Take average of the ADC readings
	temp_C = convertToCelsius(average_value);			//Calculate temperature in Celsius
	return temp_C;
}

