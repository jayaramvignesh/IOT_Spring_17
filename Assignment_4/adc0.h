/*
 * 	File:adc0.h
 *
 *  Created on: Feb 14, 2017
 *  Author: Vignesh Jayaram
 *
 *
 *	void ADC0_SETUP ()
 *	This routine is used to set up the ADC0
 *
 *	float convertToCelsius(int32_t adcSample)
 *	This routine is used to set convert the ADC sample to temtperature in celsius
 *
 *	float TEMP_CALC()
 *  Function to read the ADC values and calculate the temperature in celsius
 *
 *
 */


#ifndef ADC0_H
#define ADC0_H

#include "sleep.h"

#define ADC_MODE adcStartSingle			// Macro to define ADC scan mode
#define ADC_PRESCALER 49				// Macro to define ADC Prescaler
#define ADC_ACQ_TIME adcAcqTime2		//Macro to set the acquisition time clock cycles
#define ADC_INPUT adcSingleInputTemp	//Macro to define ADC input
#define ADC_REFERENCE_VOLTAGE adcRef1V25 //Macro to define ADC reference voltage
#define ADC_RESOLUTION adcRes12Bit		//Macro to define ADC Resolution
#define ADC_EM 1						//Macro to define the ADC sleep mode

#define LOW_TEMP_LIMIT_C 15				//Macro to define lower temperature limit
#define HIGH_TEMP_LIMIT_C 35			//Macro to define higher temperature limit

#define TOTAL_CONVERSION 750			//Macro to define total number of conversions required

void ADC_SETUP ();
float convertToCelsius(int32_t adcSample);
float TEMP_CALC();

#endif /* ADC0_H */
