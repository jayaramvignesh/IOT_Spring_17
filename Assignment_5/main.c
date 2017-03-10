/*
 *  File: main.c
 *
 *  Created on: Feb 32, 2017
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
 ********************************************************************************/

#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "em_letimer.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "em_adc.h"
#include "em_dma.h"
#include "em_common.h"
#include "dmactrl.h"
#include "em_i2c.h"

/*********************MACRO DEFINITIONS********************************************/

#define ENERGY_MIN 3			// Macro to define minimum energy mode
#define INT_CLR_ALL 0xFFFFF	// Macro to clear all interrupts

#define LFXO_COUNT  32768		//ideal Count for 1 sec for LFXO
#define ULFRCO_COUNT  1000		//ideal count for 1 sec for ULFRCO

#define ONE_CYCLE 4.25			//one time cycle is 1.75 seconds
#define ON_TIME 0.004     	//LED shld be on for 4 milliseconds
#define LETIMER_MAX_COUNT 65535 //Max count of Letimer

#define LED0_PORT gpioPortE		//Macro to define LED0 PORT
#define LED0_PIN 2				//Macro to define LED0 PIN
#define LED1_PORT gpioPortE		//Macro to define LED1 PORT
#define LED1_PIN 3				//Macro to define LED1 PIN

#define LIGHT_SENSOR_PORT gpioPortC		//Macro to define LIGHT SENSOR PORT
#define LIGHT_SENSOR_PIN 6				//Macro to define LIGHT SENSOR PIN
#define LIGHT_EXCITE_PORT gpioPortD		//Macro to define LIGHT SENSOR EXCITE PORT
#define LIGHT_EXCITE_PIN 6				//Macro to define LIGHT SENSOR EXCITE PIN

#define HIGH_LEVEL 61					//Macro to define High Threshold
#define LOW_LEVEL 2						//Macro to define Low Threshold

#define ACMP_CHANNEL acmpChannel6		//Macro to define ACMP0 channel
#define ACMP_REF acmpChannelVDD			//Macro to define ACMP0 reference

#define CALIBRATION 1 					//Macro for calibration

#define DMA_REQD 						//Macro to define if DMA is reqd or not

#define DMA_CHANNEL_ADC 0   	      	//DMA channel 0
#define ADC0_DMA_ARBITRATE dmaArbitrate1024 //Macro to set arbitration to 1024
#define ADC0_DESTINATION_INCREMENT dmaDataInc2 //Macro for destination increment
#define ADC0_SOURCE_INCREMENT dmaDataIncNone   //Macro for source increment
#define ADC_DMA_TRANSFER_SIZE dmaDataSize2	   //Macro for hwo many bytes DMA should transfer

#define ADC_MODE adcStartSingle			// Macro to define ADC scan mode
#define ADC_PRESCALER 24				// Macro to define ADC Prescaler
#define ADC_ACQ_TIME adcAcqTime2		//Macro to set the acquisition time clock cycles
#define ADC_INPUT adcSingleInputTemp	//Macro to define ADC input
#define ADC_REFERENCE_VOLTAGE adcRef1V25//Macro to define ADC reference voltage
#define ADC_RESOLUTION adcRes12Bit		//Macro to define ADC Resolution
#define ADC_EM 1						//Macro to define the ADC sleep mode
#define ADC_REP true					//ADC rep mode

#define LOW_TEMP_LIMIT_C 15				//Macro to define lower temperature limit
#define HIGH_TEMP_LIMIT_C 25			//Macro to define higher temperature limit

#define TOTAL_CONVERSION 1000			//Macro to define total number of conversions required

#define LOCATION_ROUTE 0				//Macro to define ROUTE FOR I2C1
#define SLAVE_ADDR	0x39				//Macro to define SLAVE ADDRESS
#define I2C_WRITE	0						//Macro to define WRITE
#define I2C_READ	1						//Macro to define READ

#define TSL2651_PWR_PORT gpioPortD		//Macro to define POWER PORT FOR SENSOR TSL2651
#define TSL2651_PWR_PIN 0				//Macro to define POWER PIN FOR SENSOR TSL2651
#define TSL2651_INT_PORT gpioPortD		//Macro to define INTERRUPT PORT FOR SENSOR TSL2651
#define TSL2651_INT_PIN 1				//Macro to define INTERRUPT PIN FOR SENSOR TSL2651

#define I2C1_SCL_PORT gpioPortC			//Macro to define SCL port for I2C 1
#define I2C1_SDA_PORT gpioPortC			//Macro to define SDA port for I2C 1
#define I2C1_SCL_PIN 5					//Macro to define SCL pin for I2C 1
#define I2C1_SDA_PIN 4					//Macro to define SDA pin for I2C 1

#define THRESHOLD_LOW_LOW	0x82		//Macro to set the register for low threshold [low byte]
#define THRESHOLD_LOW_HIGH	0x83		//Macro to set the register for low threshold [high byte]
#define THRESHOLD_HIGH_LOW	0x84    	//Macro to set the register for high threshold [low byte]
#define THRESHOLD_HIGH_HIGH	0x85		//Macro to set the register for high threshold [high byte]
#define INTERRUPT 0x86					//Macro to set persistence register
#define TIMING_REG	0x81			    //Macro to set the timing register
#define CONTROL	0x80					//Macro to define the control register

//#define PASSIVE_LIGHT_SENSOR
#define GPIO_ODD_INT_NO 	1			 //Macro to set the GPIO interrupt number
#define SENSOR_THRESHOLD_LOW	0x000f	 //Macro to set the low threshold
#define SENSOR_THRESHOLD_HIGH	0x0800   //Macro to sete the high threshold
#define SENSOR_ADC0_LOW	0xAC			 //Macro to set the ADC0 data low register
#define SENSOR_ADC0_HIGH 0xAD			 //Macro to set the ADC0 data high register
#define	PERSISTENCE	0x04				 //Macro to set persistence to 4
#define INTR	0x01					 //Macro to select interrupt to level mode
#define SENSOR_THRESHOLD_LOW_LOW	0x0f //Macro to set the low threshold[low byte]
#define SENSOR_THRESHOLD_LOW_HIGH	0x00 //Macro to set the low threshold[high byte]
#define SENSOR_THRESHOLD_HIGH_LOW	0x00 //Macro to set the high threshold[low byte]
#define SENSOR_THRESHOLD_HIGH_HIGH	0x08 //Macro to set the high threshold[high byte]
#define INTEGRATION_TIME	0x01		 //Macro to set integration time to 101 ms
#define GAIN	0x00					 //Macro to set LOW GAIN
#define POWER_UP	0x03				 //Macro to give the power to the sensor
#define POWER_DOWN	0x00				 //Macro to disable the power
#define INTERRUPT_CLEAR	0xC6			 //Macro to clear the interrupts in control
#define INTERRUPT_DISABLE	0x04		 //Macro to disable interrupts
#define TIMING_REG_VAL	GAIN<<4|INTEGRATION_TIME //Macro to give the value to TIMING register
#define	INTERRUPT_REG_VAL	INTR<<4|PERSISTENCE	 //Macro to give the value to INTERRUPT register
#define I2C1_EM 1						 //Macro to define the I2C1 sleep mode

/*********************GLOBAL VARIABLES********************************************/
uint32_t total_count_LFXO = 0;			//variable to store TIMER count for LFXO
uint32_t total_count_ULFRCO = 0;		//variable to store TIMER count for ULFCRO
float calib_ratio=0;					//variable for calibration ratio
uint32_t on_period_EM3;					//variable for on period for EM3
uint32_t period_EM3;					//variable for period for EM3
uint32_t period;						//variable for period
uint32_t on_period;						//variable for on period
unsigned int letimer_interrupt_flag;	//variable to store the letimer interrupt
unsigned int acmp_value;				//variable to store ACMP value
uint8_t letimer_prescaler;				//variable to store calculate prescaler value
uint32_t frequency_value =0;			//variable to store frequency of clock
uint16_t conversion_count = 0;			//variable to store the no of ADC conversions
float temp_celsius = 0;					//variable to store the temperature

uint16_t data0low;						//variable to read the lower byte of sensor adc0
uint16_t data0high;						//variable to read the higher byte of sensor adc0
uint16_t data0;							//variable to store the total 16 bit data of adc sensor

uint8_t interrupt_cnt=0;

int sleep_block_counter[4] = {0, 0, 0,0};		//Array to determine which sleep mode to enter


/***********************************************************************************************
 * The below piece of code is used from Silicon Labs
 **********************************************************************************************/
#if ( ( DMA_CHAN_COUNT > 0 ) && ( DMA_CHAN_COUNT <= 4 ) )
#define DMACTRL_CH_CNT      4
#define DMACTRL_ALIGNMENT   128

#elif ( ( DMA_CHAN_COUNT > 4 ) && ( DMA_CHAN_COUNT <= 8 ) )
#define DMACTRL_CH_CNT      8
#define DMACTRL_ALIGNMENT   256

#elif ( ( DMA_CHAN_COUNT > 8 ) && ( DMA_CHAN_COUNT <= 12 ) )
#define DMACTRL_CH_CNT      16
#define DMACTRL_ALIGNMENT   256

#else
#error "Unsupported DMA channel count (dmactrl.c)."
#endif

/** DMA control block array, requires proper alignment. */
SL_ALIGN(DMACTRL_ALIGNMENT)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMACTRL_CH_CNT * 2] SL_ATTRIBUTE_ALIGN(DMACTRL_ALIGNMENT);

volatile uint16_t ramBufferAdcData[TOTAL_CONVERSION];		//Array to store the ADC values


/* ******************************************************************************
 * Function to set the minimum energy mode, so that energy mode does not go lower
 * Thus allowing to enter any other energy mode as required
 *
 * Input variable:  minMode
 * Global variable: sleep_block_counter
 * Local variable: none
 * Return type: none
 *
 * COPYRIGHT: The routine below has been taken from Silicon Labs
 *
 ********************************************************************************/
void blockSleepMode(int minMode)
{
	INT_Disable();
	sleep_block_counter[minMode]++;
	INT_Enable();
}

/* ******************************************************************************
 * This routine is used to exit from the set energy mode back to EM0.
 *
 * Input variable:  minMode
 * Global variable: sleep_block_counter
 * Local variable: none
 * Return type: none
 *
 * COPYRIGHT: The routine below has been taken from Silicon Labs
 ********************************************************************************/
void unblockSleepMode(int minMode)
{
	INT_Disable();
	if(sleep_block_counter[minMode] > 0)
	{
		sleep_block_counter[minMode]--;
	}
	INT_Enable();
}

/* ****************************************************
 * Function is used to enter the desired energy mode
 *
 * Input variable:  none
 * Global variable: sleep_block_counter
 * Local variable: none
 * Return type: none
 *
 * COPYRIGHT: The routine below has been taken from Silicon Labs
 *
 *******************************************************/
void sleep(void)
{
	if (sleep_block_counter[0] > 0){		//Go to Energy Mode 0
		return;
	}
	else if(sleep_block_counter[1] > 0){    //Go to Energy Mode 1
		EMU_EnterEM1();
	}
	else if(sleep_block_counter[2] > 0){	//Go to Energy Mode 2
		EMU_EnterEM2(true);
	}
	else if(sleep_block_counter[3] > 0){	//Go to Energy Mode 2
			EMU_EnterEM3(true);
	}
	else{
		EMU_EnterEM3(true);					//Go to Energy Mode 3
	}
}


/* ****************************************************
 * Function to set up the clock tree
 *
 * Input variable:  CMU_Osc_TypeDef osc
 * Local variable: none
 * Global variable: none
 * Return type: none
 * macro: ENERGY_MIN
 *******************************************************/
void CLOCK_SETUP(CMU_Osc_TypeDef osc)
{

	CMU_OscillatorEnable(osc, true, true);  	    		//Enabling low frequeuncy
	CMU_ClockSelectSet(cmuClock_LFA, osc);					//Selecting low frequency clock for LFA tree
	CMU_ClockEnable(cmuClock_CORELE, true);					//Enabling Low frequency Clock tree
	CMU_ClockEnable(cmuClock_LETIMER0, true);				//Enabling LFA clock tree to LETIMER0
	CMU_ClockEnable(cmuClock_GPIO, true);					//Enabling Clock to GPIO
	CMU_ClockEnable(cmuClock_ADC0, true);					//Enabling Clock to ADC0
	CMU_ClockEnable(cmuClock_DMA, true);					//Enabling Clock to DMA
	#ifdef PASSIVE_LIGHT_SENSOR
	CMU_ClockEnable(cmuClock_ACMP0, true);					//Enabling Clock to ACMP
	#endif
}

/* ****************************************************
 * Function to calculate PRESCALAR
 *
 * Input variable: none
 * Local variable: none
 * Global variable: period, on_period, letimer_prescaler
 * Return type: none
 * macro: ONE_CYCLE,LFXO_COUNT, LETIMER_MAX_COUNT,ON_TIME
 ********************************************************/
void prescaler_set(void)
{
	letimer_prescaler = 0;
	period = (ONE_CYCLE * LFXO_COUNT);						//calculate desired period
	on_period = (ON_TIME* LFXO_COUNT);						//calculate on_time
	while(period > LETIMER_MAX_COUNT)						//check if prescaler required
	{
		letimer_prescaler++;								//increment prescaler count
		period/=2;											//divide desired period by 2
		on_period/=2;										//divide on period by 2
	}
}

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


/******************************************************
 * Function is used to setup the onboard LEDS.
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: LED0_PORT, LED0_PIN, LED1_PORT, LED1_PIN
 *
 *******************************************************/
void GPIO_LED_SETUP()
{
	GPIO_PinModeSet(LED0_PORT, LED0_PIN, gpioModePushPull, 1);		// Configure Pe2 as push pull and output*/
	GPIO_DriveModeSet(LED0_PORT, gpioDriveModeStandard);	// Standard drive mode

	GPIO_PinModeSet(LED1_PORT, LED1_PIN, gpioModePushPull, 1);		// Configure Pe2 as push pull and output*/
	GPIO_DriveModeSet(LED1_PORT, gpioDriveModeStandard);	// Standard drive mode
}

/************************************************************************************
 * Function is used to setup the LIGHTSENSOR
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: LIGHT_SENSOR_PORT, LIGHT_EXCITE_PORT, LIGHT_SENSOR_PIN, LIGHT_EXCITE_PIN
 *
 ***********************************************************************************/
void LIGHT_SENSOR_SETUP()
{
	GPIO_DriveModeSet(LIGHT_SENSOR_PORT, gpioDriveModeStandard);	// Standard drive mode
	GPIO_DriveModeSet(LIGHT_EXCITE_PORT, gpioDriveModeStandard);	// Standard drive mode

	GPIO_PinModeSet(LIGHT_SENSOR_PORT, LIGHT_SENSOR_PIN, gpioModeDisabled, 0);	// Configure Light sensor
	GPIO_PinModeSet(LIGHT_EXCITE_PORT, LIGHT_EXCITE_PIN, gpioModePushPull, 0);	//Configure Excitation for sensor as pushpull
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
				.rep = ADC_REP,								//repetition required
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
 * Function to turn on the LED
 *
 * Input variable: led_on
 * Local variable: none
 * Global variable: none
 * Return type: none
 * MACRO:  LED0_PORT,LED1_PORT, LED1_PIN, LED1_PIN
 *
 ***************************************************************************/
void LED_ON(uint8_t led_no)
{
	if(led_no == 0)
	{
		GPIO_PinOutSet(LED0_PORT,LED0_PIN);						//Glow LED0
	}
	else
	{
		GPIO_PinOutSet(LED1_PORT,LED1_PIN);						//Glow LED0
	}
}

/* **************************************************************************
 * Function to turn on the LED
 *
 * Input variable: led_on
 * Local variable: none
 * Global variable: none
 * Return type: none
 * MACRO:  LED0_PORT,LED1_PORT, LED1_PIN, LED1_PIN
 *
 ***************************************************************************/
void LED_OFF(uint8_t led_no)
{
	if(led_no == 0)
	{
		GPIO_PinOutClear(LED0_PORT,LED0_PIN);						//Glow LED0
	}
	else
	{
		GPIO_PinOutClear(LED1_PORT,LED1_PIN);						//Glow LED0
	}
}

/* **************************************************************************
 * Function to reaad the ADC values and calulate the temperature in celsius
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
	blockSleepMode(ADC_EM);									//Energy Mode 1
	float temp_C = 0;
	conversion_count = 0;
	uint32_t sum = 0;
	uint32_t average_value = 0;
	ADC_Start(ADC0,ADC_MODE);					//Start ADC
	while(conversion_count != TOTAL_CONVERSION)			//Check for 750 conversions
	{
		while(!(ADC0->IF & ADC_IFS_SINGLE));
		sum += ADC0->SINGLEDATA;						//Add the newest read value to take the sum
		conversion_count++;								//increment conversion count
		ADC_IntClear(ADC0,ADC_IFC_SINGLE);
	}
	ADC0->CMD = ADC_CMD_SINGLESTOP;						//Disable ADC
	unblockSleepMode(ADC_EM);							//Come out of Energy Mode 1
	average_value = sum / TOTAL_CONVERSION;				//Take average of the ADC readings
	temp_C = convertToCelsius(average_value);			//Calculate temperature in Celsius
	return temp_C;
}

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
	 LETIMER_Enable(LETIMER0, true);
}

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


/*********************************************
 * Routine to setup ACMP0
 *
 * Input variable:  none
 * Global variable: none
 * Return type: none
 *
 *********************************************/
void ACMP0_SETUP()
{
	ACMP_Init(ACMP0,&acmpInit);								//Initialize ACMP0
	ACMP_ChannelSet(ACMP0, ACMP_REF, ACMP_CHANNEL);	//Set the channel
	ACMP_Enable(ACMP0);										//Enable ACMP0
}

/*********************************************
 * Call Back function to close off the DMA operation
 *
 * Input variable:  channel , primary, user pointer
 * Global variable: ramBufferAdcData
 * Return type: none
 * MACROS: TOTAL_CONVERSION,HIGH_TEMP_LIMIT_C,LOW_TEMP_LIMIT_C
 *
 *********************************************/
void DMA_ADC0_CB(unsigned int channel, bool primary, void *user)
{
    int32_t sum = 0;
    int32_t average;
    float temp_C = 0;
    INT_Disable();									//Disable interrupts

    DMA_IntClear(DMA_IFC_CH0DONE);					//clear the DMA interrupts

    ADC0->CMD = ADC_CMD_SINGLESTOP;					//Disable ADC
    unblockSleepMode(ADC_EM);

    for (int i=0; i < TOTAL_CONVERSION; i++)		//Add all the 750 samples
    {
            sum += ramBufferAdcData[i];
    }
    average = sum / TOTAL_CONVERSION;				//Take average of the samples

    temp_C = convertToCelsius(average);				//Calculate the temperature in celsius

    if((temp_C > HIGH_TEMP_LIMIT_C) || (temp_C < LOW_TEMP_LIMIT_C))	//check for temperature wrt thresholds
    {
    	LED_ON(1);
    }
    else
    {
    	LED_OFF(1);
    }

    INT_Enable();										//Re-enable the interrupts
}

DMA_CB_TypeDef adc0_cb =								// DMA callback structure
{
		.cbFunc = DMA_ADC0_CB,
		.userPtr = NULL,
		.primary = true
};


/*********************************************
 * Individual call to setup up the DMA Channel for the ADC0
 *
 * Input variable:  none
 * Global variable: ramBufferAdcData
 * Return type: none
 * MACROS: ADC0_DMA_ARBITRATE,ADC0_DESTINATION_INCREMENT,ADC_DMA_TRANSFER_SIZE,ADC0_SOURCE_INCREMENT
 * 			DMA_CHANNEL_ADC
 *
 *********************************************/
void DMA_ADC0_Setup(void)
{
    DMA_CfgDescr_TypeDef DMA_ADC0_cfg =
    {
    	.arbRate = dmaArbitrate1,					//Arbitrate factor set to 1024 cycles
		.dstInc = ADC0_DESTINATION_INCREMENT,			//Destination increment set to 2 bytes
		.hprot = 0,										//no protection
		.size = ADC_DMA_TRANSFER_SIZE,					//DMA transfers of 2 bytes
		.srcInc = ADC0_SOURCE_INCREMENT					//Do not increment source
    };

    DMA_CfgDescr(DMA_CHANNEL_ADC, true, &DMA_ADC0_cfg);	//Configure the descriptor

    DMA_CfgChannel_TypeDef DMA_ADC0_channel =
    {
    		.cb = &adc0_cb,								//Address of callback routine
			.enableInt = true,							//enable the interrupts
			.highPri = true,							//assign high priority
			.select = DMAREQ_ADC0_SINGLE				//ADC0 for single scan mode
    };

    DMA_CfgChannel(DMA_CHANNEL_ADC, &DMA_ADC0_channel);	//Configure the DMA channel 0

    DMA_IntClear(DMA_IFC_CH0DONE);						//Clear the interrupts
    DMA_IntEnable(DMA_IEN_CH0DONE);						//Enable channel 0 interrupt
    DMA_ActivateBasic(DMA_CHANNEL_ADC,					//Activate Basic mode for DMA channel 0
                        true,							//use primary descriptor
                        false,							//Burst mode disabled
                        (void *)ramBufferAdcData,		//destination address
                        (void *)&(ADC0->SINGLEDATA),	//source address
                        TOTAL_CONVERSION - 1);			//No of ADC reads required
}

/***************************************************************************************
 * Main call to set up the DMA descriptor location and all of the individual DMA channels
 *
 * Input variable:  none
 * Global variable: ramBufferAdcData
 * Return type: none
 * MACROS: none
 *
 *********************************************/
void DMA_Setup(void)
{
    DMA_Init_TypeDef DMA_init =
    {
    		.controlBlock = dmaControlBlock,
			.hprot = 0
    };
    DMA_Init(&DMA_init);    /* initial DMA dma Control Block descriptor location */

    /* call all the individual DMA setup routines */
    DMA_ADC0_Setup();				//Setup the individual descriptors and channels
    NVIC_EnableIRQ(DMA_IRQn);		//Enable the NVIC for DMA
}

/******************************************************
 * Function is used to setup the GPIO for I2C1.
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: I2C1_SCL_PORT, I2C1_SCL_PIN, I2C1_SDA_PORT, I2C1_SDA_PIN
 *
 *******************************************************/
void I2C_GPIO_SETUP()
{
	GPIO_PinModeSet(I2C1_SCL_PORT,I2C1_SCL_PIN, gpioModeWiredAnd, 1);		// Configure SCL for I2C1 as push pull and output*/
	GPIO_DriveModeSet(I2C1_SCL_PORT, gpioDriveModeLowest);				// Standard drive mode

	GPIO_PinModeSet(I2C1_SDA_PORT,I2C1_SDA_PIN, gpioModeWiredAnd, 1);		// Configure SDA for I2C1 as push pull and output*/
	GPIO_DriveModeSet(I2C1_SDA_PORT, gpioDriveModeLowest);				// Standard drive mode
}

/******************************************************
 * Function is used to setup the GPIO for external light sensor.
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: TSL2651_PWR_PORT, TSL2651_PWR_PIN, TSL2651_INT_PORT, TSL2651_INT_PIN
 *
 *******************************************************/
void TSL2651_GPIO_SETUP()
{
	GPIO_PinModeSet(TSL2651_PWR_PORT,TSL2651_PWR_PIN, gpioModePushPull, 0);	// Configure PWR PRT for TSL2651 as push pull and output*/
	GPIO_DriveModeSet(TSL2651_PWR_PORT, gpioDriveModeLowest);				// Standard drive mode

	GPIO_PinModeSet(TSL2651_INT_PORT,TSL2651_INT_PIN, gpioModeInput, 0);	// Configure INT PIN for TSL2651 as push pull and output*/
	GPIO_DriveModeSet(TSL2651_INT_PORT, gpioDriveModeLowest);				// Standard drive mode
	GPIO_PinOutClear(TSL2651_PWR_PORT,TSL2651_PWR_PIN);

}


/******************************************************
 * Function is used to reset the I2C
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: none
 *
 *******************************************************/
void I2C_RESET()
{
	if (I2C1->STATE & I2C_STATE_BUSY) 						//check if busy, if busy then abort
	{
	      I2C1->CMD = I2C_CMD_ABORT;
	 }
}


/******************************************************
 * Function is used to enable SCL
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: I2C1_SCL_PORT,I2C1_SCL_PIN
 *
 *******************************************************/
void SCL_ENABLE()
{
	GPIO_PinOutSet(I2C1_SCL_PORT,I2C1_SCL_PIN);

    for (int i=0;i<9;i++) {
           GPIO_PinOutClear(I2C1_SCL_PORT, I2C1_SCL_PIN);
           GPIO_PinOutSet(I2C1_SCL_PORT, I2C1_SCL_PIN);
       }
}

/******************************************************
 * Function is used to enable SDA
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: I2C1_SDA_PORT,I2C1_SDA_PIN
 *
 *******************************************************/
void SDA_ENABLE()
{
	GPIO_PinOutSet(I2C1_SDA_PORT,I2C1_SDA_PIN);
}


/*********************************************
 * Routine to Setup the I2C1
 *
 * Input variable:  none
 * Global variable: none
 * Return type: none
 * Macros: LOCATION_ROUTE
 *********************************************/
void I2C1_SETUP()
{
	CMU_ClockEnable(cmuClock_I2C1, true);					//Enabling Clock to I2C1
	I2C_Init_TypeDef i2c1_Init =
	{
			.enable = false,								//do not enavle on init
			.clhr = i2cClockHLRStandard,					//4:4 ratio
			.freq = I2C_FREQ_STANDARD_MAX,					//92KHz
			.master = true,									//Enable as MASTER
			.refFreq = 0									//no std ref
	};

	I2C1->ROUTE |= ((LOCATION_ROUTE << _I2C_ROUTE_LOCATION_SHIFT)|I2C_ROUTE_SDAPEN|I2C_ROUTE_SCLPEN); 	//Set the route 						//Setting the route for I2C1
	I2C_Init(I2C1,&i2c1_Init);								//Initializing the I2C
	I2C_RESET();
	SCL_ENABLE();
	SDA_ENABLE();
	I2C_IntClear(I2C1,INT_CLR_ALL);							//Clear all the interrupts
	I2C_IntEnable(I2C1,I2C_IEN_ACK|I2C_IEN_NACK|I2C_IEN_MSTOP);			//Enable ACK and NACK interrupts
	I2C_Enable(I2C1, true);
}

/*********************************************
 * Routine to setup the write for I2C1
 *
 * Input variable: reg_addr, data
 * Global variable: none
 * Return type: none
 * Macros: SLAVE_ADDR, WRITE
 *********************************************/
void I2C1_Write(uint8_t reg_addr, uint8_t data)
{
	I2C_IntClear(I2C1,INT_CLR_ALL);					//Clear all the interrupts
	I2C1->TXDATA = ((SLAVE_ADDR << 1 ) | I2C_WRITE);//sending slave address
	I2C1->CMD |= I2C_CMD_START;					 //Send the START command
	I2C1->IFC = I2C_IFC_START;

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	I2C1->TXDATA = reg_addr;					 //Send the register address

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	I2C1->TXDATA = data;					 	//Send the data

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	I2C1 -> CMD |= I2C_CMD_STOP;				 //Send the STOP command
	while ((I2C1->IF & I2C_IF_MSTOP) ==  0);  	 //check for MSTOP
	I2C1->IFC=I2C_IFC_MSTOP;					 //Clear the MSTOP interrupt
}

/*********************************************
 * Routine to setup the read for I2C1
 *
 * Input variable: reg_addr
 * Global variable: none
 * Return type: uint16_t
 * Macros: SLAVE_ADDR, WRITE, READ
 *********************************************/
uint8_t I2C1_Read(uint8_t reg_addr)
{
	I2C_IntClear(I2C1,INT_CLR_ALL);
	uint8_t data;
	I2C1->TXDATA = ((SLAVE_ADDR << 1 ) | I2C_WRITE);  //sending slave address
	I2C1->CMD |= I2C_CMD_START;					 	  //Send the START command
	I2C1->IFC = I2C_IFC_START;						  //Clear the START flag

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	I2C1->TXDATA = reg_addr;					 //Send the register address

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	I2C1->CMD |= I2C_CMD_START;					 //Send the START command
	I2C1->TXDATA = ((SLAVE_ADDR << 1 ) | I2C_READ);  //sending slave address
	I2C1->IFC = I2C_IFC_START;

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	while ((I2C1->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
	data =  I2C1->RXDATA;
	I2C1->CMD =I2C_CMD_NACK;

	I2C1 -> CMD |= I2C_CMD_STOP;				 //Send the STOP command
	while ((I2C1->IF & I2C_IF_MSTOP) ==  0);  	 //check for MSTOP
	I2C1->IFC=I2C_IFC_MSTOP;

	return data;
}

/*****************************************************************************************************
 * Routine to setup the TSL2651
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: THRESHOLD_LOW_LOW, SENSOR_THRESHOLD_LOW_LOW, THRESHOLD_LOW_HIGH,SENSOR_THRESHOLD_LOW_HIGH
 * 		   THRESHOLD_HIGH_LOW,SENSOR_THRESHOLD_HIGH_LOW,THRESHOLD_HIGH_HIGH,SENSOR_THRESHOLD_HIGH_HIGH,
 * 		   INTERRUPT,INTERRUPT_REG_VAL,TIMING_REG,TIMING_REG_VAL,CONTROL,POWER_UP
 *******************************************************************************************************/
void TSL2651_SETUP()
{
	I2C1_Write(CONTROL,POWER_UP);									//Powering up the device
	I2C1_Write(THRESHOLD_LOW_LOW,SENSOR_THRESHOLD_LOW_LOW);			//Setting the threshold in low low register
	I2C1_Write(THRESHOLD_LOW_HIGH,SENSOR_THRESHOLD_LOW_HIGH);		//Setting the threshold in low high register
	I2C1_Write(THRESHOLD_HIGH_LOW,SENSOR_THRESHOLD_HIGH_LOW);		//Setting the threshold in high low register
	I2C1_Write(THRESHOLD_HIGH_HIGH,SENSOR_THRESHOLD_HIGH_HIGH);		//Setting the threshold in high high register
	I2C1_Write(INTERRUPT,INTERRUPT_REG_VAL);						//Giving the persistence value of 4 and enbaling interrupt
	I2C1_Write(TIMING_REG,TIMING_REG_VAL);							//Giving integration time of 101 ms and a low gain
}

/*****************************************************************************************************
 * Routine to setup delay
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: none
 *******************************************************************************************************/

void delay(uint32_t delay_value)
{
	for(int i=0; i<delay_value ; i++);
}


/*****************************************************************************************************
 * Routine to power up the TSL2651
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: TSL2651_PWR_PORT,TSL2651_PWR_PIN
 ***************************************************************************************/
void TSL2651_POWERUP()
{
	GPIO_PinOutSet(TSL2651_PWR_PORT,TSL2651_PWR_PIN);				//Turnng on the device
	delay(10000);
	I2C1_SETUP();								//Setup the I2C1
	TSL2651_SETUP();												//Setup the TSL2651
	delay(10000);
}

/*****************************************************************************************************
 * Routine to power down the TSL2651
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: TSL2651_PWR_PORT,TSL2651_PWR_PIN,INTERRUPT_CLEAR,INTERRUPT_DISABLE
 ***************************************************************************************/
void TSL2651_POWERDOWN()
{
	I2C1_Write(INTERRUPT_CLEAR,INTERRUPT_DISABLE);						//Clear and the disable the interrupts
	I2C1_Write(CONTROL,POWER_DOWN);										//Disabling the power
	GPIO_INT_DISABLE();
	GPIO_PinOutClear(TSL2651_PWR_PORT, TSL2651_PWR_PIN);				//Switching the power
	GPIO_PinOutClear(I2C1_SCL_PORT, I2C1_SCL_PIN);
	GPIO_PinOutClear(I2C1_SDA_PORT, I2C1_SDA_PIN);
	I2C_Enable(I2C1,false);
	CMU_ClockEnable(cmuClock_I2C1, false);					//Enabling Clock to I2C1
}


/*****************************************************************************************************
 * Routine to GPIO interrupt enable
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: TSL2651_PWR_PORT,TSL2651_PWR_PIN
 ***************************************************************************************/

void GPIO_INT_ENABLE()
{
	 /*Configure PD1 interrupt on falling edge */
	 GPIO_IntConfig(TSL2651_INT_PORT, TSL2651_INT_PIN, false, true, true);
	 GPIO->IFC =INT_CLR_ALL;
	 NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	 NVIC_EnableIRQ(GPIO_ODD_IRQn);		//Enable GPIO_ODD interrupt vector in NVIC
}


/*****************************************************************************************************
 * Routine to GPIO interrupt disable
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: TSL2651_PWR_PORT,TSL2651_PWR_PIN
 ***************************************************************************************/

void GPIO_INT_DISABLE()
{
	 /*Configure PD1 interrupt on falling edge */
	 GPIO_ExtIntConfig(TSL2651_INT_PORT, TSL2651_INT_PIN,1, false, true, false);
	 GPIO->IFC =0xFFFF;
	 NVIC_DisableIRQ(GPIO_ODD_IRQn);		//Enable GPIO_ODD interrupt vector in NVIC
}


/*****************************************************************************************************
 * Routine to GPIO interrupt handler
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: SENSOR_ADC0_LOW,SENSOR_ADC0_HIGH,INTERRUPT_CLEAR,INTERRUPT_REG_VAL,SENSOR_THRESHOLD_LOW,SENSOR_THRESHOLD_HIGH
 ***********************************************************************************************************************/

void GPIO_ODD_IRQHandler(void)
{
  INT_Disable();							//Disable all interrupts
  GPIO_IntClear(INT_CLR_ALL);						//Clear the interrupts
  data0low = I2C1_Read(SENSOR_ADC0_LOW);   	// Reading from adc0 data0 low register
  data0high = I2C1_Read(SENSOR_ADC0_HIGH);	//Reading from adc0 data0 high register
  data0 = ((data0high << 8 ) + data0low);
  I2C1_Write(INTERRUPT_CLEAR,INTERRUPT_REG_VAL);	//Clear the interrupt on sensor and re-enable it
  if(data0 < SENSOR_THRESHOLD_LOW)				//Check for condition
  {
	  LED_ON(0);
  }
  else if(data0 > SENSOR_THRESHOLD_HIGH)
  {
	  LED_OFF(0);
  }

  INT_Enable();									//Emnable interrupts
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
	INT_Disable();											//Disable interrupts
	letimer_interrupt_flag = LETIMER_IntGet(LETIMER0);		//Check if COMP0 and COMP1 interrupt
	if(letimer_interrupt_flag & LETIMER_IF_COMP0)			//check for COMP0 interrupt
	{
		#ifdef DMA_REQD										//Check if DMA transfer or ADC polling
			DMA_Setup();									//Setup DMA
			blockSleepMode(ADC_EM);							//Enter Block Sleep Mode 1
			ADC_Start(ADC0, ADC_MODE);						//Enable ADC
		#else
			temp_celsius = TEMP_CALC();						//Calculate the temperature
			if((temp_celsius > HIGH_TEMP_LIMIT_C) || (temp_celsius < LOW_TEMP_LIMIT_C))	//check if tempereature is withtin threshold
			{
				LED_ON(1);
			}
			else
			{
				LED_OFF(1);
			}
		#endif
		#ifdef PASSIVE_LIGHT_SENSOR
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
		#else
		LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0);		//Clear interrupts
		if(interrupt_cnt == 0)								//Check for the interrupt count number
		{
			GPIO_INT_ENABLE();						//Enable the interrupts
			TSL2651_POWERUP();					//Power up the I2C sensor
			interrupt_cnt++;					//Increment interrupt count number
		}
		else if (interrupt_cnt == 1)
		{
			interrupt_cnt++;
		}
		else if(interrupt_cnt ==2)
		{
			TSL2651_POWERDOWN();				//Power down the I2C Sensor
			interrupt_cnt = 0;
		}
		#endif

	}
	else
	{
		#ifdef PASSIVE_LIGHT_SENSOR
		ACMP0->CTRL |= ACMP_CTRL_EN;						//Enable ACMP0
		GPIO_PinOutSet(LIGHT_EXCITE_PORT,LIGHT_EXCITE_PIN);	//Enable the excitation
		LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP1);		//Clear COMP1 interrupt
		while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;	//Check if warm-up is complete
		#else
		LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP1);		//Clear COMP1 interrupt
		#endif
	}
	INT_Enable();							//Enable interrupts
}


int main(void)

{
  /* Chip errata */
  CHIP_Init();

#if CALIBRATION == 1
  CLOCK_SETUP(cmuSelect_LFXO);				//Setup clock for LFXO
  TIMER0_SETUP();							//setup timer0
  TIMER1_SETUP();							//setup timer1
  LETIMER0_CALIB_SETUP(LFXO_COUNT);			//Setup LETIMER to LFXO for calibration
  LETIMER_CALIBRATION();					//Call calibration
#endif

  if(ENERGY_MIN ==3)						//Check for energy min mode and set clock
  {
	  CLOCK_SETUP(cmuSelect_ULFRCO);
  }
  else
  {
	  CLOCK_SETUP(cmuSelect_LFXO);
  }
  I2C_GPIO_SETUP();							//I2C_GPIO_SETUP
#if PASSIVE_LIGHT_SENSOR
  LIGHT_SENSOR_SETUP();						//Setup Light sensor
  ACMP0_SETUP();							//Setup the ACMP0
#else
  TSL2651_GPIO_SETUP();						//I2C SENSOR GPIO SETUP
#endif
  GPIO_LED_SETUP();							//Setup the LED
  ADC_SETUP();								//Setup ADC
  LETIMER0_SETUP();							//Setup the LETIMER0
  LED_OFF(0);								//switch LED0 off
  LED_OFF(1);								//Switch LED1 of
  blockSleepMode(ENERGY_MIN);
  while(1)
  {
	  	sleep();
  }
}
