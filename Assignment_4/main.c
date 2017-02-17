/*
 *  File: main.c
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
 *
 *
 ********************************************************************************/


#include "em_timer.h"
#include "em_acmp.h"
#include "calibration.h"
#include "gpio_setup.h"
#include "letimer0.h"
#include "acmp0.h"
#include "adc0.h"
#include "dma.h"


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

  LIGHT_SENSOR_SETUP();						//Setup Light sensor
  GPIO_LED_SETUP();							//Setup the LED
  ACMP0_SETUP();							//Setup the ACMP0
  LETIMER0_SETUP();							//Setup the LETIMER
  LED_OFF(0);								//switch LED0 off
  LED_OFF(1);								//Switch LED1 of
  ADC_SETUP();								//Setup ADC
  /* Infinite loop */
  while (1)
  {
	  	sleep();
  }
}
