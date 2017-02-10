/*
 * 	File:gpio_led.c
 *
 *  Created on: Feb 8,2017
 *  Author: Vignesh Jayaram
 *
 * 	Permission is granted to anyone to use this software for any purpose,
 *  including commercial applications, and to alter it and redistribute it
 *  freely, subject to the following restrictions:
 *
 *  1. The origin of this software must not be misrepresented; you must not
 *  claim that you wrote the original software.
 *  2. Altered source versions must be plainly marked as such, and must not be
 *  misrepresented as being the original software.
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
 *  GPIO_LED_SETUP()
 *  This routine is used to setup the on-board LED
 *
 *
 *	LIGHT_SENSOR_SETUP()
 *	This routine is used to setup the on-board ambient light sensor
 *
 */


#include "gpio_setup.h"

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
	GPIO_PinModeSet(LIGHT_EXCITE_PORT, LIGHT_EXCITE_PIN, gpioModePushPull, 1);	//Configure Excitation for sensor as pushpull
}
