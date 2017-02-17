/*
 *
 *    	File:led.c
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
 *       LED_ON(uint8_t led_no)
 *		 This routine is used to turn LED ON
 *
 *		 LED_OFF(uint8_t led_no)
 *		 This routine is used to turn LED OFF
 *
 */

#include "led.h"

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

