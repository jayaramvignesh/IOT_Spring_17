/*
 *  File: main.c
 *
 *  Created on: Jan 31, 2017
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

#include "clock.h"
#include "sleep.h"
#include "gpio_led.h"


int main(void)
{
  /* Chip errata */
  CHIP_Init();
  CLOCK_SETUP();		//Setting up the clock
  LETIMER0_SETUP();		//Setting up LETIMER0 peripheral
  GPIO_LED_SETUP();		//Setting up the onboard LED

  /* Infinite loop */
  while (1)
  {
	  sleep();			//calling sleep routine
  }
}


