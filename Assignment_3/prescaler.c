/*
 * 	File:prescaler.c
 *
 *  Created on: Feb 8, 2017
 *  Author: Vignesh Jayaram
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
 *  prescaler_set()
 *  This routine is used to calculate the prescaler.
 *
 *
 */

#include "prescaler.h"

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

