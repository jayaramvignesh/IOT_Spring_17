/*
 * 	File:led.h
 *
 *  Created on: Feb 14, 2017
 *  Author: Vignesh Jayaram
 *
 *
 *  LED_ON(uint8_t led_no)
 *	This routine is used to turn LED ON
 *
 *	LED_OFF(uint8_t led_no)
 *	This routine is used to turn LED OFF
 */

#ifndef LED_H
#define LED_H

#include "sleep.h"
#include "gpio_setup.h"

void LED_ON(uint8_t led_no);
void LED_OFF(uint8_t led_no);


#endif /* SRC_LED_H_ */
