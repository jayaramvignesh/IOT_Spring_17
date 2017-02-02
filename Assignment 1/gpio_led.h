/*
 * 	File:gpio_led.h
 *
 *  Created on: Jan 31, 2017
 *  Author: Vignesh Jayaram
 *
 *  This routine is used to setup the on-board LED
 *
 */

#ifndef SRC_GPIO_LED_H_
#define SRC_GPIO_LED_H_

#include "em_gpio.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"

void GPIO_LED_SETUP();

#endif /* SRC_GPIO_LED_H_ */
