/*
 * 	File:gpio_led.h
 *
 *  Created on: Feb 8, 2017
 *  Author: Vignesh Jayaram
 *
 *
 *  The routines described here are used to setup the on-board LED and the Light sensor
 *
 */

#ifndef GPIO_SETUP_H
#define GPIO_SETUP_H

#include "sleep.h"

#define LED0_PORT gpioPortE		//Macro to define LED0 PORT
#define LED0_PIN 2				//Macro to define LED0 PIN
#define LED1_PORT gpioPortE		//Macro to define LED1 PORT
#define LED1_PIN 3				//Macro to define LED1 PIN

#define LIGHT_SENSOR_PORT gpioPortC		//Macro to define LIGHT SENSOR PORT
#define LIGHT_SENSOR_PIN 6				//Macro to define LIGHT SENSOR PIN
#define LIGHT_EXCITE_PORT gpioPortD		//Macro to define LIGHT SENSOR EXCITE PORT
#define LIGHT_EXCITE_PIN 6				//Macro to define LIGHT SENSOR EXCITE PIN

void GPIO_LED_SETUP();
void LIGHT_SENSOR_SETUP();

#endif /* SRC_GPIO_SETUP_H_ */
