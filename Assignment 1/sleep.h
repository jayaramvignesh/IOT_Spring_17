/*
 * 		File: sleep.h
 *
 *  	Created on: Jan 31, 2017
 *      Author: Vignesh Jayaram
 *
 *      sleep():
 *      This routine is used to enter into the desired energy modes
 *
 *      blockSleepMode()
 *      This routine is used to set the minimum energy mode. Once set, the energy mode does not change.
 *
 *      unblockSleepMode()
 *      This routine is used to exit from the set energy mode back to EM0.
 */

#ifndef SRC_SLEEP_H_
#define SRC_SLEEP_H_

#include <stdbool.h>
#include <stdint.h>
#include "em_letimer.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"

void sleep(void);
void blockSleepMode(int minMode);
void unblockSleepMode(int minMode);

#endif /* SRC_SLEEP_H_ */
