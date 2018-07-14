/*
 * AudioChip.h
 *
 *  Created on: May 15, 2016
 *      Author: Michael
 */

#ifndef INCLUDE_AUDIOCHIP_H_
#define INCLUDE_AUDIOCHIP_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "diag/Trace.h"
#include "stm32f4xx_hal.h"

//
// I2C interface to configure the external audio chip
//
extern I2C_HandleTypeDef
	I2c;

void
	EnableAudioCodecPassThru();

static HAL_StatusTypeDef
//	ReadRegister(uint8_t address, uint8_t * value),
//	SetAudioVolume(int volume),
//	StopAudioChip();
	WriteRegister(uint8_t address, uint8_t value);

#endif /* INCLUDE_AUDIOCHIP_H_ */
