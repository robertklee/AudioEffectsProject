/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f4xx.h"
#include "ctype.h"
#include <sys/stat.h>
#include "stm32f4xx_hal.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F4 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

volatile int previous_button_reading = 0;
volatile int button_state = 0;

void init_GPIO_Port(uint32_t pin, uint32_t mode, uint32_t speed, uint32_t pull, char bus)
{
	GPIO_InitTypeDef GPIO_InitStructure; //a handle to initialize GPIO

	GPIO_InitStructure.Pin = pin;
	GPIO_InitStructure.Mode = mode;
	GPIO_InitStructure.Speed = speed;
	GPIO_InitStructure.Pull = pull;
	GPIO_InitStructure.Alternate = 0;
	if (bus == 'A') {
		HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	} else if (bus == 'B') {
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	} else if (bus == 'C') {
		HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	} else if (bus == 'D') {
		HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	} else if (bus == 'E') {
		HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
	}
}

void init_GPIO_Port_Default_Speed_Pull(uint32_t pin, uint32_t mode, char bus)
{
	init_GPIO_Port(pin, mode, GPIO_SPEED_MEDIUM, GPIO_NOPULL, bus);
}

TIM_HandleTypeDef	DisplayTimer;
void ConfigureTimer()
{
	__HAL_RCC_TIM3_CLK_ENABLE();
	DisplayTimer.Instance = TIM3;
	DisplayTimer.Init.Period = 49;//period & prescaler combination for 4 seconds count
	DisplayTimer.Init.Prescaler = 8399;
	DisplayTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
	DisplayTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init( &DisplayTimer );

	HAL_NVIC_SetPriority( TIM3_IRQn, 10, 10);
	//set priority for the interrupt. Value 0 corresponds to highest priority
	HAL_NVIC_EnableIRQ( TIM3_IRQn );//Enable interrupt function request of Timer3

	__HAL_TIM_ENABLE_IT( &DisplayTimer, TIM_IT_UPDATE );// Enable timer interrupt flag to be set when timer count is reached
	__HAL_TIM_ENABLE( &DisplayTimer );//Enable timer to start
}

void Configure_Ports()
{
	init_GPIO_Port_Default_Speed_Pull(GPIO_PIN_12, GPIO_MODE_OUTPUT_PP, 'D');
	init_GPIO_Port_Default_Speed_Pull(GPIO_PIN_0, GPIO_MODE_INPUT, 'A');
}

int
main(int argc, char* argv[])
{
  // At this stage the system clock should have already been configured
  // at high speed.

	HAL_Init();// initializing HAL drivers

	__GPIOD_CLK_ENABLE(); // enabling clock for port D
	__GPIOA_CLK_ENABLE(); // enabling clock for port D
	ConfigureTimer();
	Configure_Ports();

	int previous_state = 0;
	// Infinite loop
	while (1)
	{
		if (button_state) {
			previous_state = 1;
		} else {
			if (previous_state) {
				HAL_GPIO_TogglePin( GPIOD, GPIO_PIN_12);
			}
			previous_state = 0;
		}
	   // Add your code here.
	}
}

void TIM3_IRQHandler()//Timer3 interrupt function
{
	__HAL_TIM_CLEAR_FLAG( &DisplayTimer, TIM_IT_UPDATE );//clear flag status

	// This interrupt service routine is timer driven at 200 Hz
	// If the current reading is the same as the reading during the previous
	// interrupt, then the button state is reliable and we feed this to the rest
	// of the system
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
		// button is pressed.
		if (previous_button_reading) {
			// if this is consistent with previous reading, set state to 1
			button_state = 1;
		}
		//update previous reading to current reading
		previous_button_reading = 1;
	} else {
		// button is not pressed
		if (!previous_button_reading) {
			// if this is consistent with previous reading, set state to 0
			button_state = 0;
		}
		//update previous reading to current reading
		previous_button_reading = 0;
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
