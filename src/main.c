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

#include <string.h> //for memcpy

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

#define NUMBER_OF_LEDS			(64)
#define REFRESH_RATE 			(250) // this will be multiplied by 64 since there are 64 LEDs
#define FRAMES_PER_SECOND 		(25) // this number should be a factor of REFRESH_RATE
#define BUFFER_SIZE_SECONDS 	(10) // number of seconds for which the buffer stores data
// NOTE: The following row/col are NOT on the same bus
#define COL_0 					(GPIO_PIN_4)
#define COL_1 					(GPIO_PIN_2)
#define COL_2 					(GPIO_PIN_7)
#define COL_3 					(GPIO_PIN_6)
#define COL_4					(GPIO_PIN_1)
#define COL_5 					(GPIO_PIN_0)
#define COL_6 					(GPIO_PIN_6)
#define COL_7 					(GPIO_PIN_8)
#define ROW_0 					(GPIO_PIN_9)
#define ROW_1 					(GPIO_PIN_11)
#define ROW_2 					(GPIO_PIN_2)
#define ROW_3 					(GPIO_PIN_3)
#define ROW_4 					(GPIO_PIN_7)
#define ROW_5 					(GPIO_PIN_5)
#define ROW_6 					(GPIO_PIN_5)
#define ROW_7 					(GPIO_PIN_6)

/**
 * Timer usage documentation:
 * TIM3 - polling of all button inputs, and debouncing
 * TIM4 - LED board drawing
 * TIM5 - FFT on input signal
 */
#define TIM3_PRIORITY 			(10)
#define TIM4_PRIORITY			(5)
#define TIM5_PRIORITY	 		(0)
#define NUM_OF_COLS				(8)

volatile char previous_button_reading = 0;
volatile char button_state = 0;
volatile char current_frame[NUM_OF_COLS];
volatile char display_buffer[FRAMES_PER_SECOND * BUFFER_SIZE_SECONDS][NUM_OF_COLS];
const int buffer_length = FRAMES_PER_SECOND * BUFFER_SIZE_SECONDS;
volatile int buffer_head = -1; // points to front of buffer
volatile int buffer_tail = -1; // points to next available spot

volatile char current_row = 0;
volatile char current_col = 0;
volatile int current_frame_number = 0;
const int times_to_repeat_frame = REFRESH_RATE / FRAMES_PER_SECOND;

void Init_GPIO_Port(uint32_t pin, uint32_t mode, uint32_t speed, uint32_t pull, char bus)
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

void Init_GPIO_Port_Default_Speed_Pull(uint32_t pin, uint32_t mode, char bus)
{
	Init_GPIO_Port(pin, mode, GPIO_SPEED_MEDIUM, GPIO_NOPULL, bus);
}

// important for these to be in global as they need to be accessed in interrupt service routine
TIM_HandleTypeDef	DisplayTimer;
TIM_HandleTypeDef 	LEDDisplayTimer;
void ConfigureTimers()
{
	__HAL_RCC_TIM3_CLK_ENABLE();
	DisplayTimer.Instance = TIM3;
	DisplayTimer.Init.Period = 49;//period & prescaler combination for 4 seconds count
	DisplayTimer.Init.Prescaler = 8399;
	DisplayTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
	DisplayTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init( &DisplayTimer );

	HAL_NVIC_SetPriority( TIM3_IRQn, TIM3_PRIORITY, TIM3_PRIORITY);
	//set priority for the interrupt. Value 0 corresponds to highest priority
	HAL_NVIC_EnableIRQ( TIM3_IRQn );//Enable interrupt function request of Timer3

	__HAL_TIM_ENABLE_IT( &DisplayTimer, TIM_IT_UPDATE );// Enable timer interrupt flag to be set when timer count is reached
	__HAL_TIM_ENABLE( &DisplayTimer );//Enable timer to start


	__HAL_RCC_TIM4_CLK_ENABLE();
	LEDDisplayTimer.Instance = TIM4;
	int prescaler = 105;
	LEDDisplayTimer.Init.Prescaler = prescaler - 1; // reduce to 800 kHz
	LEDDisplayTimer.Init.Period =  84000000 / prescaler / REFRESH_RATE / NUMBER_OF_LEDS - 1;
	// reduce to (refresh rate * Number of LEDs) frequency
	LEDDisplayTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
	LEDDisplayTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init( &LEDDisplayTimer );

	HAL_NVIC_SetPriority( TIM4_IRQn, TIM4_PRIORITY, TIM4_PRIORITY);
	//set priority for the interrupt. Value 0 corresponds to highest priority
	HAL_NVIC_EnableIRQ( TIM4_IRQn );//Enable interrupt function request of Timer3

	__HAL_TIM_ENABLE_IT( &LEDDisplayTimer, TIM_IT_UPDATE );// Enable timer interrupt flag to be set when timer count is reached
	__HAL_TIM_ENABLE( &LEDDisplayTimer );//Enable timer to start
}

void Configure_Ports()
{
	Init_GPIO_Port_Default_Speed_Pull(GPIO_PIN_12, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(GPIO_PIN_0, GPIO_MODE_INPUT, 'A');
}

// utility inline functions to encapsulate the bus and port number of the row/col
inline void Write_Col_0 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOE, COL_0, new_state); }
inline void Write_Col_1 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOE, COL_1, new_state); }
inline void Write_Col_2 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOB, COL_2, new_state); }
inline void Write_Col_3 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, COL_3, new_state); }
inline void Write_Col_4 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, COL_4, new_state); }
inline void Write_Col_5 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, COL_5, new_state); }
inline void Write_Col_6 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOC, COL_6, new_state); }
inline void Write_Col_7 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOC, COL_7, new_state); }

inline void Write_Row_0 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOC, ROW_0, new_state); }
inline void Write_Row_1 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOC, ROW_1, new_state); }
inline void Write_Row_2 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, ROW_2, new_state); }
inline void Write_Row_3 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, ROW_3, new_state); }
inline void Write_Row_4 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, ROW_4, new_state); }
inline void Write_Row_5 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOB, ROW_5, new_state); }
inline void Write_Row_6 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOE, ROW_6, new_state); }
inline void Write_Row_7 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOE, ROW_7, new_state); }

void Configure_LED_Display() {
	// init all rows and columns as output, medium speed, no pull
	Init_GPIO_Port_Default_Speed_Pull(COL_0, GPIO_MODE_OUTPUT_PP, 'E');
	Init_GPIO_Port_Default_Speed_Pull(COL_1, GPIO_MODE_OUTPUT_PP, 'E');
	Init_GPIO_Port_Default_Speed_Pull(COL_2, GPIO_MODE_OUTPUT_PP, 'B');
	Init_GPIO_Port_Default_Speed_Pull(COL_3, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(COL_4, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(COL_5, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(COL_6, GPIO_MODE_OUTPUT_PP, 'C');
	Init_GPIO_Port_Default_Speed_Pull(COL_7, GPIO_MODE_OUTPUT_PP, 'C');

	Init_GPIO_Port_Default_Speed_Pull(ROW_0, GPIO_MODE_OUTPUT_PP, 'C');
	Init_GPIO_Port_Default_Speed_Pull(ROW_1, GPIO_MODE_OUTPUT_PP, 'C');
	Init_GPIO_Port_Default_Speed_Pull(ROW_2, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(ROW_3, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(ROW_4, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(ROW_5, GPIO_MODE_OUTPUT_PP, 'B');
	Init_GPIO_Port_Default_Speed_Pull(ROW_6, GPIO_MODE_OUTPUT_PP, 'E');
	Init_GPIO_Port_Default_Speed_Pull(ROW_7, GPIO_MODE_OUTPUT_PP, 'E');

	// turn off all columns
	Write_Col_0(GPIO_PIN_RESET);
	Write_Col_1(GPIO_PIN_RESET);
	Write_Col_2(GPIO_PIN_RESET);
	Write_Col_3(GPIO_PIN_RESET);
	Write_Col_4(GPIO_PIN_RESET);
	Write_Col_5(GPIO_PIN_RESET);
	Write_Col_6(GPIO_PIN_RESET);
	Write_Col_7(GPIO_PIN_RESET);

	// turn off all rows
	Write_Row_0(GPIO_PIN_RESET);
	Write_Row_1(GPIO_PIN_RESET);
	Write_Row_2(GPIO_PIN_RESET);
	Write_Row_3(GPIO_PIN_RESET);
	Write_Row_4(GPIO_PIN_RESET);
	Write_Row_5(GPIO_PIN_RESET);
	Write_Row_6(GPIO_PIN_RESET);
	Write_Row_7(GPIO_PIN_RESET);
}

void Buffer_Clear()
{
	buffer_head = 0;
	buffer_tail = -1; // flag that the buffer is empty
}

/**
 * frame[] MUST have length NUM_OF_COLS
 * Returns 0 if buffer full, 1 if success
 */
char Buffer_Pushback(char frame[])
{
	if (buffer_tail == -1) {
		//buffer is empty, update actual available spot
		buffer_tail = 0;
	} else if (buffer_tail == buffer_head)
	{
		return 0; // buffer is full
	}

	// copy frame to buffer
	memcpy(display_buffer[buffer_tail], frame, NUM_OF_COLS);

	buffer_tail++; //increment to next available spot
	buffer_tail %= buffer_length; // wrap around to beginning of buffer

	return 1; //pushback success
}

/**
 * Pops front of buffer and copies to destination. If empty, nothing is copied.
 * dest[] MUST be length NUM_OF_COLS
 * Returns 0 if buffer empty, 1 if successfully copied
 */
char Buffer_Pop(char dest[])
{
	if (buffer_tail == -1)
	{
		return 0; //buffer is empty
	}

	// copy frame to buffer
	memcpy(dest, display_buffer[buffer_head], NUM_OF_COLS);

	buffer_head++;
	buffer_head %= buffer_length;

	if (buffer_head == buffer_tail) {
		//buffer is empty
		Buffer_Clear();
	}

	return 1;
}

/**
 * Initializes buffer with all 0's
 */
void Buffer_Init()
{
	char all_zeros[NUM_OF_COLS];
	for (int i = 0; i < NUM_OF_COLS; i++) {
		all_zeros[i] = 0;
	}

	for (int i = 0; i < buffer_length; i++) {
		memcpy(display_buffer[i], all_zeros, NUM_OF_COLS);
	}

	Buffer_Clear();
}

/**
 * turns on all LEDs for testing
 */
void Init_Testing_Image_LED_Array() {
	int length = sizeof(current_frame) / sizeof(current_frame[0]);

	for (int i = 0; i < length; i++) {
		current_frame[i] = 0xFF;
	}
}

int
main(int argc, char* argv[])
{
  // At this stage the system clock should have already been configured
  // at high speed.

	HAL_Init();// initializing HAL drivers

	__GPIOA_CLK_ENABLE(); // enabling clock for port A
	__GPIOB_CLK_ENABLE(); // enabling clock for port B
	__GPIOC_CLK_ENABLE(); // enabling clock for port C
	__GPIOD_CLK_ENABLE(); // enabling clock for port D
	__GPIOE_CLK_ENABLE(); // enabling clock for port E

	ConfigureTimers();
	Configure_Ports();
	Configure_LED_Display();

	Init_Testing_Image_LED_Array();
	Buffer_Init();

	int previous_state = 0;
	// Infinite loop
	while (1)
	{
		if (button_state) {
			previous_state = 1;
		} else {
			if (previous_state) {
				//falling edge triggered
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

/**
 * WARNING: The LED array MUST be advanced from
 * decreasing rows and columns to increasing rows/columns
 * as the logic is optimized for that situation
 */
void TIM4_IRQHandler() //Timer4 interrupt function
{
	__HAL_TIM_CLEAR_FLAG( &LEDDisplayTimer, TIM_IT_UPDATE ); //clear flag status

	if (current_row == 8) {
		// at end of rows, need to advance to next column

		current_col++; //advance to next column
		current_row = 0; //restart row

		if (current_col == 8) {
			current_frame_number++;

			//TODO grab next frame if number_of_repeated_frames > threshold
			//TODO use the function <code> void *memcpy(void *dest, const void *src, size_t n); </code>
			current_col = 0;
		}

		// columns only need to be updated when the column number updates
		// for each case, turn off previous column, turn on current column
		switch(current_col) {
			case 0:
				Write_Col_7(GPIO_PIN_RESET);
				Write_Col_0(GPIO_PIN_SET);
				break;
			case 1:
				Write_Col_0(GPIO_PIN_RESET);
				Write_Col_1(GPIO_PIN_SET);
				break;
			case 2:
				Write_Col_1(GPIO_PIN_RESET);
				Write_Col_2(GPIO_PIN_SET);
				break;
			case 3:
				Write_Col_2(GPIO_PIN_RESET);
				Write_Col_3(GPIO_PIN_SET);
				break;
			case 4:
				Write_Col_3(GPIO_PIN_RESET);
				Write_Col_4(GPIO_PIN_SET);
				break;
			case 5:
				Write_Col_4(GPIO_PIN_RESET);
				Write_Col_5(GPIO_PIN_SET);
				break;
			case 6:
				Write_Col_5(GPIO_PIN_RESET);
				Write_Col_6(GPIO_PIN_SET);
				break;
			case 7:
				Write_Col_6(GPIO_PIN_RESET);
				Write_Col_7(GPIO_PIN_SET);
				break;
			default:
				//Should never enter this
				trace_printf("Invalid state in switch(current_col)");
				break;
		}
	}

	if (current_frame[current_col] & 1 << current_row) {
		// for each case, turn off previous row, turn on current row
		switch(current_row) {
		    case 0:
		        Write_Row_7(GPIO_PIN_RESET);
		        Write_Row_0(GPIO_PIN_SET);
		        break;
		    case 1:
		        Write_Row_0(GPIO_PIN_RESET);
		        Write_Row_1(GPIO_PIN_SET);
		        break;
		    case 2:
		        Write_Row_1(GPIO_PIN_RESET);
		        Write_Row_2(GPIO_PIN_SET);
		        break;
		    case 3:
		        Write_Row_2(GPIO_PIN_RESET);
		        Write_Row_3(GPIO_PIN_SET);
		        break;
		    case 4:
		        Write_Row_3(GPIO_PIN_RESET);
		        Write_Row_4(GPIO_PIN_SET);
		        break;
		    case 5:
		        Write_Row_4(GPIO_PIN_RESET);
		        Write_Row_5(GPIO_PIN_SET);
		        break;
		    case 6:
		        Write_Row_5(GPIO_PIN_RESET);
		        Write_Row_6(GPIO_PIN_SET);
		        break;
		    case 7:
		        Write_Row_6(GPIO_PIN_RESET);
		        Write_Row_7(GPIO_PIN_SET);
		        break;
		    default:
		        //Should never enter this
		        trace_printf("Invalid state in switch(current_row)");
		        break;
		}
	}

	current_row++; //move to next row
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
