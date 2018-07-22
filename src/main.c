// Copyright (c) 2018 Robert Lee, Declan McIntosh
// University of Victoria ECE 299 Design Project

/*
 * This file is part of the �OS++ distribution.
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

/**
 * LED MATRIX LAYOUT:
 * ~~~~~~~~~~~TOP OF LED MATRIX~~~~~~~~~~
 * Columns:	0	1	2	3	4	5	6	7
 * Rows:
 * 		0
 * 		1
 * 		2
 * 		3
 * 		4
 * 		5
 * 		6
 * 		7
 */
#define NUMBER_OF_LEDS			(64)
#define REFRESH_RATE 			(250)//5000 will reduce the high pitched noise // this will be multiplied by 64 since there are 64 LEDs
#define FRAMES_PER_SECOND 		(25) // this number should be a factor of REFRESH_RATE
#define BUFFER_SIZE_SECONDS 	(10) // number of seconds for which the buffer stores data
// NOTE: The following row/col are NOT on the same bus
#define ROW_0 					(GPIO_PIN_4)
#define ROW_1 					(GPIO_PIN_2)
#define ROW_2 					(GPIO_PIN_7)
#define ROW_3 					(GPIO_PIN_6)
#define ROW_4					(GPIO_PIN_1)
#define ROW_5 					(GPIO_PIN_0)
#define ROW_6 					(GPIO_PIN_6)
#define ROW_7 					(GPIO_PIN_8)
#define COL_0 					(GPIO_PIN_9)
#define COL_1 					(GPIO_PIN_11)
#define COL_2 					(GPIO_PIN_2)
#define COL_3 					(GPIO_PIN_3)
#define COL_4 					(GPIO_PIN_7)
#define COL_5 					(GPIO_PIN_5)
#define COL_6 					(GPIO_PIN_5)
#define COL_7 					(GPIO_PIN_6)
#define BUTTON_1				(GPIO_PIN_1)
#define BUTTON_2				(GPIO_PIN_4)
#define BUTTON_3				(GPIO_PIN_1)

#define LEFT_TO_RIGHT			(0)
#define RIGHT_TO_LEFT			(1)
#define TOP_TO_BOTTOM			(2)
#define BOTTOM_TO_TOP			(3)

#define R0						(0)
#define R1						(1)
#define R2						(2)
#define R3						(3)
#define R4						(4)
#define R5						(5)
#define R6						(6)
#define R7						(7)

#define C0						(0)
#define C1						(1)
#define C2						(2)
#define C3						(3)
#define C4						(4)
#define C5						(5)
#define C6						(6)
#define C7						(7)

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

volatile char previous_button_reading_PA0 = 0;
volatile char button_state_PA0 = 0;
volatile char previous_button_reading_PC1 = 0;
volatile char button_state_PC1 = 0;
volatile char previous_button_reading_PC4 = 0;
volatile char button_state_PC4 = 0;
volatile char previous_button_reading_PB1 = 0;
volatile char button_state_PB1 = 0;

volatile char current_frame[NUM_OF_COLS];
volatile char display_buffer[FRAMES_PER_SECOND * BUFFER_SIZE_SECONDS][NUM_OF_COLS];
const int buffer_length = FRAMES_PER_SECOND * BUFFER_SIZE_SECONDS;
volatile int buffer_head = 0; // points to front of buffer
volatile int buffer_tail = -1; // points to next available spot

volatile char current_row = 0;
volatile char current_col = 0;
volatile int current_frame_number = 0;
const int times_to_repeat_frame = REFRESH_RATE / FRAMES_PER_SECOND;

int LED_Array_State = 0;

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

	Init_GPIO_Port_Default_Speed_Pull(BUTTON_1, GPIO_MODE_INPUT, 'C');
	Init_GPIO_Port_Default_Speed_Pull(BUTTON_2, GPIO_MODE_INPUT, 'C');
	Init_GPIO_Port_Default_Speed_Pull(BUTTON_3, GPIO_MODE_INPUT, 'B');
}

// utility inline functions to encapsulate the bus and port number of the row/col
inline void Write_Row_0 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOE, ROW_0, new_state); }
inline void Write_Row_1 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOE, ROW_1, new_state); }
inline void Write_Row_2 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOB, ROW_2, new_state); }
inline void Write_Row_3 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, ROW_3, new_state); }
inline void Write_Row_4 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, ROW_4, new_state); }
inline void Write_Row_5 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, ROW_5, new_state); }
inline void Write_Row_6 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOC, ROW_6, new_state); }
inline void Write_Row_7 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOC, ROW_7, new_state); }

inline void Write_Col_0 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOC, COL_0, new_state); }
inline void Write_Col_1 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOC, COL_1, new_state); }
inline void Write_Col_2 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, COL_2, new_state); }
inline void Write_Col_3 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, COL_3, new_state); }
inline void Write_Col_4 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOD, COL_4, new_state); }
inline void Write_Col_5 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOB, COL_5, new_state); }
inline void Write_Col_6 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOE, COL_6, new_state); }
inline void Write_Col_7 (uint16_t new_state) { HAL_GPIO_WritePin(GPIOE, COL_7, new_state); }

void Configure_LED_Display() {
	// init all rows and columns as output, medium speed, no pull
	Init_GPIO_Port_Default_Speed_Pull(ROW_0, GPIO_MODE_OUTPUT_PP, 'E');
	Init_GPIO_Port_Default_Speed_Pull(ROW_1, GPIO_MODE_OUTPUT_PP, 'E');
	Init_GPIO_Port_Default_Speed_Pull(ROW_2, GPIO_MODE_OUTPUT_PP, 'B');
	Init_GPIO_Port_Default_Speed_Pull(ROW_3, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(ROW_4, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(ROW_5, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(ROW_6, GPIO_MODE_OUTPUT_PP, 'C');
	Init_GPIO_Port_Default_Speed_Pull(ROW_7, GPIO_MODE_OUTPUT_PP, 'C');

	Init_GPIO_Port_Default_Speed_Pull(COL_0, GPIO_MODE_OUTPUT_PP, 'C');
	Init_GPIO_Port_Default_Speed_Pull(COL_1, GPIO_MODE_OUTPUT_PP, 'C');
	Init_GPIO_Port_Default_Speed_Pull(COL_2, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(COL_3, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(COL_4, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(COL_5, GPIO_MODE_OUTPUT_PP, 'B');
	Init_GPIO_Port_Default_Speed_Pull(COL_6, GPIO_MODE_OUTPUT_PP, 'E');
	Init_GPIO_Port_Default_Speed_Pull(COL_7, GPIO_MODE_OUTPUT_PP, 'E');

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

	Buffer_Init();
}

/**
 * Resets buffer head and tail for empty buffer
 */
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
void LED_Array_All_On() {
	for (int i = 0; i < NUM_OF_COLS; i++) {
		current_frame[i] = 0xFF;
	}
}

/**
 * turns off all LEDs for testing
 */
void LED_Array_All_Off() {
	for (int i = 0; i < NUM_OF_COLS; i++) {
		current_frame[i] = 0;
	}
}

void Toggle_LED_Array() {
	if (LED_Array_State) {
		LED_Array_All_Off();
		LED_Array_State = 0;
	} else {
		LED_Array_All_On();
		LED_Array_State = 1;
	}
}

/**
 * Fill buffer with frames with one LED at a time, cycling through all LEDs
 * Precondition: buffer is at least framesToRepeat*64 long
 */
void Test_LED_Array_Cycle_Through() {
	const int framesToRepeat = 3;
	char frame[NUM_OF_COLS];

	for (int i = 0; i < NUM_OF_COLS; i++)
	{
		frame[i] = 0;
	}
	for (int currentLED = 0; currentLED < NUMBER_OF_LEDS; currentLED++)
	{
		int col = currentLED/NUM_OF_COLS;
		int row = currentLED%NUM_OF_COLS;

		frame[col] = 1 << row;

		for (int i = 0; i < framesToRepeat; i++)
		{
			Buffer_Pushback(frame);
		}

		frame[col] = 0;
	}

	for (int i = 0; i < NUM_OF_COLS; i++) {
		frame[i] = 0;
	}

	Buffer_Pushback(frame);
}

/**
 * Creates a bar of height 'height' in the specified column 'col'
 * Col and height must be < NUM_OF_COLS
 */
void Create_Column_With_Height(char dest[], int col, int height) {
	char col_flag = 1 << col;
	for (int i = 1; i <= NUM_OF_COLS; i++)
	{
		if (i <= height) {
			dest[NUM_OF_COLS - i] = dest[NUM_OF_COLS - i] | col_flag; // force it to be 1
		} else {
			dest[NUM_OF_COLS - i] = dest[NUM_OF_COLS - i] & (0xFF ^ (col_flag)); // force it to be 0
		}
	}
}

/**
 * source is _source[frame number]
 * _source_length is the number of frames
 * message_length is the total number of columns in the message
 * direction is the direction the image pans
 */
void Fill_Buffer_With_Panning_Image(uint32_t _source_rows, uint32_t _source_cols, char source[_source_rows][_source_cols],
		int message_length, int direction) {
	char frame[NUM_OF_COLS];

//	char (*source)[_source_length] = _source;

	// Zero out the frame
	for (int i = 0; i < NUM_OF_COLS; i++) {
		frame[i] = 0;
	}

	// Start with blank frame
	Buffer_Pushback(frame);

	for (int current_index = 0; current_index < message_length;
			current_index++) {
		int source_index = current_index / NUM_OF_COLS;
		if (source_index >= _source_rows) { break; }
		int source_frame_index = current_index % NUM_OF_COLS;

		switch (direction) {
		case (LEFT_TO_RIGHT):
			for (int i = 0; i < NUM_OF_COLS; i++) {
				frame[i] = frame[i] << 1;

				char source_char = source[source_index][i];
				// truncate everything right of column
				char right_shifted = source_char >> (NUM_OF_COLS - 1 - source_frame_index);

	 			// remove everything left of column
				char right_col_only = right_shifted & (0xFE ^ 0xFF);

				// add in only the right column
				frame[i] = frame[i] | right_col_only;
			}
			break;
		case (BOTTOM_TO_TOP):
			for (int i = 0; i < NUM_OF_COLS - 1; i++) {
				// shift rows up by one
				frame[i] = frame[i + 1];
			}

			// add in new row at bottom
			frame[NUM_OF_COLS - 1] = source[source_index][source_frame_index];
			break;
		default:
			trace_printf("Invalid direction.");
			break;
		}

		Buffer_Pushback(frame);
		Buffer_Pushback(frame);
	}
}

/**
    111       111       111       111
  11   11   11   11   11   11   11   11
 1       1 1       1 1       1 1       1
1         1         1         1         1
 1       1 1       1 1       1 1       1
  11   11   11   11   11   11   11   11
    111       111       111       111

or in hex:

0x0e	0x03	0x80	0xe0	0x38	0x00
0x31	0x8c	0x63	0x18	0xc6	0x00
0x40	0x50	0x14	0x05	0x01	0x00
0x80	0x20	0x08	0x02	0x00	0x80
0x40	0x50	0x14	0x05	0x01	0x00
0x31	0x8c	0x63	0x18	0xc6	0x00
0x0e	0x03	0x80	0xe0	0x38	0x00
0x00	0x00	0x00	0x00	0x00	0x00
 */
void Create_Sine_Wave() {
	char sine_wave[6][NUM_OF_COLS] = {
			{ 0x0e, 0x31, 0x40, 0x80, 0x40, 0x31, 0x0e, 0x00 },
			{ 0x03, 0x8c, 0x50, 0x20, 0x50, 0x8c, 0x03, 0x00 },
			{ 0x80, 0x63, 0x14, 0x08, 0x14, 0x63, 0x80, 0x00 },
			{ 0xe0, 0x18, 0x05, 0x02, 0x05, 0x18, 0xe0, 0x00 },
			{ 0x38, 0xc6, 0x01, 0x00, 0x01, 0xc6, 0x38, 0x00 },
			{ 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00 },
			{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
	Fill_Buffer_With_Panning_Image(7, NUM_OF_COLS, sine_wave, 49, LEFT_TO_RIGHT);
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

	Configure_Ports();
	Configure_LED_Display();

//	LED_Array_All_On();
	LED_Array_All_Off();

//	Test_LED_Array_Cycle_Through();

	Create_Sine_Wave();

	// Start timers LAST to ensure that no interrupts based on timers will
	// trigger before initialization of board is complete
	ConfigureTimers();

	HAL_GPIO_WritePin( GPIOD, GPIO_PIN_12, 1);

	int previous_state_PA0 = 0;
	int previous_state_PC1 = 0;
	int previous_state_PC4 = 0;
	int previous_state_PB1 = 0;

	int column = 0;
	int height = 3;
	// Infinite loop
	while (1)
	{
		if (button_state_PA0) {
			previous_state_PA0 = 1;
		} else {
			if (previous_state_PA0) {
				//falling edge triggered
//				Toggle_LED_Array();
				Create_Column_With_Height(current_frame, column, height);
				column++;
				column %= NUM_OF_COLS;
				height++;
				height %= NUM_OF_COLS + 1;
			}
			previous_state_PA0 = 0;
		}

		if (button_state_PC1) {
			previous_state_PC1 = 1;
		} else {
			if (previous_state_PC1) {
				//falling edge triggered
				Toggle_LED_Array();
			}
			previous_state_PC1 = 0;
		}

		if (button_state_PC4) {
			previous_state_PC4 = 1;
		} else {
			if (previous_state_PC4) {
				//falling edge triggered
				Toggle_LED_Array();
			}
			previous_state_PC4 = 0;
		}

		if (button_state_PB1) {
			previous_state_PB1 = 1;
		} else {
			if (previous_state_PB1) {
				//falling edge triggered
				Toggle_LED_Array();
			}
			previous_state_PB1 = 0;
		}
	}
	//TODO add code upon stop execution of program
}

void TIM3_IRQHandler()//Timer3 interrupt function
{
	__HAL_TIM_CLEAR_FLAG( &DisplayTimer, TIM_IT_UPDATE );//clear flag status

	// This interrupt service routine is timer driven at 200 Hz
	// If the current reading is the same as the reading during the previous
	// interrupt, then the button state is reliable and we feed this to the rest
	// of the system

	// Check on board button
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
		// button is pressed.
		if (previous_button_reading_PA0) {
			// if this is consistent with previous reading, set state to 1
			button_state_PA0 = 1;
		}
		//update previous reading to current reading
		previous_button_reading_PA0 = 1;
	} else {
		// button is not pressed
		if (!previous_button_reading_PA0) {
			// if this is consistent with previous reading, set state to 0
			button_state_PA0 = 0;
		}
		//update previous reading to current reading
		previous_button_reading_PA0 = 0;
	}

	// Check PC1 button
	if (HAL_GPIO_ReadPin(GPIOC, BUTTON_1)) {
			// button is pressed.
		if (previous_button_reading_PC1) {
			// if this is consistent with previous reading, set state to 1
			button_state_PC1 = 1;
		}
		//update previous reading to current reading
		previous_button_reading_PC1 = 1;
	} else {
		// button is not pressed
		if (!previous_button_reading_PC1) {
			// if this is consistent with previous reading, set state to 0
			button_state_PC1 = 0;
		}
		//update previous reading to current reading
		previous_button_reading_PC1 = 0;
	}

	// Check PC4 button
	if (HAL_GPIO_ReadPin(GPIOC, BUTTON_2)) {
			// button is pressed.
		if (previous_button_reading_PC4) {
			// if this is consistent with previous reading, set state to 1
			button_state_PC4 = 1;
		}
		//update previous reading to current reading
		previous_button_reading_PC4 = 1;
	} else {
		// button is not pressed
		if (!previous_button_reading_PC4) {
			// if this is consistent with previous reading, set state to 0
			button_state_PC4 = 0;
		}
		//update previous reading to current reading
		previous_button_reading_PC4 = 0;
	}

	// Check PB1 button
	if (HAL_GPIO_ReadPin(GPIOB, BUTTON_3)) {
			// button is pressed.
		if (previous_button_reading_PB1) {
			// if this is consistent with previous reading, set state to 1
			button_state_PB1 = 1;
		}
		//update previous reading to current reading
		previous_button_reading_PB1 = 1;
	} else {
		// button is not pressed
		if (!previous_button_reading_PB1) {
			// if this is consistent with previous reading, set state to 0
			button_state_PB1 = 0;
		}
		//update previous reading to current reading
		previous_button_reading_PB1 = 0;
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

	if (current_row >= NUM_OF_COLS) {
		// at end of rows, need to advance to next column

		current_col++; //advance to next column
		current_row = 0; //restart row

		if (current_col >= NUM_OF_COLS) {
//TODO BUG potentially
			current_frame_number++;
			if (current_frame_number > times_to_repeat_frame) {
				current_frame_number = 0;

				Buffer_Pop(current_frame);
			}
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

	// for each case, turn off previous row, turn on current row
	switch(current_row) {
		case 0:
			if (current_frame[current_col] & 1 << current_row) { Write_Row_0(GPIO_PIN_SET); }
			Write_Row_7(GPIO_PIN_RESET);
			break;
		case 1:
			if (current_frame[current_col] & 1 << current_row) { Write_Row_1(GPIO_PIN_SET); }
			Write_Row_0(GPIO_PIN_RESET);
			break;
		case 2:
			if (current_frame[current_col] & 1 << current_row) { Write_Row_2(GPIO_PIN_SET); }
			Write_Row_1(GPIO_PIN_RESET);
			break;
		case 3:
			if (current_frame[current_col] & 1 << current_row) { Write_Row_3(GPIO_PIN_SET); }
			Write_Row_2(GPIO_PIN_RESET);
			break;
		case 4:
			if (current_frame[current_col] & 1 << current_row) { Write_Row_4(GPIO_PIN_SET); }
			Write_Row_3(GPIO_PIN_RESET);
			break;
		case 5:
			if (current_frame[current_col] & 1 << current_row) { Write_Row_5(GPIO_PIN_SET); }
			Write_Row_4(GPIO_PIN_RESET);
			break;
		case 6:
			if (current_frame[current_col] & 1 << current_row) { Write_Row_6(GPIO_PIN_SET); }
			Write_Row_5(GPIO_PIN_RESET);
			break;
		case 7:
			if (current_frame[current_col] & 1 << current_row) { Write_Row_7(GPIO_PIN_SET); }
			Write_Row_6(GPIO_PIN_RESET);
			break;
		default:
			//Should never enter this
			trace_printf("Invalid state in switch(current_row)");
			break;
	}

	current_row++; //move to next row
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
