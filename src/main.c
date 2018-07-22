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

#include <hamming.h>
#include "stm32f4xx_hal.h"
#include "math.h"

#include "arm_math.h"
#include "arm_const_structs.h"
#include "main.h"
#include "hamming.h"
#include "windowing_fft.h"
#include "AudioChip.h"

// TODO move #define to header

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

#define NO_EFFECT				(0)
#define ENABLE_ECHO				(1)
#define ENABLE_PITCH_SHIFT		(1)

#define SEGMENT_A 				( GPIO_PIN_0 )
#define SEGMENT_B				( GPIO_PIN_0 )
#define SEGMENT_C				( GPIO_PIN_0 )
#define SEGMENT_D				( GPIO_PIN_0 )
#define SEGMENT_E				( GPIO_PIN_0 )
#define SEGMENT_F				( GPIO_PIN_0 )
#define SEGMENT_G				( GPIO_PIN_0 )

#define LED_DIGIT_1 			( GPIO_PIN_0 )
#define LED_DIGIT_2				( GPIO_PIN_0 )
#define LED_DIGIT_3				( GPIO_PIN_0 )
#define LED_DIGIT_4				( GPIO_PIN_0 )

#define SWITCH_1				( GPIO_PIN_0 )
#define SWITCH_2				( GPIO_PIN_0 )

volatile char previous_button_reading_PA0 = 0;
volatile char button_state_PA0 = 0;
volatile char previous_button_reading_PC1 = 0;
volatile char button_state_PC1 = 0;
volatile char previous_button_reading_PC4 = 0;
volatile char button_state_PC4 = 0;
volatile char previous_button_reading_PB1 = 0;
volatile char button_state_PB1 = 0;

char previous_state_PC1 = 0;
char previous_state_PC4 = 0;
char previous_state_PB1 = 0;

volatile char current_frame[NUM_OF_COLS];
volatile char display_buffer[FRAMES_PER_SECOND * BUFFER_SIZE_SECONDS][NUM_OF_COLS];
const int buffer_length = FRAMES_PER_SECOND * BUFFER_SIZE_SECONDS;
volatile int buffer_head = 0; // points to front of buffer
volatile int buffer_tail = -1; // points to next available spot

volatile char current_row = 0;
volatile char current_col = 0;
volatile int current_frame_number = 0;
const int times_to_repeat_frame = REFRESH_RATE / FRAMES_PER_SECOND;

int LED_Array_State = 0; //TODO remove after functional test demo
int pitch_shift_state = NO_EFFECT;
int echo_state = NO_EFFECT;



uint16_t
	LedBuffer[4] = {};


volatile uint16_t
	LedRefreshCount = 0,
	LedDisplayedDigit = 0;

volatile int16_t
	EchoBuffer[16384];

volatile uint16_t
	EchoPointer = 0;

volatile uint8_t
	ClearEchoBuffer = TRUE;

//
// Data structure for timer configuration
//

TIM_HandleTypeDef
	Timer5_16Khz;

//
// Data structure for general purpose IO configuration
//

GPIO_InitTypeDef
	GpioInitStructure;

//
// Data structure for the D/A(DAC) Converter configuration
//

DAC_ChannelConfTypeDef
	DacInitStructure;

DAC_HandleTypeDef
	AudioDac;				// Structure for the audio digital to analog converter subsystem

//
// Data structures for the A/D Converter configuration
//

ADC_HandleTypeDef
	AudioAdc,
	ReferenceAdc;


volatile int
	ButtonCount = 0,
	ButtonState = RELEASED,
	Effect = NO_EFFECT;


//
// Buffering system variables
//

volatile int
	ADCPTR = 0;

volatile struct tBuffer
	Buffers[NUMBER_OF_BUFFERS];


volatile int
	WindowingState = 0,
	WindowingDone = FALSE;

//
// 4 times the size of the main buffer to compensate for addition of complex numbers and that we are processing
// 2 buffers at a time
//

float
	delayedBuf[SIZE*4],
	procBuf[SIZE*4];

int
	AD_Offset;

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
 * Col and height must be <= NUM_OF_COLS
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
void Fill_Buffer_With_Panning_Image(int _source_rows, int _source_cols, char source[_source_rows][_source_cols],
		int message_length, int direction) {
	char frame[NUM_OF_COLS];

//	char (*source)[_source_length] = _source;

	// Zero out the frame
	for (int i = 0; i < NUM_OF_COLS; i++) {
		frame[i] = 0;
	}

	// Start with blank frame
	Buffer_Pushback(frame);

	for (int current_index = 0; current_index < message_length; current_index++) {
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
	char sine_wave[7][NUM_OF_COLS] = {
			{ 0x0e, 0x31, 0x40, 0x80, 0x40, 0x31, 0x0e, 0x00 },
			{ 0x03, 0x8c, 0x50, 0x20, 0x50, 0x8c, 0x03, 0x00 },
			{ 0x80, 0x63, 0x14, 0x08, 0x14, 0x63, 0x80, 0x00 },
			{ 0xe0, 0x18, 0x05, 0x02, 0x05, 0x18, 0xe0, 0x00 },
			{ 0x38, 0xc6, 0x01, 0x00, 0x01, 0xc6, 0x38, 0x00 },
			{ 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00 },
			{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
	Fill_Buffer_With_Panning_Image(7, NUM_OF_COLS, sine_wave, 49, LEFT_TO_RIGHT);
}

void Display_Mode() {
	if (pitch_shift_state == ENABLE_PITCH_SHIFT && echo_state == ENABLE_ECHO) {
		// TODO draw on screen
	} else if (pitch_shift_state) {
		// TODO draw on screen
	} else if (echo_state) {
		// TODO draw on screen
	} else {
		// TODO draw on screen
	}

}

void Update_State()
{
	if (button_state_PC1) {
		previous_state_PC1 = 1;
	} else {
		if (previous_state_PC1) {
			//falling edge triggered
			Toggle_LED_Array();

			pitch_shift_state = !(pitch_shift_state);
		}
		previous_state_PC1 = 0;
	}

	if (button_state_PC4) {
		previous_state_PC4 = 1;
	} else {
		if (previous_state_PC4) {
			//falling edge triggered
			Toggle_LED_Array();

			echo_state = !(echo_state);
		}
		previous_state_PC4 = 0;
	}

	if (button_state_PB1) {
		previous_state_PB1 = 1;
	} else {
		if (previous_state_PB1) {
			//falling edge triggered
			Toggle_LED_Array();

			Display_Mode();
		}
		previous_state_PB1 = 0;
	}
}



/*
 * Name: UpdateLedDisplay
 *
 * Description: Update a specific seven segment digit on the 4 digit display
 *
 * Inputs:
 * 		None
 *
 * Output:
 * 		None
 *
 * Process:
 *
 * 		Display the selected digit and advance to the next digit
 * 		Refresh the display at a rate with no flicker
 *
 */

void UpdateLedDisplay( void )
{

}

/*
 * Name: TIM5_IRQHandler
 *
 * Description: Time 5 interrupt service routine call 16,000 times a second.
 * Inputs:
 * 		None
 *
 * Output:
 * 		None
 *
 * Process:
 *
 * 		Send audio signal to D/A converter
 *		Sample audio input
 *		Do echoing effect
 *		Handle windowing state update
 *		Update the LED display
 *		Detect button press and remove bounce
 *		Switch effects mode
 *
 */


void TIM5_IRQHandler(void)
{

	int16_t
		AudioSignal;

	TIMER_DEBUG_SIGNAL_ON;


//
// Check for timer update interrupt
//
	if ( __HAL_TIM_GET_FLAG( &Timer5_16Khz, TIM_IT_UPDATE ) != RESET )
	{


//
// Check for buffer full status
//
		if( 3 == Buffers[ANALOG_OUT_OFFSET].Full )
		{
//
// Output the Audio stream to the D/A converter
//
			DAC -> DHR12R1 = Buffers[ANALOG_OUT_OFFSET].Buf[Buffers[ANALOG_OUT_OFFSET].Head];

//
// Advanced the head pointer and check for end of buffer
//
			Buffers[ANALOG_OUT_OFFSET].Head++;		//increment head

			if( Buffers[ANALOG_OUT_OFFSET].Head >= SIZE)
			{
//
// Set the head pointer to the start of the buffer
// Reset the buffer full status
//
				Buffers[ANALOG_OUT_OFFSET].Head = 0;
				Buffers[ANALOG_OUT_OFFSET].Full = 0;
			}
		}


//
// Get values from adc and fill the buffer. when it is full reset the
// head pointer and set status to full then increment ALL buffers
// the & 0x03 is to loop the buffers back to 0 when they get to 4
// the << 3 is to increase the volume due to only being a 12b adc
//

//
// See if the buffer is not full
//
		if( 0  == Buffers[ADCPTR].Full)
		{

//
// Take a reading of the analog input pin and remove the offset signal
//
			AudioSignal = HAL_ADC_GetValue( &AudioAdc ) - AD_Offset;

//
// If enabled do the echo effect on the raw signal
//
			if ( ECHO == Effect )
			{

			}
			else
			{

//
// No echo effect. just store the data in the buffer
//

				Buffers[ADCPTR].Buf[Buffers[ADCPTR].Head] = AudioSignal;
			}

//
// Update the head pointer
//
			Buffers[ADCPTR].Head++;

//
// See if the buffer is full
//
			if( Buffers[ADCPTR].Head >= SIZE )
			{

//
// If this statement returns true then the FFT portion of the code has failed.
//
				if (( FALSE == WindowingDone ) && ( 0 != WindowingState ))
				{
//
// Fatal error
//
					while ( TRUE );
				}

//
// Advance to the next buffer
//
				Buffers[ADCPTR].Head = 0;		// Reset the head pointer
				Buffers[ADCPTR].Full = 1;		// Buffer Full = 1
				ADCPTR = ( ADCPTR + 1 ) & BUFFERS_MASK;

//
// changes the state for the overlapping windowing system
//
				switch( WindowingState )
				{
					case 0:
					{
						WindowingState = 1;
						WindowingDone = FALSE;
						break;
					}

					case 1:
					{
						WindowingState = 2;
						WindowingDone = FALSE;
						break;
					}

					case 2:
					{
						WindowingState = 3;
						WindowingDone = FALSE;
						break;
					}

					case 3:
					{
						WindowingState = 4;
						WindowingDone = FALSE;
						break;
					}

					case 4:
					{
						WindowingState = 3;
						WindowingDone = FALSE;
						break;
					}

					default:
					{

//
// Invalid state. Should not get here
//
						while ( TRUE );
						break;
					}
				}
			}
		}

//
// Start another conversion
//
		HAL_ADC_Start( &AudioAdc );

//
// Update the multiplexing LED display
//

		UpdateLedDisplay();

//
// Button push detection with debounce
//
		if ( HAL_GPIO_ReadPin( GPIOD, SWITCH_1 ) == GPIO_PIN_RESET )
		{

		}
		else
		{

		}



//
// Clear the timer update interrupt flag
//
		__HAL_TIM_CLEAR_FLAG( &Timer5_16Khz, TIM_IT_UPDATE );

	}
	TIMER_DEBUG_SIGNAL_OFF;
}


void PitchShift( float *Buffer )
{
	int
		PitchShift,
//
// Pitch Shift by 32 bins in the FFT table
// Each bin contains one complex number comprised of one real and one imaginary floating point number
//
	PitchOffset = 32 * 2;


	if (( PITCH_SHIFT_UP_8 == Effect ) || ( PITCH_SHIFT_DOWN_8 == Effect ))
	{
		PitchOffset = 8 * 2;
	}

	if (( PITCH_SHIFT_UP_16 == Effect ) || ( PITCH_SHIFT_DOWN_16 == Effect ))
	{
		PitchOffset = 16 * 2;
	}

//
// Shift frequencies up effect
//

	if (( PITCH_SHIFT_UP_8 == Effect ) || ( PITCH_SHIFT_UP_16 == Effect ) || ( PITCH_SHIFT_UP_32 == Effect ))
	{
//
// Do the lower half of FFT table
//
		PitchShift = 1022;
		while ( PitchShift >= PitchOffset  )
		{
			Buffer[PitchShift] = Buffer[PitchShift-PitchOffset];
			Buffer[PitchShift+1] = Buffer[(PitchShift+1)-PitchOffset];
			PitchShift -= 2;
		}

//
// Clear the duplicated portion of the table
//
		while ( PitchShift >= 0 )
		{
			Buffer[PitchShift] = 0;
			PitchShift--;
		}


//
// Do the upper half of the FFT table
//
		PitchShift = 1024;
		while ( PitchShift < ( 2048 - PitchOffset ))
		{
			Buffer[PitchShift] = Buffer[PitchShift+PitchOffset];
			Buffer[PitchShift+1] = Buffer[(PitchShift+1)+PitchOffset];
			PitchShift += 2;
		}
//
// Clear the duplicated portion of the table
//
		while ( PitchShift < 2048 )
		{
			Buffer[PitchShift] = 0;
			PitchShift++;
		}
	}


//
// Shift frequencies down effect
//

	if (( PITCH_SHIFT_DOWN_8 == Effect ) || ( PITCH_SHIFT_DOWN_16 == Effect ) || ( PITCH_SHIFT_DOWN_32 == Effect ))
	{


	}
}

int
main(int argc, char* argv[])
{
  // At this stage the system clock should have already been configured
  // at high speed.

	unsigned int loop;

	HAL_Init();// initializing HAL drivers

	__GPIOA_CLK_ENABLE(); // enabling clock for port A
	__GPIOB_CLK_ENABLE(); // enabling clock for port B
	__GPIOC_CLK_ENABLE(); // enabling clock for port C
	__GPIOD_CLK_ENABLE(); // enabling clock for port D
	__GPIOE_CLK_ENABLE(); // enabling clock for port E

	for( loop = 0; loop < NUMBER_OF_BUFFERS; loop++)
	{
		Buffers[loop].Head = 0;
		Buffers[loop].Full = 0;
		memset( (_PTR)&Buffers[loop].Buf, 0, sizeof( Buffers[loop].Buf ));
	}

	InitSystemPeripherals();

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

	int column = 0;
	int height = 3;

//
//	Take an Offset reading to remove the DC offset from the analog reading
//  Source PC2 ( ADC_CHANNEL_12 )
//
	AD_Offset = ConvertReference();

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

		Update_State();

		WindowingFFT();
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
 * increasing rows and columns
 */
void TIM4_IRQHandler() //Timer4 interrupt function
{
	__HAL_TIM_CLEAR_FLAG( &LEDDisplayTimer, TIM_IT_UPDATE ); //clear flag status

	if (current_row >= NUM_OF_COLS) {
		// at end of rows, need to advance to next column

		current_col++; //advance to next column
		current_row = 0; //restart row

		if (current_col >= NUM_OF_COLS) {
			// if the image has been displayed more than the number of times required
			// to achieve the desired REFRESH_RATE, pull the next image from buffer
			current_frame_number++;
			if (current_frame_number > times_to_repeat_frame) {
				Buffer_Pop(current_frame);

				current_frame_number = 0; //restart counting
			}
			current_col = 0; //restart column
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

void InitSystemPeripherals( void )
{

	ADC_ChannelConfTypeDef
		sConfig;

//
// Enable device clocks TIMER and GPIO port E
//
	__HAL_RCC_TIM5_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_DAC_CLK_ENABLE();


//
// Enable ADC3 and GPIO port C clocks
//
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_ADC2_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GpioInitStructure.Pin = SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G;
	GpioInitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_PULLUP;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOD, &GpioInitStructure );

	GpioInitStructure.Pin = LED_DIGIT_1 | LED_DIGIT_2 | LED_DIGIT_3 | LED_DIGIT_4;
	GpioInitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_PULLUP;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &GpioInitStructure );




//
// Enable GPIO Port E8 as an input ( used for button select options )
//

	GpioInitStructure.Pin = SWITCH_1 | SWITCH_2;
	GpioInitStructure.Mode = GPIO_MODE_INPUT;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_PULLUP;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOD, &GpioInitStructure );

//
// Enable GPIO Port E15 as an output ( used for timing with scope )
//

//	GpioInitStructure.Pin = GPIO_PIN_15 | GPIO_PIN_13;
//	GpioInitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
//	GpioInitStructure.Pull = GPIO_PULLUP;
//	GpioInitStructure.Alternate = 0;
//	HAL_GPIO_Init(GPIOD, &GpioInitStructure);

//
// Enable GPIO port A1 as an analog output
//
	GpioInitStructure.Pin = GPIO_PIN_4;
	GpioInitStructure.Mode = GPIO_MODE_ANALOG;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_NOPULL;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &GpioInitStructure );

	EnableAudioCodecPassThru();

//
// Configure DAC channel 1
//
	AudioDac.Instance = DAC;

	HAL_DAC_Init( &AudioDac );

	DacInitStructure.DAC_Trigger = DAC_TRIGGER_NONE;
	DacInitStructure.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	HAL_DAC_ConfigChannel( &AudioDac, &DacInitStructure ,DAC_CHANNEL_1 );


//
// Enable DAC channel 1
//
//
	HAL_DAC_Start( &AudioDac, DAC_CHANNEL_1 );

//
// Configure A/D converter channel 3
//

//
// Enable GPIO port C1, C2 and C5 as an analog input
//
	GpioInitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5;
	GpioInitStructure.Mode = GPIO_MODE_ANALOG;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_NOPULL;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOC, &GpioInitStructure );

//
// Configure audio A/D ( ADC2 ) for the audio stream
//

	AudioAdc.Instance = ADC2;
	AudioAdc.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AudioAdc.Init.Resolution = ADC_RESOLUTION_12B;
	AudioAdc.Init.ScanConvMode = DISABLE;
	AudioAdc.Init.ContinuousConvMode = DISABLE;
	AudioAdc.Init.DiscontinuousConvMode = DISABLE;
	AudioAdc.Init.NbrOfDiscConversion = 0;
	AudioAdc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AudioAdc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AudioAdc.Init.NbrOfConversion = 1;
	AudioAdc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AudioAdc.Init.DMAContinuousRequests = DISABLE;
	AudioAdc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	HAL_ADC_Init( &AudioAdc );

//
// Select PORTC pin 5 ( ADC_CHANNEL_15 ) for the audio stream
//
	sConfig.Channel = ADC_CHANNEL_15;
//	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	sConfig.Offset = 0;

	HAL_ADC_ConfigChannel(&AudioAdc, &sConfig);
	HAL_ADC_Start( &AudioAdc );

//
// Configure level shifting reference A/D (ADC1)
//
	ReferenceAdc.Instance = ADC1;
	ReferenceAdc.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	ReferenceAdc.Init.Resolution = ADC_RESOLUTION_12B;
	ReferenceAdc.Init.ScanConvMode = DISABLE;
	ReferenceAdc.Init.ContinuousConvMode = DISABLE;
	ReferenceAdc.Init.DiscontinuousConvMode = DISABLE;
	ReferenceAdc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	ReferenceAdc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	ReferenceAdc.Init.NbrOfConversion = 1;
	ReferenceAdc.Init.NbrOfDiscConversion = 0;
	ReferenceAdc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	ReferenceAdc.Init.DMAContinuousRequests = DISABLE;
	ReferenceAdc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	HAL_ADC_Init( &ReferenceAdc );
	HAL_ADC_Start( &ReferenceAdc );

//
// Initialize timer to 16Khz
//
	Timer5_16Khz.Instance = TIM5;
	Timer5_16Khz.Init.CounterMode = TIM_COUNTERMODE_UP;
	Timer5_16Khz.Init.Period = 250;
	Timer5_16Khz.Init.Prescaler = 20;
	Timer5_16Khz.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init( &Timer5_16Khz );

//
// Enable the timer interrupt
//
	HAL_NVIC_SetPriority( TIM5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ( TIM5_IRQn );

	__HAL_TIM_ENABLE_IT( &Timer5_16Khz, TIM_IT_UPDATE );

//
// Enable timer 5 update interrupt
//
	__HAL_TIM_ENABLE( &Timer5_16Khz );


}

int ConvertAudio(void)
{
	int
		ADCResult;

//
// Start a conversion
//
	HAL_ADC_Start( &AudioAdc );

//
// Wait for end of conversion
//
    HAL_ADC_PollForConversion( &AudioAdc, HAL_MAX_DELAY );

//
// Get the 12 bit result
//
    ADCResult = HAL_ADC_GetValue( &AudioAdc );

    return(ADCResult);
}


int ConvertReference(void)
{
	int
		ADCResult;

	ADC_ChannelConfTypeDef sConfig;

//
// Select the channel to convert and start the conversion
//
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	sConfig.Offset = 0;

	HAL_ADC_ConfigChannel(&ReferenceAdc, &sConfig);

//
// Start a conversion
//
	HAL_ADC_Start( &ReferenceAdc );

//
// Wait for end of conversion
//
    HAL_ADC_PollForConversion( &ReferenceAdc, HAL_MAX_DELAY );

//
// Get the 12 bit result
//
    ADCResult = HAL_ADC_GetValue( &ReferenceAdc );

    return(ADCResult);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
