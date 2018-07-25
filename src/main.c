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
#define REFRESH_RATE 			(5000)//5000 will reduce the high pitched noise // this will be multiplied by 64 since there are 64 LEDs
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
#define BUTTON_1				(GPIO_PIN_11)
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
 * TIM2 - Generating frequency bars to display on LED array
 * TIM3 - polling of all button inputs, and debouncing
 * TIM4 - LED board drawing
 * TIM5 - FFT on input signal
 */
#define TIM2_PRIORITY			(7)
#define TIM3_PRIORITY 			(10)
#define TIM4_PRIORITY			(5)
#define TIM5_PRIORITY	 		(0)
#define NUM_OF_COLS				(8)

#define NO_EFFECT				(0)
#define ENABLE_ECHO				(1)
#define ENABLE_PITCH_SHIFT		(1)

#define ECHO_BUFFER_SIZE		(16384)
#define ECHO_DAMPING			(0.5)

volatile char previous_button_reading_PA0 = 0;
volatile char button_state_PA0 = 0;
volatile char previous_button_reading_PB11 = 0;
volatile char button_state_PB11 = 0;
volatile char previous_button_reading_PC4 = 0;
volatile char button_state_PC4 = 0;
volatile char previous_button_reading_PB1 = 0;
volatile char button_state_PB1 = 0;

char previous_state_PB11 = 0;
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

int LED_Array_State = 0;
int pitch_shift_state = NO_EFFECT;
int echo_state = NO_EFFECT;

volatile int pitch_shift_offset = 0;

volatile int16_t
	EchoBuffer[ECHO_BUFFER_SIZE];

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
	ReferenceAdc,
	PitchShiftOffsetAdc;


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
TIM_HandleTypeDef	FrequencySpectrumGeneratorTimer;
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
	// reduce to (REFRESH_RATE * NUMBER_OF_LEDS) frequency
	LEDDisplayTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
	LEDDisplayTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init( &LEDDisplayTimer );

	HAL_NVIC_SetPriority( TIM4_IRQn, TIM4_PRIORITY, TIM4_PRIORITY);
	//set priority for the interrupt. Value 0 corresponds to highest priority
	HAL_NVIC_EnableIRQ( TIM4_IRQn );//Enable interrupt function request of Timer3

	__HAL_TIM_ENABLE_IT( &LEDDisplayTimer, TIM_IT_UPDATE );// Enable timer interrupt flag to be set when timer count is reached
	__HAL_TIM_ENABLE( &LEDDisplayTimer );//Enable timer to start


	__HAL_RCC_TIM2_CLK_ENABLE();
	FrequencySpectrumGeneratorTimer.Instance = TIM2;
	prescaler = 140;
	FrequencySpectrumGeneratorTimer.Init.Period = prescaler - 1; // reduce to 600 kHz
	// reduce to (FRAMES_PER_SECOND * 2) frequency
	FrequencySpectrumGeneratorTimer.Init.Prescaler = 84000000 / prescaler / (FRAMES_PER_SECOND*2) - 1;
	FrequencySpectrumGeneratorTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
	FrequencySpectrumGeneratorTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init( &FrequencySpectrumGeneratorTimer );

	HAL_NVIC_SetPriority( TIM2_IRQn, TIM2_PRIORITY, TIM2_PRIORITY);
	//set priority for the interrupt. Value 0 corresponds to highest priority
	HAL_NVIC_EnableIRQ( TIM2_IRQn );//Enable interrupt function request of Timer2

	__HAL_TIM_ENABLE_IT( &FrequencySpectrumGeneratorTimer, TIM_IT_UPDATE );// Enable timer interrupt flag to be set when timer count is reached
	__HAL_TIM_ENABLE( &FrequencySpectrumGeneratorTimer );//Enable timer to start
}

void Configure_Ports()
{
	Init_GPIO_Port_Default_Speed_Pull(GPIO_PIN_12, GPIO_MODE_OUTPUT_PP, 'D');
	Init_GPIO_Port_Default_Speed_Pull(GPIO_PIN_0, GPIO_MODE_INPUT, 'A');

	Init_GPIO_Port_Default_Speed_Pull(BUTTON_1, GPIO_MODE_INPUT, 'B');
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
inline void Buffer_Clear()
{
	buffer_head = 0;
	buffer_tail = -1; // flag that the buffer is empty
}

/**
 * Indicates if buffer is empty
 */
inline int Buffer_Is_Empty()
{
	return buffer_tail == -1;
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
 * Initializes buffer with all 0's. Returns 1 when successful
 */
int Buffer_Init()
{
	char all_zeros[NUM_OF_COLS];
	for (int i = 0; i < NUM_OF_COLS; i++) {
		all_zeros[i] = 0;
	}

	for (int i = 0; i < buffer_length; i++) {
		memcpy(display_buffer[i], all_zeros, NUM_OF_COLS);
	}

	Buffer_Clear();

	return 1;
}

/**
 * turns on all LEDs for testing
 */
void Display_All_On() {
	for (int i = 0; i < NUM_OF_COLS; i++) {
		current_frame[i] = 0xFF;
	}
}

/**
 * turns off all LEDs for testing
 */
void Display_All_Off() {
	for (int i = 0; i < NUM_OF_COLS; i++) {
		current_frame[i] = 0;
	}
}

/**
 * If LED_Array is on, toggle off. If off, toggle on.
 */
void Toggle_Display_State() {
	if (LED_Array_State) {
		Display_All_Off();
		LED_Array_State = 0;
	} else {
		Display_All_On();
		LED_Array_State = 1;
	}
}

/**
 * Fill buffer with frames with one LED at a time, cycling through all LEDs
 * Precondition: buffer is at least framesToRepeat*64 long
 */
void Display_Scan_Across_LEDs() {
	const int framesToRepeat = 3;
	char frame[NUM_OF_COLS];

	for (int i = 0; i < NUM_OF_COLS; i++)
	{
		frame[i] = 0;
	}
	for (int currentLED = 0; currentLED < NUMBER_OF_LEDS; currentLED++)
	{
		int row = currentLED/NUM_OF_COLS;
		int col = currentLED%NUM_OF_COLS;

		frame[row] = 1 << col;

		for (int i = 0; i < framesToRepeat; i++)
		{
			Buffer_Pushback(frame);
		}

		frame[row] = 0;
	}

	for (int i = 0; i < NUM_OF_COLS; i++) {
		frame[i] = 0;
	}

	Buffer_Pushback(frame);
}

/**
 * Creates a bar of height 'height' in the specified column 'col'
 * Col must be <= NUM_OF_COLS
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
			trace_printf("Not implemented exception. Invalid direction.");
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

in hex:

0x0e	0x03	0x80	0xe0	0x38	0x00
0x31	0x8c	0x63	0x18	0xc6	0x00
0x40	0x50	0x14	0x05	0x01	0x00
0x80	0x20	0x08	0x02	0x00	0x80
0x40	0x50	0x14	0x05	0x01	0x00
0x31	0x8c	0x63	0x18	0xc6	0x00
0x0e	0x03	0x80	0xe0	0x38	0x00
0x00	0x00	0x00	0x00	0x00	0x00
 */
void Display_Sine_Wave() {
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

/**
   1     1       11          11       1     1
    1   1       1111        1111       1   1
   1111111     111111      111111     1111111
  11 111 11   11 11 11    11 11 11   11 111 11
 11111111111  11111111    11111111  11111111111
 1 1111111 1    1  1        1  1    1 1111111 1
 1 1     1 1   1 11 1      1 11 1   1 1     1 1
    11 11     1 1  1 1    1 1  1 1     11 11

in hex:

10	40	60	06	02	08
08	80	F0	0F	01	10
1F	C1	F8	1F	83	F8
37	63	6C	36	C6	EC
7F	F3	FC	3F	CF	FE
5F	D0	90	09	0B	FA
50	51	68	16	8A	0A
0D	82	94	29	41	B0
 */
void Invade_Space()
{
	char space_invaders[6][NUM_OF_COLS] = {
			{ 0x10, 0x08, 0x1F, 0x37, 0x7F, 0x5F, 0x50, 0x0D },
			{ 0x40, 0x80, 0xC1, 0x63, 0xF3, 0xD0, 0x51, 0x82 },
			{ 0x60, 0xF0, 0xF8, 0x6C, 0xFC, 0x90, 0x68, 0x94 },
			{ 0x06, 0x0F, 0x1F, 0x36, 0x3F, 0x09, 0x16, 0x29 },
			{ 0x02, 0x01, 0x83, 0xC6, 0xCF, 0x0B, 0x8A, 0x41 },
			{ 0x08, 0x10, 0xF8, 0xEC, 0xFE, 0xFA, 0x0A, 0xB0 }
	};
	Fill_Buffer_With_Panning_Image(6, NUM_OF_COLS, space_invaders, 55, LEFT_TO_RIGHT);
}

/**

   1111 111 11111 1111 1   1         1         1
   1  1  1    1   1    1   1  1     1 1       1
   1111  1    1   1    11111 1 1   1   1     1
   1     1    1   1    1   1    1 1     1   1
   1    111   1   1111 1   1     1       1 1
                                          1
111111111111111111111111111111111111111111111111

in hex:

00	00	00	00	00	00
1E	EF	BD	10	04	01
12	42	21	12	0A	02
1E	42	21	F5	11	04
10	42	21	10	A0	88
10	E2	3D	10	40	50
00	00	00	00	00	20
FF	FF	FF	FF	FF	FF
 */
void Display_Pitch_Shift()
{
	char pitch_shift_message[6][NUM_OF_COLS] = {
			{ 0x00, 0x1E, 0x12, 0x1E, 0x10, 0x10, 0x00, 0xFF },
			{ 0x00, 0xEF, 0x42, 0x42, 0x42, 0xE2, 0x00, 0xFF },
			{ 0x00, 0xBD, 0x21, 0x21, 0x21, 0x3D, 0x00, 0xFF },
			{ 0x00, 0x10, 0x12, 0xF5, 0x10, 0x10, 0x00, 0xFF },
			{ 0x00, 0x04, 0x0A, 0x11, 0xA0, 0x40, 0x00, 0xFF },
			{ 0x00, 0x01, 0x02, 0x04, 0x88, 0x50, 0x20, 0xFF }
	};
	Fill_Buffer_With_Panning_Image(6, NUM_OF_COLS, pitch_shift_message, 56, LEFT_TO_RIGHT);
}

/**
                                             1
   1111 1111 1   1 11111          1         1
   1    1    1   1 1   1   1     1 1       1
   1111 1    11111 1   1  1 1   1   1     1
   1    1    1   1 1   1     1 1     1   1
   1111 1111 1   1 11111      1       1 1
                                       1
111111111111111111111111111111111111111111111111

in hex:

00	00	00	00	00	04
1E	F4	5F	00	20	08
10	84	51	10	50	10
1E	87	D1	28	88	20
10	84	51	05	04	40
1E	F4	5F	02	02	80
00	00	00	00	01	00
FF	FF	FF	FF	FF	FF
 */
void Display_Echo()
{
	char echo_message[6][NUM_OF_COLS] = {
			{ 0x00, 0x1E, 0x10, 0x1E, 0x10, 0x1E, 0x00, 0xFF },
			{ 0x00, 0xF4, 0x84, 0x87, 0x84, 0xF4, 0x00, 0xFF },
			{ 0x00, 0x5F, 0x51, 0xD1, 0x51, 0x5F, 0x00, 0xFF },
			{ 0x00, 0x00, 0x10, 0x28, 0x05, 0x02, 0x00, 0xFF },
			{ 0x00, 0x20, 0x50, 0x88, 0x04, 0x02, 0x01, 0xFF },
			{ 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0xFF }
	};
	Fill_Buffer_With_Panning_Image(6, NUM_OF_COLS, echo_message, 56, LEFT_TO_RIGHT);
}

/**

  1111 111 11111 1111  1   1111 1111 1   1 11111
  1  1  1    1   1    1 1  1    1    1   1 1   1
  1111  1    1   1     1   1111 1    11111 1   1
  1     1    1   1    1 1  1    1    1   1 1   1
  1    111   1   1111  1   1111 1111 1   1 11111

111111111111111111111111111111111111111111111111

in hex:

00	00	00	00	00	00
3D	DF	79	1E	F4	5F
24	84	42	90	84	51
3C	84	41	1E	87	D1
20	84	42	90	84	51
21	C4	79	1E	F4	5F
00	00	00	00	00	00
FF	FF	FF	FF	FF	FF
 */
void Display_Pitch_Echo()
{
	char pitch_echo_message[6][NUM_OF_COLS] = {
			{ 0x00, 0x3D, 0x24, 0x3C, 0x20, 0x21, 0x00, 0xFF },
			{ 0x00, 0xDF, 0x84, 0x84, 0x84, 0xC4, 0x00, 0xFF },
			{ 0x00, 0x79, 0x42, 0x41, 0x42, 0x79, 0x00, 0xFF },
			{ 0x00, 0x1E, 0x90, 0x1E, 0x90, 0x1E, 0x00, 0xFF },
			{ 0x00, 0xF4, 0x84, 0x87, 0x84, 0xF4, 0x00, 0xFF },
			{ 0x00, 0x5F, 0x51, 0xD1, 0x51, 0x5F, 0x00, 0xFF }
	};
	Fill_Buffer_With_Panning_Image(6, NUM_OF_COLS, pitch_echo_message, 56, LEFT_TO_RIGHT);
}

void Display_Debugging() {
	Display_Sine_Wave();
}

void Display_Mode() {
	Buffer_Clear();
	if (pitch_shift_state == ENABLE_PITCH_SHIFT && echo_state == ENABLE_ECHO) {
		Display_Pitch_Echo();
	} else if (pitch_shift_state) {
		Display_Pitch_Shift();
	} else if (echo_state) {
		Display_Echo();
	} else {
		Invade_Space();
	}

	// TODO flag when you can write to buffer again
}

/**
 * Clears out echo buffer to AD_Offset so when it's subtracted it becomes 0
 */
inline void Echo_Buffer_Clear()
{
	for (int i = 0; i < ECHO_BUFFER_SIZE; i++)
	{
		EchoBuffer[i] = AD_Offset;
	}

	EchoPointer = 0;
}

/**
 * Appends value to end of echo buffer
 */
inline void Echo_Buffer_Pushback(int16_t value)
{
	EchoBuffer[EchoPointer] = value;

	// increment and wrap around
	EchoPointer++;
	EchoPointer %= ECHO_BUFFER_SIZE;
}

/**
 * Returns value of EchoBuffer[EchoPointer - 1], wrapping the index around to ECHO_BUFFER_SIZE
 */
inline int16_t Echo_Buffer_Pop()
{
	int index = EchoPointer + 1;
	index %= ECHO_BUFFER_SIZE;

	return EchoBuffer[index];
}

void Update_State()
{
	if (button_state_PB11) {
		previous_state_PB11 = 1;
	} else {
		if (previous_state_PB11) {
			//falling edge triggered
			pitch_shift_state = !(pitch_shift_state);

			Display_Mode();
		}
		previous_state_PB11 = 0;
	}

	if (button_state_PC4) {
		previous_state_PC4 = 1;
	} else {
		if (previous_state_PC4) {
			//falling edge triggered
			echo_state = !(echo_state);

			Display_Mode();

			if (echo_state == ENABLE_ECHO)
			{
				// Zero out buffer
				Echo_Buffer_Clear();
			}
		}
		previous_state_PC4 = 0;
	}

	if (button_state_PB1) {
		previous_state_PB1 = 1;
	} else {
		if (previous_state_PB1) {
			//falling edge triggered

			Display_Mode();
		}
		previous_state_PB1 = 0;
	}
}


/**
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
			if ( echo_state == ENABLE_ECHO )
			{
				Echo_Buffer_Pushback(AudioSignal); // pushback current AudioSignal

				// pop from one index ahead of current EchoPointer
				// (which was the AudioSignal value one second ago)
				Buffers[ADCPTR].Buf[Buffers[ADCPTR].Head] = AudioSignal + (Echo_Buffer_Pop() - AD_Offset)* ECHO_DAMPING;
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
// Clear the timer update interrupt flag
//
		__HAL_TIM_CLEAR_FLAG( &Timer5_16Khz, TIM_IT_UPDATE );

	}
	TIMER_DEBUG_SIGNAL_OFF;
}


/**
 * A FFT table utility function that shifts the buffer elements so buffer[i] = buffer[i-PitchOffset]
 * Starts at start_index, which is the highest index and stops before end_index
 * Clears all elements for last PitchOffset number of elements with 0's.
 */
inline void ShiftBufferElementsUp( float *Buffer, int start_index, int end_index, int PitchOffset)
{
    int PitchShift;

    // Start at highest index, start_index, and grab elements from smaller indices,
    // stopping before writing past end_index
    PitchShift = start_index;
    while ( PitchShift >= end_index + PitchOffset )
    {
        Buffer[PitchShift] = Buffer[PitchShift-PitchOffset];
        Buffer[PitchShift+1] = Buffer[(PitchShift+1)-PitchOffset];
        PitchShift -= 2;
    }

    // Clear the remaining (duplicated) portion of the table
    while ( PitchShift >= end_index )
    {
        Buffer[PitchShift] = 0;
        PitchShift--;
    }
}

/**
 * A FFT table utility function that shifts the buffer elements so buffer[i] = buffer[i+PitchOffset]
 * Starts at start_index, which is the lowest index and stops before end_index
 * Clears all elements for last PitchOffset number of elements with 0's.
 */
inline void ShiftBufferElementsDown ( float *Buffer, int start_index, int end_index, int PitchOffset)
{
    int PitchShift;

    // Start at lowest index, start_index, and grab elements from higher indices,
    // stopping before writing past end_index
    PitchShift = start_index;
    while ( PitchShift < ( end_index - PitchOffset ))
    {
        Buffer[PitchShift] = Buffer[PitchShift+PitchOffset];
        Buffer[PitchShift+1] = Buffer[(PitchShift+1)+PitchOffset];
        PitchShift += 2;
    }

    // Clear the remaining (duplicated) portion of the table
    while ( PitchShift < end_index )
    {
        Buffer[PitchShift] = 0;
        PitchShift++;
    }
}

void PitchShift( float *Buffer )
{
//
// Pitch Shift by 32 bins in the FFT table
// Each bin contains one complex number comprised of one real and one imaginary floating point number
//
	int PitchOffset = (pitch_shift_offset >= 0)? pitch_shift_offset * 2: pitch_shift_offset * -2;
	//between -32 and 32, take absolute value

    // The FFT table is 2048 in length
    const int FFT_table_size = 2048;

    // The lower half, the indices [0, 1024), corresponds to positive frequencies
    // The upper half, the indices [1024, 2048), corresponds to negative frequencies

    // Shift frequencies up effect
	if (pitch_shift_offset > 0)
	{
        // Shift the lower half of the FFT table up
        ShiftBufferElementsUp(Buffer, (FFT_table_size / 2 - 2), 0, PitchOffset);

        // Shift the upper half of the FFT table down
        ShiftBufferElementsDown(Buffer, FFT_table_size / 2, FFT_table_size, PitchOffset);
	}

    // Shift frequencies down effect
	if (pitch_shift_offset < 0)
	{
        // Shift the lower half of the FFT table down
        ShiftBufferElementsDown(Buffer, 0, FFT_table_size / 2, PitchOffset);

        // Shift the upper half of the FFT table up
        ShiftBufferElementsUp(Buffer, (FFT_table_size - 2), FFT_table_size / 2, PitchOffset);
	}
}

int ConvertPitchShiftOffset(void)
{
	int
		ADCResult;

//
// Start a conversion
//
	HAL_ADC_Start( &PitchShiftOffsetAdc );

//
// Wait for end of conversion
//
    HAL_ADC_PollForConversion( &PitchShiftOffsetAdc, HAL_MAX_DELAY );

//
// Get the 8 bit result
//
    ADCResult = HAL_ADC_GetValue( &PitchShiftOffsetAdc );

    return(ADCResult);
}

int
main(int argc, char* argv[])
{
	// TODO make a header file with function declarations
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

	Display_All_Off();

//	Display_Scan_Across_LEDs();

	Display_Sine_Wave();

	// Start timers LAST to ensure that no interrupts based on timers will
	// trigger before initialization of board is complete
	ConfigureTimers();

	HAL_GPIO_WritePin( GPIOD, GPIO_PIN_12, 1); // Signal initialization is complete on on-board LED

	int previous_state_PA0 = 0;

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
				Buffer_Clear();
				Display_Debugging();
			}
			previous_state_PA0 = 0;
		}

		Update_State();

		WindowingFFT();
	}
	//TODO add code upon stop execution of program
}

void TIM3_IRQHandler() //Timer3 interrupt function
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
	if (HAL_GPIO_ReadPin(GPIOB, BUTTON_1)) {
			// button is pressed.
		if (previous_button_reading_PB11) {
			// if this is consistent with previous reading, set state to 1
			button_state_PB11 = 1;
		}
		//update previous reading to current reading
		previous_button_reading_PB11 = 1;
	} else {
		// button is not pressed
		if (!previous_button_reading_PB11) {
			// if this is consistent with previous reading, set state to 0
			button_state_PB11 = 0;
		}
		//update previous reading to current reading
		previous_button_reading_PB11 = 0;
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

	// Check potentiometer of pitch_shift_offset if ENABLE_PITCH_SHIFT
	if (pitch_shift_state == ENABLE_PITCH_SHIFT) {
		int pitch_shift_offset_raw = ConvertPitchShiftOffset(); // 0 to 255

		pitch_shift_offset_raw = 64.0/255 * pitch_shift_offset_raw; // reduce range to 0 to 64
		pitch_shift_offset -= 32; // shift range to -32 to 32;
	} else {
		pitch_shift_offset = 0;
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

	char enable_row = current_frame[current_col] & 1 << current_row;
	// for each case, turn off previous row, turn on current row
	switch(current_row) {
		case 0:
			if (enable_row) { Write_Row_0(GPIO_PIN_SET); }
			Write_Row_7(GPIO_PIN_RESET);
			break;
		case 1:
			if (enable_row) { Write_Row_1(GPIO_PIN_SET); }
			Write_Row_0(GPIO_PIN_RESET);
			break;
		case 2:
			if (enable_row) { Write_Row_2(GPIO_PIN_SET); }
			Write_Row_1(GPIO_PIN_RESET);
			break;
		case 3:
			if (enable_row) { Write_Row_3(GPIO_PIN_SET); }
			Write_Row_2(GPIO_PIN_RESET);
			break;
		case 4:
			if (enable_row) { Write_Row_4(GPIO_PIN_SET); }
			Write_Row_3(GPIO_PIN_RESET);
			break;
		case 5:
			if (enable_row) { Write_Row_5(GPIO_PIN_SET); }
			Write_Row_4(GPIO_PIN_RESET);
			break;
		case 6:
			if (enable_row) { Write_Row_6(GPIO_PIN_SET); }
			Write_Row_5(GPIO_PIN_RESET);
			break;
		case 7:
			if (enable_row) { Write_Row_7(GPIO_PIN_SET); }
			Write_Row_6(GPIO_PIN_RESET);
			break;
		default:
			//Should never enter this
			trace_printf("Invalid state in switch(current_row)");
			break;
	}

	current_row++; //move to next row
}

void TIM2_IRQHandler() //Timer2 interrupt function
{
	__HAL_TIM_CLEAR_FLAG( &FrequencySpectrumGeneratorTimer, TIM_IT_UPDATE );//clear flag status

	// This interrupt service routine is timer driven at 50 Hz

	// The FFT table is 2048 in length
	const int FFT_table_size = 2048;

	// Look at lower half of FFT table where higher indices correspond to higher frequencies
	// These indices are [0, 1023].
	// Each complex number takes up two elements in the float array. (One for real, one for imaginary)
	// We break 1024 elements, or 512 bins, into 8 groups, one for each column of the LED matrix
	// This means we investigate 512 / 8, or 64 bins, for each group
	// TODO  Since octaves are multiplicative, ideally we investigate in powers of 2
	// For each bin, we take max(real, imaginary) and add to the float. We are avoiding taking
	// the magnitude using sqrt(real^2 + imaginary^2) since sqrt is processor intensive and
	// we don't need the accuracy. max(real,imaginary) is an adequate approximation since
	// the max will dominate the square root anyway

	// group_sum / group_num_bins gives the average sort-of-magnitude in that group
	// average sort-of-magnitude / normalizing_constant brings the magnitude to a normalized_range
	// normalized_range * 8 gives the number of LEDs to light up in the column

	float group_sum = 0; // holds the accumulated sum for each group, used to average
	const int group_num_bins = 64; // 64 bins per group, which is converted to a column on the display

	const float normalizing_constant = 100; // divide the average by this, to normalize and convert to bars

	int height_of_bar = 0; // the height to make the frequency bar
	char frequency_spectrum_frame[NUM_OF_COLS]; // to hold the frame being generated

	int bins_analyzed = 0; // the number of bins already analyzed in the group
	int current_col = 0; // tracking which column we are in

	if (Buffer_Is_Empty())
	{
		// Only generate the frequency spectrum frame if nothing is being displayed on LED display

		for (int i = 0; i < FFT_table_size / 2; i += 2)
		{
			// Add max(procBuf[i], procBuf[i+1]) to group_sum
			group_sum += (procBuf[i] > procBuf[i+1])? procBuf[i]: procBuf[i+1];
			bins_analyzed++;

			// if we have already analyzed the group, create a bar
			if (bins_analyzed >= group_num_bins * 2)
			{
				group_sum /= group_num_bins; //average magnitude

				group_sum /= normalizing_constant; // normalize

				height_of_bar = (int) (group_sum * 8); // calculate hight of bar

				Create_Column_With_Height(frequency_spectrum_frame, current_col, height_of_bar);
				// reset temporary variables
				bins_analyzed = 0;
				group_sum = 0;
				height_of_bar = 0;

				// increment to next column
				current_col++;
			}
		}
		Buffer_Pushback(frequency_spectrum_frame); // add it to buffer
	}
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
	__HAL_RCC_ADC3_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();


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
	GpioInitStructure.Pin = GPIO_PIN_2 | GPIO_PIN_5; //GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5;
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

	GpioInitStructure.Pin = GPIO_PIN_1;
	GpioInitStructure.Mode = GPIO_MODE_ANALOG;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_NOPULL;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &GpioInitStructure );

//
// Configure pitch shift offset A/D (ADC3)
//
	PitchShiftOffsetAdc.Instance = ADC3;
	PitchShiftOffsetAdc.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	PitchShiftOffsetAdc.Init.Resolution = ADC_RESOLUTION_8B;
	PitchShiftOffsetAdc.Init.ScanConvMode = DISABLE;
	PitchShiftOffsetAdc.Init.ContinuousConvMode = DISABLE;
	PitchShiftOffsetAdc.Init.DiscontinuousConvMode = DISABLE;
	PitchShiftOffsetAdc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	PitchShiftOffsetAdc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	PitchShiftOffsetAdc.Init.NbrOfConversion = 1;
	PitchShiftOffsetAdc.Init.NbrOfDiscConversion = 0;
	PitchShiftOffsetAdc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	PitchShiftOffsetAdc.Init.DMAContinuousRequests = DISABLE;
	PitchShiftOffsetAdc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	HAL_ADC_Init( &PitchShiftOffsetAdc );
	HAL_ADC_Start( &PitchShiftOffsetAdc );

//
// Select PORTA pin 1 ( ADC_CHANNEL_1 ) for the pitch offset
//
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	sConfig.Offset = 0;

	HAL_ADC_ConfigChannel(&PitchShiftOffsetAdc, &sConfig);
	HAL_ADC_Start( &PitchShiftOffsetAdc );

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
