/*
 * main.h
 *
 *  Created on: Mar 21, 2018
 *      Author: brent
 */

#ifndef MAIN_H_
#define MAIN_H_

#define TRUE				( 1 == 1 )
#define FALSE				( 1 == 0 )

#define PUSHED				0
#define	RELEASED			1

#define	NO_EFFECT			0
#define ECHO				1
#define	PITCH_SHIFT_UP_8	2
#define PITCH_SHIFT_UP_16	3
#define PITCH_SHIFT_UP_32	4
#define PITCH_SHIFT_DOWN_8	5
#define PITCH_SHIFT_DOWN_16	6
#define PITCH_SHIFT_DOWN_32	7

//
// Buffering constants
//

#define PROCOFFSET 			((ADCPTR-1) & 0x03)
#define PROCOUTOFFSET		((ADCPTR-2) & 0x03)
#define ANALOG_OUT_OFFSET 	((ADCPTR-3) & 0x03)
#define SIZE 				(sizeof( Buffers[0].Buf )/sizeof( Buffers[0].Buf[0]))

//
// Debug signals for timing specific functions with an oscilloscope
//

//#define TIMER_DEBUG_SIGNAL_ON	( GPIOE->BSRR = GPIO_PIN_15 )
//#define TIMER_DEBUG_SIGNAL_OFF	( GPIOE->BSRR = GPIO_PIN_15 << 16 )

//#define FFT_DEBUG_SIGNAL_ON		( GPIOE->BSRR = GPIO_PIN_13 )
//#define FFT_DEBUG_SIGNAL_OFF	( GPIOE->BSRR = GPIO_PIN_13 << 16 )

#define TIMER_DEBUG_SIGNAL_ON
#define TIMER_DEBUG_SIGNAL_OFF

#define FFT_DEBUG_SIGNAL_ON
#define FFT_DEBUG_SIGNAL_OFF


//
// Must be binary multiple (4,8,16... )
//

#define NUMBER_OF_BUFFERS 	(4)
#define	BUFFERS_MASK		(NUMBER_OF_BUFFERS - 1)

struct tBuffer
{
	volatile int16_t
		Buf[512];

	volatile unsigned int
		Head,
		Full;
};


void
	PitchShift( float *Buffer ),
	UpdateLedDisplay( void ),
	InitSystemPeripherals( void );

int
	ConvertReference(void);

#endif /* MAIN_H_ */
