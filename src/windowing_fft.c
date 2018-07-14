#include "stm32f4xx_hal.h"
#include "main.h"
#include "hamming.h"
#include "arm_math.h"
#include "arm_const_structs.h"


extern volatile int
	ADCPTR;

extern volatile struct tBuffer
	Buffers[NUMBER_OF_BUFFERS];

extern volatile int
	WindowingState,
	WindowingDone;

extern float
	delayedBuf[SIZE*4],
	procBuf[SIZE*4];

extern int
	AD_Offset;



void WindowingFFT(void)
{

	if(1 == Buffers[PROCOFFSET].Full)
	{


		unsigned int
			loop = 0;

		int
			counts = 0;


//
// States 1 and 2 are for starting the overlap system, they only run once each.
// They wait until 2 512 size buffers are full before doing the first fft and transmit.
//
		if (( 1 == WindowingState ) && ( FALSE == WindowingDone ))
		{
//
// From 0 -> 512 placing the rising edge multiplied by the data into first 1024 of processbuf
// every second value is 0 for complex.
// processbuf and delay buf are size 2048 so they can fit 2 of the 512 data buffers as well as
// the complex value of 0 for every 2nd value of the array.  The complex value is needed for the
// fft.
//
			for( loop = 0; loop < SIZE; loop++)		//SIZE = 512
			{
//
// multiply the first data with the rising edge of the hamming window
// and put it into the first half of procbuf
// data looks like  [0] = data , [1] = 0 , [2] = data in the array
//
				procBuf[counts] = Buffers[PROCOFFSET].Buf[loop] * Hamming[loop]; //hamming full array size is 1024
				procBuf[counts+1] = 0;
				counts += 2;
			}
//
// makes sure this state does not run twice during the down time between state changes.
//
			WindowingDone = TRUE;
		}

		else if(( 2 == WindowingState ) && ( FALSE == WindowingDone ))
		{
//
// fills the second half of procbuf with a falling hamming window edge and the first half of delayedbuf
// with a rising hamming window edge
//
			counts = SIZE *2; //1024
			for( loop=0; loop < SIZE; loop++)
			{
				procBuf[counts] = Buffers[PROCOFFSET].Buf[loop] * Hamming[loop + SIZE];
				procBuf[counts+1] = 0;
				delayedBuf[counts-1024] = Buffers[PROCOFFSET].Buf[loop] * Hamming[loop];
				delayedBuf[counts-1023] = 0;
				counts+=2;
			}

//
// FFT
//
			arm_cfft_f32( &arm_cfft_sR_f32_len1024, procBuf, 0, 1 );

//
// process the frequency spectrum
//
			PitchShift( procBuf );

//
// inverse FFT
//
			arm_cfft_f32( &arm_cfft_sR_f32_len1024, procBuf, 1, 1 );

//
// Takes first half of processed buffer (very first rising edge)
// and puts it in a 512 outputbuffer to be transmitted
//
			counts = 0;
			for( loop=0;loop < 1024; loop+=2)
			{
				Buffers[PROCOUTOFFSET].Buf[counts] = procBuf[loop] +AD_Offset;
				counts++;
			}
//
// Set a flags to allow tx and change checkIfDone to ensure state doesn't repeat
//
			Buffers[PROCOUTOFFSET].Full = 3;
			Buffers[PROCOFFSET].Full = 2;
			WindowingDone = TRUE;
		}

//
// States 3 and 4 rotate for the rest of the program.  they each do a different half of the delay buffer and the
// process buffer.
//
		else if (( 3 == WindowingState ) && ( FALSE == WindowingDone ))
		{
			FFT_DEBUG_SIGNAL_ON;
//
// Fills the first half of procbuf and second half of delayedbuf
//
			counts = SIZE*2; 	//1024
			for(loop=0; loop < SIZE; loop++)
			{
				delayedBuf[counts] = Buffers[PROCOFFSET].Buf[loop] * Hamming[loop + SIZE];
				delayedBuf[counts+1] = 0;
				procBuf[counts-1024] = Buffers[PROCOFFSET].Buf[loop] * Hamming[loop];
				procBuf[counts-1023] = 0;
				counts+=2;
			}

//
// FFT
//
			arm_cfft_f32( &arm_cfft_sR_f32_len1024, delayedBuf, 0, 1 );

//
// process the frequency spectrum
//
			PitchShift( delayedBuf );

//
// inverse FFT
//
			arm_cfft_f32( &arm_cfft_sR_f32_len1024, delayedBuf, 1, 1 );
//
// Takes first half of delayed buffer and adds it to second half of proc buf
// and puts it in a 512 buffer to be transmitted
//
			counts = 0;
			for(loop=0;loop < 1024; loop+=2)
			{
				Buffers[PROCOUTOFFSET].Buf[counts] = delayedBuf[loop] + procBuf[loop + (SIZE*2)] +AD_Offset;
				counts++;
			}
//
// Set a flag to allow tx
//
			Buffers[PROCOUTOFFSET].Full = 3;
			Buffers[PROCOFFSET].Full = 2;
			WindowingDone = TRUE;
			FFT_DEBUG_SIGNAL_OFF;
		}


		else if (( 4 == WindowingState  ) && ( FALSE == WindowingDone ))
		{
			FFT_DEBUG_SIGNAL_ON;
//
// fills the first half of delayedbuf and second half of procbuf
//
			counts = SIZE*2; 	//1024
			for(loop=0; loop < SIZE; loop++)
			{
				procBuf[counts] = Buffers[PROCOFFSET].Buf[loop] * Hamming[loop + SIZE];
				procBuf[counts+1] = 0;
				delayedBuf[counts-1024] = Buffers[PROCOFFSET].Buf[loop] * Hamming[loop];
				delayedBuf[counts-1023] = 0;
				counts+=2;
			}

//
// FFT
//
			arm_cfft_f32( &arm_cfft_sR_f32_len1024, procBuf, 0, 1 );

//
// process the frequency spectrum
//
			PitchShift( procBuf );

//
// inverse FFT
//
			arm_cfft_f32( &arm_cfft_sR_f32_len1024, procBuf, 1, 1 );

//
// takes second half of delayed buffer and adds it to first half of proc buf
// and puts it in a 512 buffer to be transmitted
//
			counts = 0;
			for(loop=0;loop < 1024; loop+=2)
			{
				Buffers[PROCOUTOFFSET].Buf[counts] = delayedBuf[loop+(SIZE*2)] + procBuf[loop] +AD_Offset;
				counts++;
			}
//
// Set a flag to allow tx
//
			Buffers[PROCOUTOFFSET].Full = 3;
			Buffers[PROCOFFSET].Full = 2;
			WindowingDone = TRUE;
			FFT_DEBUG_SIGNAL_OFF;
		}

	}
}
