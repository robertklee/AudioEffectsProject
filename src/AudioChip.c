/*
 * AudioChip.c
 *
 *  Created on: May 15, 2016
 *      Author: Michael
 */

#include "stm32f4xx_hal.h"
#include "AudioChip.h"

#define AUDIO_RST_PIN	GPIO_PIN_4
#define AUDIO_SCL_PIN   GPIO_PIN_6
#define	AUDIO_SDA_PIN	GPIO_PIN_9
//
// Settings for 44.1 kHz sampling.
//
int
	pllr = 271,
	plln = 2,
	i2sdiv = 6,
	i2sodd = 0;


I2C_HandleTypeDef
	I2c;

GPIO_InitTypeDef
	GPIO_InitStruct;


/*
 * InitAudioChip()
 *
 * Initialized the CS43L22 Audio chip for I2S audio generation.
 */

void EnableAudioCodecPassThru()
{
	GPIO_InitTypeDef
		GPIO_InitStruct;

	HAL_StatusTypeDef
		status;

	uint8_t
		check_value;

//
// Configure the I2C and SPI related pins
//

	GPIO_InitStruct.Pin = AUDIO_RST_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = AUDIO_SCL_PIN | AUDIO_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//
// Configure C7,C10,C12 from I2S (SPI3)
//
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//
// Peripheral clock enable
//
	__HAL_RCC_I2C1_CLK_ENABLE();


	I2c.Instance = I2C1;
	I2c.Init.ClockSpeed = 100000;
	I2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2c.Init.OwnAddress1 = 0;
	I2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2c.Init.OwnAddress2 = 0;
	I2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init( &I2c );


	__HAL_RCC_SPI3_CLK_ENABLE();

//
// Reset the CS43L22 audio chip.
//
	HAL_GPIO_WritePin(GPIOD, AUDIO_RST_PIN, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOD, AUDIO_RST_PIN, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOD, AUDIO_RST_PIN, GPIO_PIN_SET);
//	trace_printf( "Detecting audio chip: " );

//
// Check that we can communicate wit the chip
//
	status = HAL_I2C_IsDeviceReady(&I2c, 0x94, 3 , 100);

	if ( status == HAL_OK )
	{
//		trace_printf("found\n");
	}
	else
	{
//		trace_printf("failed\n\n");
		return;
	}
//	trace_printf( "ChipID: " );

	status =  HAL_I2C_Mem_Read( &I2c , 0x94 , 0x01 , 1 , & check_value , 1, 100);

	if ( status == HAL_OK )
	{
//		trace_printf("%d\n\n", check_value );
	}
	else
	{
//		trace_printf("failed\n\n");
		return;
	}

//
// Disable I2S.
//
	SPI3 ->I2SCFGR = 0;

//
// I2S clock configuration
// Disable PLLI2S and wait until is off
//
	RCC->CR &= ~RCC_CR_PLLI2SON;
	while ( RCC -> CR & RCC_CR_PLLI2SON );

	RCC ->CFGR &= ~RCC_CFGR_I2SSRC; // PLLI2S clock used as I2S clock source.
	RCC ->PLLI2SCFGR = (pllr << 28) | (plln << 6);

//
// Configure I2S.
//
	SPI3 ->I2SPR = i2sdiv | (i2sodd << 8) | SPI_I2SPR_MCKOE;

//
// Configure I2S as Master receiver, Phillips mode, 16-bit values, clock polarity low, enable.
//
	SPI3 ->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1 | SPI_I2SCFGR_I2SCFG_0
			| SPI_I2SCFGR_I2SE;

//
// WARNING: In MASTER TRANSMITTER MODE, the STM32F407 will only generate the master clock signal when data is being
// actively transmitted.  At all other times, the Master Clock (MCLK) signal will be turned off.
// If you are using the CS43L22 chip, which uses the MCLK to run a charge pump for its analog components during
// ANALOG PASSTHRU mode this might have unintended consequences.
//

//
// Enable PLLI2S and wait until it is ready.
//
	RCC ->CR |= RCC_CR_PLLI2SON;
	while (!(RCC ->CR & RCC_CR_PLLI2SRDY ));

	WriteRegister(0x00, 0x99);
	WriteRegister(0x47, 0x80);
	WriteRegister(0x32, 0x80);
	WriteRegister(0x32, 0x00);
	WriteRegister(0x00, 0x00);

	WriteRegister(0x02, 0x01);
	WriteRegister(0x04, 0xaf);			// HP always on, Speakers always off
	WriteRegister(0x08, 0x01);			// Select AINA1 as input
	WriteRegister(0x09, 0x01);			// Select AINB1 as input
	WriteRegister(0x0e, 0xc0);			// Analog pass through enabled.
	WriteRegister(0x14, 0xe0);
	WriteRegister(0x15, 0xe0);
	WriteRegister(0x02, 0x9e);			// Activate the chip

	return;
}

//static HAL_StatusTypeDef ReadRegister(uint8_t address, uint8_t * value)
//{
//	HAL_StatusTypeDef hresult;
//	hresult =  HAL_I2C_Mem_Read( &I2c , 0x94 , address , 1 , value , 1, 100);
//
//	return(hresult);
//}

static HAL_StatusTypeDef WriteRegister(uint8_t address, uint8_t value)
{
	HAL_StatusTypeDef hresult;
	hresult = HAL_I2C_Mem_Write( &I2c, 0x94, address, 1, & value, 1, 100);

	return(hresult);
}

