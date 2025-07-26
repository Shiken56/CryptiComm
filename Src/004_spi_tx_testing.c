/*
 * 004spiTx.c
 *
 *  Created on: Jun 5, 2025
 *      Author: aayus
 */

/*
 * PD3 SPI2_SCLK
 * PC2 SPI2_MISO
 * PB14 SPI2_MOSI
 * PB12 SPI2_NSS
 * Alt function mode 5
 */
#include "stm32f446xx.h"
#include <string.h>


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPI2Pins;

	SPI2Pins.pGPIOx = GPIOA;
	SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2Pins.GPIO_PinConfig.GPIO_PinAltFunMode= 5;
	SPI2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI2Pins.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;

	//SCLK -  PA5
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO5;
	GPIO_Init(&SPI2Pins);

	//MOSI - PA7
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO7;
	GPIO_Init(&SPI2Pins);

//	//MISO
//	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO6;
//	GPIO_Init(&SPI2Pins);
//
//	//NSS
//	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO4;
//	GPIO_Init(&SPI2Pins);



}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI1;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //software ssm enable for NSS pin

	SPI_Init(&SPI2handle);

}

int main(void)
{
	char user_data[] = "Hello world";
	//function to init GPIO pins for AFR of SPI2
	SPI2_GPIOInits();

	//now the peripheral init
	SPI2_Inits();

	//Enable the SPI2 peripheral and SSI bit
	SPI_SSIConfig(SPI1, ENABLE);
	SPI_PeripheralControl(SPI1, ENABLE);


	SPI_SendData(SPI1,(uint8_t*)user_data, strlen(user_data));


	while(1);
	return 0;

}
