/*
 * 001ledtoggle.c
 *
 *  Created on: May 20, 2025
 *      Author: aayus
 */

#include "stm32f446xx.h"

void delay(void)
{
	for(uint32_t i=0; i<100; i++)
	{

	}
}
int main(void)
{

	/*
	 * User application decides state of the GPIO Peripheral
	 */
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO5);
		delay();
	}
	return 0;
}
