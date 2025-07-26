/*
 * ledbutton.c
 *
 *  Created on: May 21, 2025
 *      Author: aayus
 */


#include "stm32f446xx.h"

void delay(void)
{
	for(uint32_t i=0; i<1000000; i++)
	{

	}
}




int main(void)
{

	/*
	 * User application decides state of the GPIO Peripheral
	 */

	GPIO_Handle_t GpioLed, GpioBtn;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO5;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	//GpioBtn.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;  (doesnt matter if its input)
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD; //as the external circuit already has the pull up/down resistor


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO13)== 0)
		{
			//delay();

			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO5);


		}
	}
	return 0;
}
