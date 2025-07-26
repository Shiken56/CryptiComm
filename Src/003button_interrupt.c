/*
 * 003button_interrupt.c
 *
 *  Created on: May 29, 2025
 *      Author: aayus
 */


#include "stm32f446xx.h"
#include "string.h"


int main(void)
{

	/* order is
	 * 1. Pin Number
	 * 2. Pin mode
	 * 3. Pin speed
	 * 4. Pin output type
	 * 5. Pin PuPd type
	 *
	 * enable clock
	 */

	GPIO_Handle_t GpioLed,GpioBtn;

	//imp to avoid random data corruption
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO5;
	GpioLed.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	//GpioBtn.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;  (doesnt matter if its input)
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU; //as the external circuit already has the pull up/down resistor



	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);

	//IRQ CONFIGS

	//only to init the interrupt for Button pin
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);

	return 0;







}

void EXTI15_10_IRQHandler(void)
{
	mdelay(200);
	GPIO_IRQHandling(GPIO_PIN_NO13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO5);
}
