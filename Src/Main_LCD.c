

#include "stm32f446xx.h"
#include "aes.h"

#include <string.h>
#include <stdint.h>

int main(void)
{




		GPIO_PeriClockControl(GPIOA, ENABLE);
		GPIO_PeriClockControl(GPIOB, ENABLE);
		GPIO_PeriClockControl(GPIOC, ENABLE);


		lcd_init();
		lcd_set_cursor(0,1);

	/*
	 * User application decides state of the GPIO Peripheral
	 */




		lcd_print_string("Mark 27");




	return 0;
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);
}
/*
 * lcd.c
 *
 *  Created on: May 21, 2025
 *      Author: aayus
 */


