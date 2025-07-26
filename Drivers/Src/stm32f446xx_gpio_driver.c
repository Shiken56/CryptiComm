/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: May 19, 2025
 *      Author: aayus
 */

#include "stm32f446xx_gpio_driver.h"



/*Peripheral clock setup */

//Takes the peripheral and enable or disable

//always document the function definitions as comments

/****************************************************************
 * @fn 						GPIO Peripheral control
 *
 * @brief					this function enables/ disables the gpio clock
 * @param  					base address of the gpio handle being used
 * @param					the peripheral is enabled or disabled
 *
 * @return 					void
 * @Note
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi )
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();

		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();

		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

	}

}

/*Init and de-init*/

void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{

	uint32_t temp =0;

	//enable the gpio clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of the gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<< 2* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |=temp; //setting
		temp = 0;

	}
	else
	{

		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT)
		{
			//config the FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//configure the RTSR
			EXTI->RTSR &=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT)
		{
			//config the RTSR

			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//configure the RTSR
			EXTI->FTSR &=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RFT)
		{
			//config the RTSR AND FTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//configure the RTSR
			EXTI->FTSR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);



		//3. enable the exti interrupt IMR
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


	}


	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Pinspeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<2*( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. configure the pupd settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<< 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. configure the optype

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
			pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. configure the alt function type
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//we have 2 AFR, so we gotta decide which register to use
		uint8_t temp1, temp2;

		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8;
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &=~(0xF <<(4* temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4* temp2));


	}


}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	//based on what the user gives, disable that particular gpio peripheral
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();

	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}


/*Data read and write*/

/*
 * Reading from input pin function
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	/*Only getting that particular bit as return value*/
	value = (uint8_t)((pGPIOx->IDR >> PinNumber)&0x00000001);

	return value;
}

/*
 * Reading from a port (all 16 pins at once)
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t value;

	/*Now getting that particular port (all bits) as return value*/
	value = (uint16_t)(pGPIOx->IDR);

	return value;
}

/*
 * Write function to GPIO pin as an output
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	//based on value we will set or reset the bit using conditional statement

	if(Value == GPIO_PIN_SET)
	{
		//Write 1 to the ODR at that particular bit
		pGPIOx ->ODR |= (1 << PinNumber);

	}
	else
	{
		//write 0
		pGPIOx ->ODR &=~(1 << PinNumber);
	}
}

/*
 * Writing to the GPIO Port as output
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*
 * This is just continuously changing the value of the pin
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

/*IRQ Config and ISR settings*/

//note- irq number is equal to the interrupt to be used

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1<< IRQNumber);

		}
		if(IRQNumber >31 && IRQNumber <= 63 )
		{
			//program ISER1 register //32-63
			*NVIC_ISER1 |= (1<< (IRQNumber %32));

		}
		if(IRQNumber >63 && IRQNumber <= 95)
		{
			//prgram the ISER2 register //64-95
			*NVIC_ISER2 |= (1<< (IRQNumber %32));

		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1<< IRQNumber);

		}
		if(IRQNumber >31 && IRQNumber <= 63 )
		{
			//program ICER1 register //32-63
			*NVIC_ICER1 |= (1<< (IRQNumber %32));

		}
		if(IRQNumber >63 && IRQNumber <= 95)
		{
			//prgram the ICER2 register //64-95
			*NVIC_ICER2 |= (1<< (IRQNumber %32));

		}

	}

}

/*
 * Configuring the priority of an interrupt (
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. Lets find out the IPR reg
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	//In case of st, the lower 4 bits arent used in PR, hence give a +4 to the reg section
	uint8_t shift_amount = (8* iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);

	//write into the nvic IPR, the priority for the particular IRQ number

	*(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority<< shift_amount );
}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register for that pin no.
	if(EXTI->PR & (1<<PinNumber))
	{
		//clear the bit
		EXTI->PR |= (1<<PinNumber);
		//writing to the bit will clear it
	}
}




