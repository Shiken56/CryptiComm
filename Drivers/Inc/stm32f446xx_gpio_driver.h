/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: May 19, 2025
 *      Author: aayus
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*This is for the config structure of a GPIO pin*/

typedef struct
{
	uint8_t GPIO_PinNumber;					//possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;					//possible values from @GPIO_PIN_MODES
	uint8_t GPIO_Pinspeed;					//possible vlaues from  @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl;			//possible values from @GPIO_PUPD_CONTROL
	uint8_t GPIO_PinOPType;					//possible values from @GPIO_PIN_OP_TYPES
	uint8_t GPIO_PinAltFunMode;				//possible values from @GPIO_PIN_AFR

}GPIO_PinConfig_t;
/*this is a handle structure for a GPIO pin*/

typedef struct
{
	GPIO_RegDef_t *pGPIOx; 					/*This holds base addr of the gpio port which the pin belongs to*/
	GPIO_PinConfig_t GPIO_PinConfig; 		/*This holds the GPIO pin congif settings*/
}GPIO_Handle_t; //lcd_signalA;

/*Gpio pin numbers
 *  @GPIO_PIN_NUMBERS
 */

#define GPIO_PIN_NO0		0
#define GPIO_PIN_NO1		1
#define GPIO_PIN_NO2		2
#define GPIO_PIN_NO3		3
#define GPIO_PIN_NO4		4
#define GPIO_PIN_NO5		5
#define GPIO_PIN_NO6		6
#define GPIO_PIN_NO7		7
#define GPIO_PIN_NO8		8
#define GPIO_PIN_NO9		9
#define GPIO_PIN_NO10		10
#define GPIO_PIN_NO11		11
#define GPIO_PIN_NO12		12
#define GPIO_PIN_NO13		13
#define GPIO_PIN_NO14		14
#define GPIO_PIN_NO15		15


/*Gpio pin mode types
 *  @GPIO_PIN_MODES
 */

#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG 	3

#define GPIO_MODE_IT_FT 	4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * GPIO pin possible output types
 * @GPIO_PIN_OP_TYPES
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * GPIO pin possible output speeds
 * @GPIO_PIN_SPEEDS
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * GPIO pupd config macros
 * GPIO_PUPD_CONTROL
 */

#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2


/*********************************************************************************************************************
 * 											APIs supported by this driver
 * ********************************************************************************************************************
 */



/*Peripheral clock setup */

//Takes the peripheral and enable or disable

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi );

/*Init and de-init*/

void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Data read and write*/


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/*IRQ Config and ISR settings*/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);







#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
