/*
 * stm32f446xx.h
 *
 *  Created on: May 19, 2025
 *      Author: aayus
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo 		volatile
#define __weak		__attribute__((weak))

/* *******************************Processor specific details**********************************/

/*
 * ARM Cortex Mx processor NVIC ISERx reg addr
 */

#define NVIC_ISER0					((__vo uint32_t *) 0xE000E100)
#define NVIC_ISER1					((__vo uint32_t *) 0xE000E104)
#define NVIC_ISER2					((__vo uint32_t *) 0xE000E108)
#define NVIC_ISER3					((__vo uint32_t *) 0xE000E10C)

/*
 * ARM Cortex Mx processor NVIC ICERx reg addr
 */
#define NVIC_ICER0					((__vo uint32_t *) 0xE000E180)
#define NVIC_ICER1					((__vo uint32_t *) 0xE000E184)
#define NVIC_ICER2					((__vo uint32_t *) 0xE000E188)
#define NVIC_ICER3					((__vo uint32_t *) 0xE000E18C)

//i.e. in the IPR reg how many bits are actually used


/*
 * Priority reg address
 */

#define NVIC_PR_BASE_ADDR  			((__vo uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED		4

/*
 * base addr of pr
 */

/*BASE ADDDR OF FLASH AND SRAM*/

#define FLASH_BASEADDR				0x08000000U


#define SRAM1_BASEADDR  			0x20000000U			//112kB sram 1 is the main ram
#define SRAM2_BASEADDR				0X20001C00U


#define ROM_BASEADDR 				0x1FFF0000U 		//ROM is the same as system memory
#define SRAM 						SRAM1_BASEADDR

/*AHB AND APPB BUS PERIPHERALS BASE ADDR*/

#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASE
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U



/*BASE ADDR OF ALL PERIPHERALS IN THE AHB1 BUS*/

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0X0000)
#define GPIOB_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)


/*BASE ADDR OF ALL THE APB1 BUS PERIPHERALS */

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR               (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR               (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR              (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR              (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR            (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR            (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR             (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR             (APB1PERIPH_BASEADDR + 0x5000)


/* APB2 peripheral base addresses */
#define SPI1_BASEADDR              (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR              (APB2PERIPH_BASEADDR + 0x3400)

#define USART1_BASEADDR            (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR            (APB2PERIPH_BASEADDR + 0x1400)

#define EXTI_BASEADDR              (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR            (APB2PERIPH_BASEADDR + 0x3800)




/**************************************Peripheral structure definition************************************************/

/*GPIO peripheral*/

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

}GPIO_RegDef_t;

/*RCC Peripheral*/

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED5;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
} RCC_RegDef_t;

/*
 * SYSCFG Peripheral
 */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t	RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t	RESERVED2[2];
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;

/*
 * EXTI Peripheral
 */

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;

/*
 * SPI peripheral
 */

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

/*
 * I2C peripheral
 */
typedef struct
{
  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;



/*Peripheral definitions */

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC 					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1					((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_BASEADDR)


/*GPIO CLOCK enables*/

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1<<7))

/*gpio CLOCKS Disable */

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &=~ (1<<0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &=~ (1<<1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &=~ (1<<2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &=~ (1<<3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &=~ (1<<4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &=~ (1<<5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &=~ (1<<6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &=~ (1<<7))

/*
 * Clock En macros for SYSCFG Peripheral
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1<<14))




/*SPI CLOCK ENABLE*/

#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1<<12))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1<<13))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1<<15))

/*
 * SPI CLK disable
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &=~ (1<<12))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &=~ (1<<13))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &=~ (1<<14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &=~ (1<<15))

/*
 * I2C CLK enable
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * I2C CLK disable
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &=~ (1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &=~ (1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &=~ (1 << 23))

/*
 * Bit position defn for I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15


/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)


/*
 * IRQ Number of different interrupt/ events
 */


#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_I2C1_EV		31
#define	IRQ_MO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define	IRQ_MO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define	IRQ_MO_I2C3_ER		73

/*
 * macros for all possible priority levels
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI15		15
















/*
 * Macros to reset GPIOx peripherals
 */

//WE SET THE AHB1RSTR BIT AND THEN RESET IT TO TURN IT OFF using do while
//No need semicolon for the while part as the programmer will itself put the semicolon there
//Technique do while is used to execute multiple c statements using single c macro

#define GPIOA_REG_RESET()		do {(RCC->AHB1RSTR |= (1<<0) ); (RCC->AHB1RSTR &=~(1U<<0));} while(0)
#define GPIOB_REG_RESET()		do {(RCC->AHB1RSTR |= (1<<1) ); (RCC->AHB1RSTR &=~(1U<<1));} while(0)
#define GPIOC_REG_RESET()		do {(RCC->AHB1RSTR |= (1<<2) ); (RCC->AHB1RSTR &=~(1U<<2));} while(0)
#define GPIOD_REG_RESET()		do {(RCC->AHB1RSTR |= (1<<3) ); (RCC->AHB1RSTR &=~(1U<<3));} while(0)
#define GPIOE_REG_RESET()		do {(RCC->AHB1RSTR |= (1<<4) ); (RCC->AHB1RSTR &=~(1U<<4));} while(0)
#define GPIOF_REG_RESET()		do {(RCC->AHB1RSTR |= (1<<5) ); (RCC->AHB1RSTR &=~(1U<<5));} while(0)
#define GPIOG_REG_RESET()		do {(RCC->AHB1RSTR |= (1<<6) ); (RCC->AHB1RSTR &=~(1U<<6));} while(0)
#define GPIOH_REG_RESET()		do {(RCC->AHB1RSTR |= (1<<7) ); (RCC->AHB1RSTR &=~(1U<<7));} while(0)

#define SPI1_REG_RESET()			do {(RCC->APB2RSTR |= (1<<12) );(RCC->APB2RSTR &=~ (1<<12));}while(0)
#define SPI2_REG_RESET()			do {(RCC->APB1RSTR |= (1<<14) );(RCC->APB2RSTR &=~ (1<<12));}while(0)
#define SPI3_REG_RESET()			do {(RCC->APB1RSTR |= (1<<15) );(RCC->APB2RSTR &=~ (1<<12));}while(0)
#define SPI4_REG_RESET()			do {(RCC->APB2RSTR |= (1<<13) );(RCC->APB2RSTR &=~ (1<<12));}while(0)

/*
 * portcode function
 */

#define GPIO_BASEADDR_TO_CODE(x)		((x==GPIOA)?0 :\
										(x==GPIOB)? 1:\
										(x==GPIOC)? 2:\
										(x==GPIOD)? 3:\
										(x==GPIOE)? 4:\
										(x==GPIOF)? 5:\
										(x==GPIOG)? 6:\
										(x==GPIOH)? 7:0)



/*Some generic macros */

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET 			RESET
#define FLAG_SET			SET

/***********************************************************************************************************
*Bit position definitions of SPI peripherals
*Bit
************************************************************************************************************

*/

/*
 * SPI CR1 register
 */

#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * SPI CR2 register
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

/*
 * SPI SR register
 */

#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8



/*
 * Including all the header files
 */

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "Lcd1602.h"

#endif /* INC_STM32F446XX_H_ */
