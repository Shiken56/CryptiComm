/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Jun 3, 2025
 *      Author: aayus
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include <stdint.h>
#include <stddef.h>

#include "stm32f446xx.h"
/*
 * Config structure for spi
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 * handle structure for spix peripheral
 */
typedef struct
{
	SPI_RegDef_t	*pSPIx;  			/*Holds base addr of the peripheral*/
	SPI_Config_t	SPIConfig;
	uint8_t *pTxBuffer	;				//Store Tx buffer address
	uint8_t *pRxBuffer	;				//Store the Rx buffer adddress
	uint32_t TxLen	;					//Store the TxLen
	uint32_t RxLen	;					//Store the RxLen
	uint8_t TxState	;					//Tx state
	uint8_t RxState	;					//Rx state
}SPI_Handle_t;


/*
 * SPI busy or ready config
 */

#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * Possible SPI application events
 */

#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT  2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4


/*
 * Different macros for SPI
 */

/*
 * DEVICE MODES
 */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0


/*
 * SPI Bus config modes duplexes
 */

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
//#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	3  (as for this just remove miso)
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * Clock speed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * SPI dff
 */
#define SPI_DFF_8BITS 	0
#define SPI_DFF_16BITS	1

/*
 * CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * CPHA
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * SSM
 */
#define SPI_SSM_EN		1
#define SPI_SSM_DI		0

/*
 * SPI related status flag defn
 */

#define SPI_TXE_FLAG 	(1<< SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1<< SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1<< SPI_SR_BUSY)




/***********************************************************************************************************
 * 												APIs for SPI protocol
 *
 * *********************************************************************************************************
 */


/*
 * Peripheral Clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 * Init and De-Init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Deinit(SPI_RegDef_t *pSPIx);

/*
 * Data send and recieve
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// using the handle for Interrupt based read write

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ and ISR config
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * other APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi );
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx );
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callbacks
 */

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);








#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
