/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Jun 3, 2025
 *      Author: aayus
 */
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx.h"



/***********************************************************************************************************
 * 												APIs for SPI protocol
 *
 * *********************************************************************************************************
 */


/*
 * Peripheral Clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	//based on what the user gives, disable that particular gpio peripheral
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();

		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}

	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();

		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}


}


/*
 * Init and De-Init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first config the SPI_CR1 reg

	uint32_t tempreg = 0;

	//1. config the device mode
	tempreg |= pSPIHandle ->SPIConfig.SPI_DeviceMode << 2 ;

	//2. Config the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode bit should be cleared
		tempreg &=~(1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode bit should be set
		tempreg |= (1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &=~(1<<15);
		//RXONLY bit must be set
		tempreg |= (1<<10);

	}
	//2. Config the SSM bit (software slave)
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM <<SPI_CR1_SSM;

	//3. Config the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed <<SPI_CR1_BR;

	//4. Config the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF <<SPI_CR1_DFF;

	//5. Config the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL <<SPI_CR1_CPOL;

	//6. Config the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA <<SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;

}
void SPI_Deinit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();

	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}


/*
 * Status flag functions
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	//the function will give a 1 or 0

	// Get the status reg bit and check its value
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	else
	{
		return FLAG_RESET;
	}
}

/*
 * Data send and recieve         Blocking Call
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait till TXE is set, only then we can transfer data

		//while(!(pSPIx->SR & (1<<1))); instead make function

		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)==FLAG_RESET);

		//2. Check the DFF format in CR1

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16BIT DFF

			//1. Load the data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;

			//Now typw cast pointer to 2 bytes so that ++ inc by 2 mem elements
			(uint16_t*)pTxBuffer++;


		}
		else
		{
			//8bit DFF

			pSPIx->DR = *((uint8_t*) pTxBuffer);
			Len--;
			pTxBuffer++;

		}


	}
}


void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait till RXNE is set, only then we can recieve data

			//while(!(pSPIx->SR & (1<<1))); instead make function

			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)==FLAG_RESET);

			//2. Check the DFF format in CR1

			if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
				//16BIT DFF

				//1. Load the data from DR to rx buffer
				*((uint16_t*)pRxBuffer) = pSPIx->DR  ;
				Len--;
				Len--;

				//Now typw cast pointer to 2 bytes so that ++ inc by 2 mem elements
				(uint16_t*)pRxBuffer++;


			}
			else
			{
				//8bit DFF

				*((uint8_t*) pRxBuffer) = pSPIx->DR ;
				Len--;
				pRxBuffer++;

			}


		}
}




/*
 * IRQ and ISR config
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. Lets find out the IPR reg
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	//In case of st, the lower 4 bits arent used in PR, hence give a +4 to the reg section
	uint8_t shift_amount = (8* iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);

	//write into the nvic IPR, the priority for the particular IRQ number

	*(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority<< shift_amount );
}





/*
 * Other peripheral apis
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi )
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &=~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &=~(1 << SPI_CR1_SSI);
	}
}





/*
 * Intrrupt based read write
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_BUSY_IN_TX)
	{
			//Only start the process when the state is currently free

			//1. Save Txx buffer address and len info in some global variables

			pSPIHandle->pTxBuffer = pTxBuffer;
			pSPIHandle->TxLen = Len;

			//2. Mark the SPI state as busy in transmission so
			//no other state takes over SPI peripheral

			pSPIHandle->TxState = SPI_BUSY_IN_TX;

			//3. Enable TXEIE control bit to get interrupt whenever the TXE flag is set in SR

			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
			//4. Data transmission will be handled by the ISR code

	}

	return state;

}
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX)
	{
			//Only start the process when the state is currently free

			//1. Save Rx buffer address and len info in some global variables

			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;

			//2. Mark the SPI state as busy in transmission so
			//no other state takes over SPI peripheral

			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			//3. Enable RXNEIE control bit to get interrupt whenever the TXE flag is set in SR

			pSPIHandle->pSPIx->CR2 |= (1<< SPI_CR2_RXNEIE);
			//4. Data recieving will be handled by the ISR code

	}

	return state;

}

/*
 * SPI IRQ handling
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);



void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	//first check for TXE bit and also if TXEIE enabled
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//first check for RXNE bit and also if RXNEIE enabled
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}


	//first check for OVRR bit and also if TXEIE enabled
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle OVR
		spi_ovr_err_interrupt_handle(pHandle);
	}




}


/*
 * Helper function implementations
 */


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16BIT DFF

		//1. Load the data into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;

		//Now typw cast pointer to 2 bytes so that ++ inc by 2 mem elements
		(uint16_t*)pSPIHandle->pTxBuffer++;


	}
	else
	{
		//8bit DFF

		//1. Load the data into DR
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;

		//Now typw cast pointer to 2 bytes so that ++ inc by 2 mem elements
		pSPIHandle->pTxBuffer++;


	}
	if(! pSPIHandle->TxLen)
	{
		//TXLen is zero, close the spi transmissioon and inform application that tx is over

		//Make the TXEIE bit as reset
		//This is so that any interrupt wont be set

		//closing the SPI transmission

		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}

}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16BIT DFF

		//1. Load the data into DR
		*((uint16_t*)pSPIHandle->pRxBuffer)= pSPIHandle->pSPIx->DR ;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;


	}
	else
	{
		//8bit DFF
		*((uint16_t*)pSPIHandle->pRxBuffer)= pSPIHandle->pSPIx->DR ;
		pSPIHandle->RxLen--;


		pSPIHandle->pRxBuffer++;



	}
	if(! pSPIHandle->RxLen)
	{
		//TXLen is zero, close the spi transmissioon and inform application that tx is over

		//Make the TXEIE bit as reset
		//This is so that any interrupt wont be set

		//closing the SPI transmission
		SPI_CloseReception(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Note: If during transmission OVR happens, this fxn wont clear the OVR Flag, u need another fxn
	//We dont want to read unless the device is not transmitting, or else transmit data will get lost



	uint8_t temp;
	//1. clear the ovr flag
	// (read the DR, then read the SR acc to datasheet
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX )
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

	}
	//typecast temp to nothing
	(void) temp;

	//2. inform the application (using a callback)

	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);


}



/*
 * Functions called by application to be used
 */

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &=~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &=~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx )
{
	uint8_t temp;

		temp = pSPIx->DR;
		temp = pSPIx->SR;
		(void) temp;
}


/*
 * Weak implementation
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//This is weak implementation, user may take over
}













