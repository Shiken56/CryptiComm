/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Jun 16, 2025
 *      Author: aayus
 */


/*
 * Getting the RCC clock
 */


#include "stm32f446xx_i2c_driver.h"
/*
 * Getting PLL
 */

uint16_t AHB_PreScalar[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScalar[4] = {2,4, 8, 16};


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx );
static void I2C_ClearADDRFlagIT(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1 ;

	//for write

	SlaveAddr &=~(1);
	pI2Cx->DR = SlaveAddr;

}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1 ;

	//for read

	SlaveAddr |=(1);
	pI2Cx->DR = SlaveAddr;

}



static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx )
{
	uint32_t dummyread = pI2Cx->SR1;
	 dummyread = pI2Cx->SR2;
	(void)dummyread;
}

static void I2C_ClearADDRFlagIT(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyread = pI2Cx->SR1;
	 dummyread = pI2Cx->SR2;
	(void)dummyread;
}



static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}







uint8_t RCC_GetPLLOutputClock()
{


	return 0;
}


/*
 * Peripheral Clock setup
 */

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk ;
	uint8_t clksrc, temp, ahbp, apb1p;

	//1. As SWS systemm clock switch status SWS done by hardware

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc==0)
	{
		SystemClk = 16000000;
	}
	else if(clksrc==1)
	{
		SystemClk = 8000000;
	}
	else if(clksrc ==2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//2. For reading the AHB prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp <8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScalar[temp-8];
	}

	//3. For reading the APB prescalar
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 8)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScalar[temp-4];
	}

	pclk1= (SystemClk / ahbp)/apb1p ;

	return pclk1;

}


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	//based on what the user gives, disable that particular gpio peripheral
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();

		}


	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();

		}
	}


}


/*
 * Init and De-Init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg =0;

	//enable the clock for i2c peripheral

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);


	//1. Enabling the automatic ACK for master CR1
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	//2. Config the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F) ;

	//3. Config the OAR reg for address if slave

	tempreg = 0;
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress <<1;
	tempreg |= ( 1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//Clock configurations

	//4. CCR Calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is std mode so bit 15 is 0

		ccr_value = RCC_GetPCLK1Value()/ (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ;
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		//mode is fast mode so bit 15 is 1
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if( pI2CHandle->I2C_Config.I2C_FMDutyCycle ==I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value()/ (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ;
		}
		else if( pI2CHandle->I2C_Config.I2C_FMDutyCycle ==I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value()/ (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ;
		}
		tempreg |= (ccr_value & 0xFFF);

	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//Trise configuration


	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is std mode


		tempreg = (RCC_GetPCLK1Value()/1000000U) + 1 ; //+1 acc to ref manual


	}
	else
	{
		//mode is fast mode

		tempreg = (RCC_GetPCLK1Value()* 300 /1000000000U) + 1 ; //+1 acc to ref manual
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}
void I2C_Deinit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and recieve
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//Note: Until SB is cleared SCL will be stretched LOW

	while((!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

	//3. Send the address of slave with r/w bit

	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm the address phase is complete by checking the ADDR flag
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag reading SR1 AND SR2
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. Send dataa till len becomes 0

	while( Len > 0)
	{
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. When Len becomes zero, wait for TXE=1 and BTF =1 before STOP condit
	//Note : TXE=1, BTF = 1, means both SR, DR empty
	//When BTF =1, SCL will be stretched (pulled to low)

	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TXE)))
	{

	}

	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BTF)))
	{

	}

	//8. Generate STOP, master need'nt wait for completion of stop
	// Note: generate STOP, automatically clears the BTF

		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//2. confirm that start generation is completed by checking the SB flag in the SR1
		//Note: Until SB is cleared SCL will be stretched LOW

		while((!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

		//3. Send the address of slave with r/w bit

		I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

		//4. Confirm the address phase is complete by checking the ADDR flag
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

		//proced if only 1 byte
		if(Len == 1)
		{
			//Disable ACK

			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

			//Clear the ADDR flag
			I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

			//wait till RXNE becomes 1

			while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE)));

			//generate STOP
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);






			//read data into the buffer

			*pRxBuffer = pI2CHandle->pI2Cx->DR;
		}

		else if(Len >1)
		{

			//clear the ADDR flag
			I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

			//read the data till length becomes zero
			for( uint32_t i = Len ; i>0 ; i--)
			{
				//wait till RXNE =1

				while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE)));


				if(i == 2)
				{
					//Disable ACK

					I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

					//generate STOP condition

					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				}

				//read the data from DR into buffer
				*pRxBuffer = pI2CHandle->pI2Cx->DR;

				//inc the buffer address
				pRxBuffer++;

				//Note: buffer is a variable that stores the value of the first element of array

			}


		}

		if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		{
			//re- enable the acking after everything is done. as we disabled it
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
		}



}


/*
 * Interrupt based transmitting and recieving
 */


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	//1. Make sure the bus isn't busy

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		//1. Update the buffer and values

		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;


		//2. Generate START

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//3. Enable the ITBUFEN Control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//3. Enable the ITEVEN Control bit

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//4. Enable the ITEREN Control bit

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;



}


uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;


	//1. Make sure the bus isn't busy

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		//1. Update the buffer and values

		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;


		//2. Generate START

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//3. Enable the ITBUFEN Control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//3. Enable the ITEVEN Control bit

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//4. Enable the ITEREN Control bit

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;


}


/*
 * IRQ and ISR config
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber %32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}
		else if( IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if( IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}


	}


}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. first find the IPR of the register
	uint8_t iprx = IRQNumber/ 4;
	uint8_t iprx_section = IRQNumber %4;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8- NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/*
 * other APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi )
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE	);
	}
	else
	{
		pI2Cx->CR1 &=~ (1 << I2C_CR1_PE	);
	}
}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	//the function will give a 1 or 0

	// Get the status reg bit and check its value
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}

	else
	{
		return FLAG_RESET;
	}
}

void  I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}

	else
	{
		//disable the ack
		pI2Cx->CR1 &=~ (1 << I2C_CR1_ACK);
	}




}

/*
 * IRQ Handler APIs
 */


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint8_t temp1, temp2, temp3;

	//check if the interrupt event flags are actually set?

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);
	//1. Handle for interrupt by SB
	//note : SB is only applicable in master mode
	if( temp1 && temp3)
	{
		//1. Generated due to SB event

		//2. Execute the address phase
	    if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	    {
	    	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
	    }
	    else if ( pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
	    {
	    	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
	    }
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);

	//2. Handle for interrupt generated by ADDR event
	//note: When master mode : Addr is sent
	//		When slave mode  : Addr is matched with own addr

	if( temp1 && temp3)
	{
		//interrupt for ADDR flag set, clock stretch until further action
		//Imp to first clear the I2C addr flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	//3. Handle for interrupt generated by BTF ( Byte Transfer Finished) event
	if( temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Make sure TXE also set
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE))
			{
				// BTF, TXE both are 1
				if(pI2CHandle->TxLen == 0)
				{
					//1. Generate STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. Reset all member elem of handle struct
					I2C_CloseSendData();

					//3. Notify applic about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle for interrupt generated by STOPF event
	//note: stop detection flag only in slave mode
	if( temp1 && temp3)
	{
		//STOPF flag is set
		//clear the stop flag
		// read SR1, write to CR1

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//notify the appplicaition that stop is detected

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle for interrupt by TXE event
	if(temp1 && temp2 && temp3 )
	{
		//check if device is master
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{


			//TXE flag is set
			//We do data transmit only if busy in TX
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->TxLen >0)
				{
					//1. load the data into DR

					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

					//2. Decrement Tx Length
					pI2CHandle->TxLen--;

					//3. Increase buffer address
					pI2CHandle->pTxBuffer++;

				}
			}
		}

	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);

	//6. Handle for interrupt by RXNE event

	if(temp1 && temp2 && temp3 )
	{
		//RXNE flag is set

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//We have to recieve data
			if(pI2CHandle->RxSize == 1)
			{

			}

			if(pI2CHandle->RxSize > 1)
			{

			}

		}
	}

}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);




