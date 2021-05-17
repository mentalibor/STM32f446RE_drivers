/******************************************************************************
** Copyright (c) 2021 Mentalibor. All rights reserved. Email: mentalibor@gmail.com
**
** Name
** stm32f44xx_i2c.c
**
** Purpose
** I2C driver for the STM32F44XX
**
** Revision
** 08-Feb-2021 (Mentalibor) Initial version.
**
******************************************************************************/

#include "stm32f44xx_i2c.h"

/** *****************************************************************************
 * @ingroup static_i2c
 * @brief Generates start condition of the I2C communication
 *
 * @param[in]   pI2Cx  Reference to the I2C definition structure
 *
 ******************************************************************************/
static void I2C_GenerateStartCondition ( I2C_RegDef_t *pI2Cx);

/** *****************************************************************************
 * @ingroup static_i2c
 * @brief Executes the address phase with write operation bit
 *
 * @param[in]   pGPIOx  	Reference to the I2C definition structure
 * @param[in]	SlaveAddr	Address of the slave device
 *
 ******************************************************************************/
static void I2C_ExecuteAddressPhaseWrite 	( I2C_RegDef_t *pI2Cx
											, uint8_t SlaveAddr
											);

/** *****************************************************************************
 * @ingroup static_i2c
 * @brief Executes the address phase with read operation bit
 *
 * @param[in]   pGPIOx  	Reference to the I2C definition structure
 * @param[in]	SlaveAddr	Address of the slave device
 *
 ******************************************************************************/
static void I2C_ExecuteAddressPhaseRead ( I2C_RegDef_t *pI2Cx
										, uint8_t SlaveAddr
										);

/** *****************************************************************************
 * @ingroup static_i2c
 * @brief Clears the ADDR flag by reading SR1 and SR2 registers
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
static void I2C_ClearADDRFlag( I2C_Handle_t *pI2CHandle);

/** *****************************************************************************
 * @ingroup static_i2c
 * @brief Handles the interrupt in case of RXNE event
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
static void I2C_MasterHandleRXNEInterrupt( I2C_Handle_t *pI2CHandle );

/** *****************************************************************************
 * @ingroup static_i2c
 * @brief Handles the interrupt in case of TXE event
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
static void I2C_MasterHandleTXEInterrupt( I2C_Handle_t *pI2CHandle );

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Generates stop condition of the I2C communication
 *
 * @param[in]   pI2Cx  Reference to the I2C definition structure
 *
 ******************************************************************************/
void I2C_GenerateStopCondition( I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}

 /** *****************************************************************************
  * @ingroup api_i2c
  * @brief Enables or disables the callback events
  *
  * @param[in]   pI2Cx   Reference to the I2C definition structure
  * @param[in]   EnorDi  ENABLE/DISABLE macro
  *
  ******************************************************************************/
void I2C_SlaveEnableDisableCallbackEvents	( I2C_RegDef_t *pI2Cx
		 	 	 	 	 	 	 	 	 	, uint8_t EnorDi
											)
 {
	 if(EnorDi == ENABLE)
	 {
			pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
			pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
			pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	 }
	 else
	 {
			pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
			pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
			pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	 }
 }

 /*******************************************************************************
  * @ingroup api_i2c
  * @brief Enables or disables the I2C peripheral
  *
  * @param[in]   pI2Cx   Reference to the I2C definition structure
  * @param[in]   EnorDi  ENABLE/DISABLE macro
  *
  ******************************************************************************/
void I2C_PeripheralControl	( I2C_RegDef_t *pI2Cx
							, uint8_t EnOrDi
							)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Enables or disables the I2C peripheral clock
 *
 * @param[in]   pI2Cx   Reference to the I2C definition structure
 * @param[in]   EnorDi  ENABLE/DISABLE macro
 *
 ******************************************************************************/
void I2C_PeriClockControl	( I2C_RegDef_t *pI2Cx
							, uint8_t EnorDi
							)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
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
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}

}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Initializes the I2C peripheral
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
void I2C_Init( I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

    //program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}
	else
	{
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Deinitializes the I2C peripheral
 *
 * @param[in]   pI2Cx   Reference to the I2C definition structure
 *
 ******************************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}

	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}

	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}

}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Gets flag status from the SR1 register
 *
 * @param[in]   pI2Cx   	Reference to the I2C definition structure
 * @param[in]   FlagName   	flag name
 *
 * @return uint8_t
 * @retval FLAG_SET
 * @retval FLAG_RESET
 ******************************************************************************/
uint8_t I2C_GetFlagStatus	(I2C_RegDef_t *pI2Cx
							, uint32_t FlagName
							)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/********************************************************************************
 * @ingroup api_i2c
 * @brief Sends data as master in blocking mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]	pTxbuffer	Reference to the TX buffer
 * @param[in]	Len			Length of the TX buffer
 * @param[in]	SlaveAddr	Slave address
 * @param[in]	Sr			Flag for stop condition bit
 *
 ******************************************************************************/
void I2C_MasterSendData ( I2C_Handle_t *pI2CHandle
						, uint8_t *pTxbuffer
						, uint32_t Len
						, uint8_t SlaveAddr
						, uint8_t Sr
						)
{
	// generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//confirm that start generation is completed
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//confirm that address phase is completed
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );

	//clear the ADDR flag according to its software sequenc
	I2C_ClearADDRFlag(pI2CHandle);

	//send the data until len becomes 0

	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//when Len becomes zero, wait for TXE=1 and BTF=1 before generating the STOP condition

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) );

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF) );


	//generate STOP condition and master need not to wait for the completion of stop condition.
	if(Sr == I2C_DISABLE_SR )
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Receives data as master in blocking mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]	pTxbuffer	Reference to the RX buffer
 * @param[in]	Len			Length of the RX buffer
 * @param[in]	SlaveAddr	Slave address
 * @param[in]	Sr			Flag for stop condition bit
 *
 ******************************************************************************/
void I2C_MasterReceiveData	( I2C_Handle_t *pI2CHandle
							, uint8_t *pRxBuffer
							, uint8_t Len
							, uint8_t SlaveAddr
							, uint8_t Sr
							)
{

	//generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//confirm that start generation is completed
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//wait until address phase is completed
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );

	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR )
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}

    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR )
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}

	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Manages ACKing of I2C peripheral
 *
 * @param[in]   pI2Cx   Reference to the I2C definition structure
 * @param[in]	EnorDi	Enable or disable macro
 *
 ******************************************************************************/
void I2C_ManageAcking	( I2C_RegDef_t *pI2Cx
						, uint8_t EnorDi
						)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Enables or disables the corresponding IRQ
 *
 * @param[in]   IRQNumber   IRQ number
 * @param[in]	EnorDi		Enable or disable macro
 *
 ******************************************************************************/
void I2C_IRQInterruptConfig ( uint8_t IRQNumber
							, uint8_t EnorDi
							)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Configures the IRQ priority
 *
 * @param[in]   IRQNumber   IRQ number
 * @param[in]	IRQPriority	IRQ priority
 *
 ******************************************************************************/
void I2C_IRQPriorityConfig	( uint8_t IRQNumber
							, uint32_t IRQPriority
							)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_IPR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Sends data as master in non-blocking mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]	pTxbuffer	Reference to the TX buffer
 * @param[in]	Len			Length of the TX buffer
 * @param[in]	SlaveAddr	Slave address
 * @param[in]	Sr			Flag for stop condition bit
 *
 * @return uint8_t
 * @retval I2C_BUSY_IN_TX	Currently sending data
 * @retval I2C_BUSY_IN_RX	Currently receiving data
 * @retbal I2C_READY		Currently ready for data transfer
 ******************************************************************************/
uint8_t I2C_MasterSendDataIT	( I2C_Handle_t *pI2CHandle
								, uint8_t *pTxBuffer
								, uint32_t Len
								, uint8_t SlaveAddr
								, uint8_t Sr
								)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Receives data as master in non-blocking mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]	pTxbuffer	Reference to the RX buffer
 * @param[in]	Len			Length of the RX buffer
 * @param[in]	SlaveAddr	Slave address
 * @param[in]	Sr			Flag for stop condition bit
 *
 * @return uint8_t
 * @retval I2C_BUSY_IN_TX	Currently sending data
 * @retval I2C_BUSY_IN_RX	Currently receiving data
 * @retbal I2C_READY		Currently ready for data transfer
 ******************************************************************************/
uint8_t I2C_MasterReceiveDataIT	( I2C_Handle_t *pI2CHandle
								, uint8_t *pRxBuffer
								, uint8_t Len
								, uint8_t SlaveAddr
								, uint8_t Sr
								)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Closes receiving data process
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
void I2C_CloseReceiveData( I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Closes sending data process
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
void I2C_CloseSendData( I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Perfoms sending data process in slave mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]   data  		data
 *
 ******************************************************************************/
void I2C_SlaveSendData	( I2C_RegDef_t *pI2C
						, uint8_t data
						)
{
	pI2C->DR = data;
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Perfoms receiving data process in slave mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]   data  		data
 *
 * @return uint8_t
 * @retval data		0x0 - 0xFF Data
 ******************************************************************************/
uint8_t I2C_SlaveReceiveData( I2C_RegDef_t *pI2C)
{
    return (uint8_t) pI2C->DR;
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Handles the interrupt event
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1, temp2, temp3;

	temp1   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;
	temp2   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN) ;

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);

	if(temp1 && temp3)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);

	if(temp1 && temp3)
	{
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);

	if(temp1 && temp3)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) )
			{
				if(pI2CHandle->TxLen == 0 )
				{
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					I2C_CloseSendData(pI2CHandle);

					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				}
			}

		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			;
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);

	if(temp1 && temp3)
	{
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);

	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);

	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Handles the interrupt error
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
void I2C_ER_IRQHandling( I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);

	if(temp1  && temp2 )
	{
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );

	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}


	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);

	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);

	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);

	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Application event callback function
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]   AppEv  		Application event macro
 *
 ******************************************************************************/
__attribute__((weak)) void I2C_ApplicationEventCallback	( I2C_Handle_t *pI2CHandle
									, uint8_t AppEv
									)
{
	/* This is a week implementation. The application may override this function. */
}

static void I2C_GenerateStartCondition( I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite	( I2C_RegDef_t *pI2Cx
											, uint8_t SlaveAddr
											)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead (I2C_RegDef_t *pI2Cx
										, uint8_t SlaveAddr
										)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag( I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;

	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		//device is in slave mode
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

static void I2C_MasterHandleRXNEInterrupt( I2C_Handle_t *pI2CHandle )
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//then close the I2C data reception and notify the application

		//generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}

static void I2C_MasterHandleTXEInterrupt( I2C_Handle_t *pI2CHandle )
{
	if(pI2CHandle->TxLen > 0)
	{
		//load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//decrement the TxLen
		pI2CHandle->TxLen--;

		//Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

