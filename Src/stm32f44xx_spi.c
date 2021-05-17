/******************************************************************************
** Copyright (c) 2021 Mentalibor. All rights reserved. Email: mentalibor@gmail.com
**
** Name
** stm32f44xx_spi.c
**
** Purpose
** SPI driver for the STM32F44XX
**
** Revision
** 08-Feb-2021 (Mentalibor) Initial version.
**
******************************************************************************/

#include "stm32f44xx.h"

/*******************************************************************************
 * @ingroup api_spi
 * @brief Enables or disables the SPI peripheral clock
 *
 * @param[in]   pSPIx   Reference to the SPI definition structure
 * @param[in]   EnorDi  ENABLE/DISABLE macro
 *
 ******************************************************************************/
void SPI_PerClockControl	( SPI_RegDef_t *pSPIx
							, uint8_t EnorDi
							)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}

		else if (pSPIx == SPI2_I2S2)
		{
			SPI2_I2S2_PCLK_EN();
		}

		else if (pSPIx == SPI3_I2S3)
		{
			SPI2_I2S2_PCLK_EN();
		}

		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else {
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}

		else if (pSPIx == SPI2_I2S2)
		{
			SPI2_I2S2_PCLK_DI();
		}

		else if (pSPIx == SPI3_I2S3)
		{
			SPI2_I2S2_PCLK_DI();
		}

		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Initializes the SPI peripheral
 *
 * @param[in]   pSPIHandler  Reference to the SPI handler
 *
 ******************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandler)
{
	uint32_t tempreg = 0;

	//enable the clock
	SPI_PerClockControl(pSPIHandler->pSPIx, ENABLE);

	//configure the device mode
	tempreg |= pSPIHandler->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//configure the bus config
	if (pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be enabled
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandler->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//configure the dff
	tempreg |= pSPIHandler->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//configure the CPOL
	tempreg |= pSPIHandler->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//configure the CPHA
	tempreg |= pSPIHandler->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//configure the SSM
	tempreg |= pSPIHandler->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandler->pSPIx->CR1 = tempreg;
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Deinitializes the SPI peripheral
 *
 * @param[in]   pSPIx   Reference to the SPI definition structure
 *
 ******************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}

	else if (pSPIx == SPI2_I2S2)
	{
		SPI2_I2S2_REG_RESET();
	}

	else if (pSPIx == SPI3_I2S3)
	{
		SPI3_I2S3_REG_RESET();
	}

	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Enables or disables the SPI peripheral
 *
 * @param[in]   pSPIx   Reference to the SPI definition structure
 * @param[in]   EnorDi  ENABLE/DISABLE macro
 *
 ******************************************************************************/
void SPI_PeripheralControl	( SPI_RegDef_t *pSPIx
							, uint8_t EnOrDi
							)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Configures the SSI of the	corresponding SPI peripheral
 *
 * @param[in]   pSPIx   Reference to the SPI definition structure
 * @param[in]   EnorDi  ENABLE/DISABLE macro
 *
 ******************************************************************************/
void  SPI_SSIConfig	( SPI_RegDef_t *pSPIx
					, uint8_t EnOrDi
					)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Configures the SSOE of the corresponding SPI peripheral
 *
 * @param[in]   pSPIx   Reference to the SPI definition structure
 * @param[in]   EnorDi  ENABLE/DISABLE macro
 *
 ******************************************************************************/
void  SPI_SSOEConfig ( SPI_RegDef_t *pSPIx
					 , uint8_t EnOrDi
					 )
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}


}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Reads given flag from the SR status register of the corresponding SPI
 * 		  peripheral
 *
 * @param[in]   pSPIx   	Reference to the SPI definition structure
 * @param[in]   FlagName  	status flag macro name
 *
 * @return uint8_t
 * @retval FLAG_SET
 * @retval FLAG_RESET
 *
 ******************************************************************************/
uint8_t SPI_GetFlagStatus	( SPI_RegDef_t *pSPIx
							, uint32_t FlagName
							)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Sends data in blocking mode
 *
 * @param[in]   pSPIx   	Reference to the SPI definition structure
 * @param[in]	pTxbuffer	Reference to the TX buffer
 * @param[in]	length		Length of the TX buffer
 *
 ******************************************************************************/
void SPI_SendData	( SPI_RegDef_t *pSPIx
					, uint8_t *pTxBuffer
					, uint32_t length
					)
{
	while (length > 0) {
		//wait until the TXE flag indicates empty status
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//if the data frame format is 16-bit
		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) ) {
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			length--;
			length--;
			(uint16_t*)pTxBuffer++;
		}
		else {
			pSPIx->DR = *(pTxBuffer);
			length--;
			pTxBuffer++;
		}
	}
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Receives data in blocking mode
 *
 * @param[in]   pSPIx   	Reference to the SPI definition structure
 * @param[in]	pRxBuffer	Reference to the RX buffer
 * @param[in]	length		Length of the RX buffer
 *
 ******************************************************************************/
void SPI_ReceiveData ( SPI_RegDef_t *pSPIx
					 , uint8_t *pRxBuffer
					 , uint32_t length
					 )
{
	while (length > 0) {
		//wait until the RXNE flag indicates non-empty status
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//if the data frame format is 16-bit
		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) ) {
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			length--;
			length--;
			(uint16_t*)pRxBuffer++;
		}
		//if the data frame format is 8-bit
		else {
			*(pRxBuffer) = pSPIx->DR;
			length--;
			pRxBuffer++;
		}
	}
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Sends data in non-blocking mode
 *
 * @param[in]   pSPIHandler Reference to the SPI handler
 * @param[in]	pTxbuffer	Reference to the TX buffer
 * @param[in]	length		Length of the TX buffer
 *
 * @return uint8_t
 * @retval SPI_BUSY_IN_RX
 * @retval SPI_BUSY_IN_TX
 * @retval SPI_READY
 ******************************************************************************/
uint8_t SPI_SendDataInterrupt	( SPI_Handle_t *pSPIHandle
								, uint8_t *pTxBuffer
								, uint32_t length
								)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_RX) {
		//store the Tx buffer
		pSPIHandle->pTxBuffer = pTxBuffer;

		//store length info
		pSPIHandle->TxLen = length;

		//mark the SPi state as busy in transmission mode
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;

}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Receives data in non-blocking mode
 *
 * @param[in]   pSPIHandler Reference to the SPI handler
 * @param[in]	pRxbuffer	Reference to the RX buffer
 * @param[in]	length		Length of the RX buffer
 *
 * @return uint8_t
 * @retval SPI_BUSY_IN_RX
 * @retval SPI_BUSY_IN_TX
 * @retval SPI_READY
 ******************************************************************************/
uint8_t SPI_ReceiveDataInterrupt	( SPI_Handle_t *pSPIHandle
									, uint8_t *pRxBuffer
									, uint32_t length
									)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_TX) {
		//store the Tx buffer
		pSPIHandle->pRxBuffer = pRxBuffer;

		//store length info
		pSPIHandle->RxLen = length;

		//mark the SPi state as busy in transmission mode
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}

/*
 * IRQ Configuration and ISR handling
 */
/*******************************************************************************
 * @ingroup api_spi
 * @brief Enables or disables the corresponding IRQ
 *
 * @param[in]   IRQNumber   IRQ number
 * @param[in]	EnorDi		Enable or disable macro
 *
 ******************************************************************************/
void SPI_IRQInterruptConfig ( uint8_t IRQNumber
							, uint8_t EnorDi
							)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Configures the IRQ priority
 *
 * @param[in]   IRQNumber   IRQ number
 * @param[in]	IRQPriority	IRQ priority
 *
 ******************************************************************************/
void SPI_IRQPriorityConfig	( uint8_t IRQNumber
							, uint32_t IRQPriority
							)
{
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprx_section = (IRQNumber % 4);

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx) |=  (IRQPriority << shift_amount);
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Handles the interrupt
 *
 * @param[in]   pHandle  Reference to the SPI handler
 *
 ******************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1 , temp2;
	//first lets check for TXE
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		// check the DFF bit in CR1
		if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTxBuffer++;
		}
		else
		{
			//8 bit DFF
			pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}

		if(! pSPIHandle->TxLen)
		{
			//this prevents interrupts from setting up of TXE flag
			SPI_CloseTransmisson(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}
	}

	// check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//do rxing as per the dff
		if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
		{
			//16 bit
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRxBuffer++;
			pSPIHandle->pRxBuffer++;

		}else
		{
			//8 bit
			*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}

		if(! pSPIHandle->RxLen)
		{
			//reception is complete
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}
	}

	// check for ovr flag
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		uint8_t temp3;
		//1. clear the ovr flag
		if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
		{
			temp3 = pSPIHandle->pSPIx->DR;
			temp3 = pSPIHandle->pSPIx->SR;
		}
		(void)temp3;
		//2. inform the application
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
	}
}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Closes the SPI transmission
 *
 * @param[in]   pSPIHandle  Reference to the SPI handler
 *
 ******************************************************************************/
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Closes the SPI reception
 *
 * @param[in]   pSPIHandle  Reference to the SPI handler
 *
 ******************************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

/*******************************************************************************
 * @ingroup api_spi
 * @brief Clears the OVR flag
 *
 * @param[in]   pSPIx   	Reference to the SPI definition structure
 *
 ******************************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


/*******************************************************************************
 * @ingroup api_spi
 * @brief Application event callback function
 *
 * @param[in]   pSPIHandle  Reference to the SPI handler
 * @param[in]   AppEv  		application event macro
 *
 ******************************************************************************/
__attribute__((weak)) void SPI_ApplicationEventCallback	( SPI_Handle_t *pSPIHandle
														, uint8_t AppEv
														)
{

	//This is a weak implementation . the user application may override this function.
}
