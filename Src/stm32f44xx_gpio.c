/******************************************************************************
** Copyright (c) 2021 Mentalibor. All rights reserved. Email: mentalibor@gmail.com
**
** Name
** stm32f44xx_gpio.c
**
** Purpose
** GPIO driver source file for the STM32F44XX
**
** Revision
** 08-Feb-2021 (Mentalibor) Initial version.
**
******************************************************************************/

#include "stm32f44xx_gpio.h"

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]   pGPIOx  Reference to the GPIO definition structure
 * @param[in]   EnorDi  Enable or disable macro
 *
 ******************************************************************************/
void GPIO_PerClockControl 	(GPIO_RegDef_t*	pGPIOx
							, uint8_t 		EnorDi
							)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}

		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}

		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}

		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}

		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}

		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}

		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}

		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else {
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}

		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}

		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}

		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}

		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}

		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}

		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}

		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Initializes the given GPIO peripheral
 *
 * @param[in]   pGPIOHandler  Reference to the GPIO handler
 *
 ******************************************************************************/
void GPIO_Init (GPIO_Handler_t *pGPIOHandler)
{
	uint32_t temp = 0;

	/*enable the clock*/
	GPIO_PerClockControl(pGPIOHandler->pGPIOx, ENABLE);


	/*configure the pin mode*/
	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandler->pGPIOx->MODER &= ~(0x3 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandler->pGPIOx->MODER |= temp; //setting
	}
	else
	{
		if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_INT_FALL_EDGE)
		{
			/*configure the FTSR for the corresponding GPIO pin*/
			EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
			/*clear the RTSR for the corresponding GPIO pin*/
			EXTI->RTSR &= ~(1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_INT_RISE_EDGE)
		{
			/*configure the RTSR for the corresponding GPIO pin*/
			EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
			/*clear the FTSR for the corresponding GPIO pin*/
			EXTI->FTSR &= ~(1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_INT_RISE_FALL_EDGE)
		{
			/*configure the FTSR for the corresponding GPIO pin*/
			EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
			/*configure the RTSR for the corresponding GPIO pin*/
			EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
		}

		/*configure GPIO port selection in SYSCFG EXTICR*/
		uint8_t temp1 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandler->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);

		/*enable the EXTI interrupt delivery using IMR*/
		EXTI->IMR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	}

	/*configure the pin speed*/
	temp = 0;
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->OSPEEDER &= ~(0x3 << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandler->pGPIOx->OSPEEDER |= temp;

	/*configure the pull up and pull down configuration*/
	temp = 0;
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandler->pGPIOx->PUPDR |= temp;


	/*configure the output type*/
	temp = 0;
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandler->pGPIOx->OTYPER |= temp;

	/*configure alternate mode if necessary*/
	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTERNAT)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandler->pGPIOx->AFR[temp1] &= ~(0xF << 4*temp2);
		pGPIOHandler->pGPIOx->AFR[temp1] |= pGPIOHandler->GPIO_PinConfig.GPIO_PinAltFunMode << 4*temp2;
	}
}

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Denitializes the given GPIO peripheral
 *
 * @param[in]   pGPIOHandler  Reference to the GPIO handler
 *
 ******************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}

	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}

	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}

	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}

	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}

	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}

	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}

	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Reads from the given GPIO pin
 *
 * @param[in]   pGPIOx  	Reference to the GPIO definition structure
 * @param[in]	PinNumber 	Number of GPIO pin
 *
 * @return uint8_t
 * @retval 0 - low state
 * @retval 1 - high state *
 ******************************************************************************/
uint8_t GPIO_ReadFromInputPin 	(GPIO_RegDef_t *pGPIOx
								, uint8_t PinNumber
								)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Reads from the given GPIO port
 *
 * @param[in]   pGPIOx  	Reference to the GPIO definition structure
 *
 * @return uint16_t
 * @retval 0x0 - 0xFFFF - port pin [0,1,x.,15] states
 ******************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);
	return value;
}

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Writes a value to the GPIO pin
 *
 * @param[in]   pGPIOx  	Reference to the GPIO definition structure
 * @param[in]   PinNumber	GPIO pin number
 * @param[in]	Value		Set/Reset values
 *  ******************************************************************************/
void GPIO_WriteToOutputPin	(GPIO_RegDef_t *pGPIOx
							, uint8_t PinNumber
							, uint8_t Value
							)
{
	if (Value  == GPIO_PIN_SET)
	{
		//write 1
		pGPIOx->ODR |= 1 << PinNumber;
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Writes a value to the GPIO pin
 *
 * @param[in]   pGPIOx  	Reference to the GPIO definition structure
 * @param[in]	Value		0x0 - 0xFFFF
 *  ******************************************************************************/
void GPIO_WriteToOutputPort	(GPIO_RegDef_t *pGPIOx
							, uint16_t Value
							)
{
	pGPIOx->ODR = Value;
}

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Toggles the state of the given GPIO pin
 *
 * @param[in]   pGPIOx  	Reference to the GPIO definition structure
 * @param[in]	PinNumber 	Number of GPIO pin
 *  ******************************************************************************/
void GPIO_ToggleOutputPin	(GPIO_RegDef_t *pGPIOx
							, uint8_t PinNumber
							)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Enables or disables the interrupt registers for the corresponding GPIO
 * 		  port
 *
 * @param[in]   IRQ_Number  	IRQ number
 * @param[in]	EnorDi		 	enable or disable macro
 *  ******************************************************************************/
void GPIO_IRQInterruptConfig 	(uint8_t IRQ_Number
								, uint8_t EnorDi
								)
{
	if (EnorDi == ENABLE)
	{
		if (IRQ_Number <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQ_Number);
		}
		else if(IRQ_Number > 31 && IRQ_Number < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQ_Number % 32);
		}
		else if (IRQ_Number >= 64 && IRQ_Number < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << IRQ_Number % 64);
		}
	}
	else
	{
		if (IRQ_Number <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQ_Number);
		}
		else if(IRQ_Number > 31 && IRQ_Number < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQ_Number % 32);
		}
		else if (IRQ_Number >= 64 && IRQ_Number < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << IRQ_Number % 64);
		}
	}
}

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Configures the interrupt priority register
 *
 * @param[in]   IRQ_Number  	IRQ number
 * @param[in]	IRQ_Priority	IRQ priority value
 *  ******************************************************************************/
void GPIO_IRQPriorityConfig	(uint8_t IRQ_Number
							, uint32_t  IRQ_Priority
							)
{
	uint8_t iprx = (IRQ_Number / 4);
	uint8_t iprx_section = (IRQ_Number % 4);

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx) |=  (IRQ_Priority << shift_amount);
}

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Clears the PR register of the EXTI after invoking interrupt
 *
 * @param[in]   PinNumber  		GPIO pin number
 *  ******************************************************************************/
void GPIO_IRQHandling (uint8_t PinNumber)
{
	//clear the PR register of the EXTI according to the corresponding pin
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
